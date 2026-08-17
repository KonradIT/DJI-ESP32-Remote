/*
 * R-SDK engine — Osmo Action 4 / 5 Pro / 6, Osmo 360.
 *
 * DJI's documented Osmo GPS-Controller SDK. Frames start 0xAA and are built by
 * protocol_create_frame() from protocol/dji_protocol_parser.c; the descriptor
 * table serialises the packed structs in dji_protocol_data_structures.h.
 *
 * Restored from the pre-DUML implementation on `main`, with the send path
 * renamed (rsdk_send) so it no longer collides with the media engine's
 * send_command() — both were non-static with different arity, which is a
 * straight linker failure if you simply drop both trees in together.
 */

#include "camera_engine.h"
#include "command_logic.h"   /* key report helper, shared with the media engine */
#include "dji_protocol_parser.h"
#include "dji_protocol_data_structures.h"
#include "enums_logic.h"
#include "data.h"
#include "ble.h"
#include "ui.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

#define TAG "ENG_RSDK"

/* R-SDK command set / ids (docs/protocol_data_segment.md) */
#define RSDK_CMDSET_SESSION       0x00
#define RSDK_CMDID_CONN_REQ       0x19
#define RSDK_CMDSET_CAMERA        0x1D
#define RSDK_CMDID_MODE_SWITCH    0x04
#define RSDK_CMDID_RECORD_CTRL    0x03
#define RSDK_CMDID_STATUS_SUBSCRIBE 0x05   /* on cmd set 0x1D */
#define RSDK_CMDID_KEY_REPORT     0x11   /* on cmd set 0x00, not 0x1D */

/* Key codes carried by 0x00/0x11. QS is context-dependent ON THE CAMERA: a
 * short press cycles the shooting mode when idle, and drops a highlight marker
 * while recording. Both callers therefore send the same code — that is the
 * camera's behaviour, not a mix-up. */
#define RSDK_KEY_QS               0x02
#define RSDK_KEY_SNAPSHOT         0x03
#define RSDK_KEY_MODE_EVENTS      0x01   /* report key events, not up/down */
#define RSDK_KEY_SHORT_PRESS      0x00

/* record_ctrl — NOTE the inversion versus the media engine, where [01] starts. */
#define RSDK_RECORD_START         0x00
#define RSDK_RECORD_STOP          0x01

/* This remote's own protocol identity, defined in main/ui.c. Not declared in a
 * header there, so the handshake picks them up by extern like g_device_id. */
extern uint32_t g_device_id;
extern uint8_t  g_mac_addr_len;
extern int8_t   g_mac_addr[6];
extern uint32_t g_fw_version;
extern uint8_t  g_verify_mode;

static uint16_t rsdk_seq(void)
{
    static uint16_t s_seq = 0;
    return ++s_seq;
}

/*
 * Build and write one R-SDK frame. Fire-and-forget by design: the ack only says
 * the command was accepted, never that the state changed — the recording bit
 * and mode are trusted solely from the camera's own status push. That is true
 * of BOTH protocols, so the UI can treat them the same way.
 */
/*
 * Same, but with the frame's cmd_type and seq chosen by the caller.
 *
 * The handshake needs both: its opening request is CMD_WAIT_RESULT, and its
 * closing frame is an ACK that must carry the SEQ THE CAMERA CHOSE, not one of
 * ours — an ack with a fresh seq answers nothing.
 */
static esp_err_t rsdk_send_typed(int slot, uint8_t cmd_set, uint8_t cmd_id,
                                 uint8_t cmd_type, const void *body, uint16_t seq,
                                 bool expect_reply)
{
    if (slot < 0 || slot >= NUM_CAMERAS || !ble_is_camera_connected(slot)) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t   frame_len = 0;
    uint8_t *frame = protocol_create_frame(cmd_set, cmd_id, cmd_type, body, seq, &frame_len);
    if (frame == NULL) {
        return ESP_FAIL;
    }
    /*
     * expect_reply picks the write that REGISTERS the seq. Only
     * data_write_with_response() allocates a tracking entry; without one the
     * camera's answer arrives, finds nothing waiting, and is dropped — which
     * looked exactly like the camera never answering. Confirmed on an OA6: the
     * reply was in the log 1.9 s before our own timeout fired.
     */
    esp_err_t ret = expect_reply
                        ? data_write_with_response(slot, seq, frame, frame_len)
                        : data_write_without_response(slot, seq, frame, frame_len);
    free(frame);
    return ret;
}

static esp_err_t rsdk_send_set(int slot, uint8_t cmd_set, uint8_t cmd_id, const void *body)
{
    if (slot < 0 || slot >= NUM_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ble_is_camera_connected(slot)) {
        ESP_LOGW(TAG, "Camera %d: not connected", slot);
        return ESP_ERR_INVALID_STATE;
    }

    size_t   frame_len = 0;
    uint16_t seq       = rsdk_seq();
    uint8_t *frame = protocol_create_frame(cmd_set, cmd_id,
                                           CMD_RESPONSE_OR_NOT, body, seq,
                                           &frame_len);
    if (frame == NULL) {
        ESP_LOGE(TAG, "Camera %d: failed to build 0x%02X/0x%02X",
                 slot, cmd_set, cmd_id);
        return ESP_FAIL;
    }

    /* 0xFFF5 reports props=0x36 (WRITE_NO_RSP) on Action bodies just as it does
     * on the Nano — hardware-checked — so write without response on both. */
    esp_err_t ret = data_write_without_response(slot, seq, frame, frame_len);
    free(frame);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Camera %d: 0x%02X/0x%02X write failed: %s",
                 slot, cmd_set, cmd_id, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t rsdk_send(int slot, uint8_t cmd_id, const void *body)
{
    return rsdk_send_set(slot, RSDK_CMDSET_CAMERA, cmd_id, body);
}

/*
 * Ask the body to identify itself: R-SDK Connection Request 0x00/0x19.
 *
 * This is the whole reason no advertisement-id catalogue is needed for the
 * Action family — the reply carries the camera's own device_id
 * (0xFF33/44/55/66), which names the exact model. A body that does not speak
 * R-SDK simply never answers, and silence is a legitimate answer here: it means
 * "not R-SDK", not "try again with different bytes".
 *
 * Safe to send to an unknown body: it is a documented request with no side
 * effects, unlike sweeping an opcode's value space (which froze a Nano).
 */
esp_err_t rsdk_probe_identity(int slot, uint32_t *out_device_id)
{
    if (out_device_id == NULL) return ESP_ERR_INVALID_ARG;
    *out_device_id = 0;

    if (slot < 0 || slot >= NUM_CAMERAS || !ble_is_camera_connected(slot)) {
        return ESP_ERR_INVALID_STATE;
    }

    connection_request_command_frame body = {0};
    body.device_id   = g_device_id;
    body.mac_addr_len = 0;
    body.fw_version  = 0;
    body.conidx      = 0;
    body.verify_mode = 0;      /* reconnect/verify — not a fresh pairing */
    body.verify_data = 0;

    size_t   frame_len = 0;
    uint16_t seq = rsdk_seq();
    uint8_t *frame = protocol_create_frame(RSDK_CMDSET_SESSION, RSDK_CMDID_CONN_REQ,
                                           CMD_RESPONSE_OR_NOT, &body, seq, &frame_len);
    if (frame == NULL) return ESP_FAIL;

    /* with_response, because it is what registers the seq — see rsdk_send_typed.
     * With the plain write this probe could never succeed: the camera's reply
     * had nowhere to land, so every Action body was reported as "not R-SDK". */
    esp_err_t ret = data_write_with_response(slot, seq, frame, frame_len);
    free(frame);
    if (ret != ESP_OK) return ret;

    void  *rsp = NULL;
    size_t rsp_len = 0;
    if (data_wait_for_result_by_seq(seq, 1200, &rsp, &rsp_len) != ESP_OK || rsp == NULL) {
        ESP_LOGI(TAG, "Camera %d: no 0x00/0x19 reply — not an R-SDK body", slot);
        return ESP_ERR_NOT_FOUND;
    }

    if (rsp_len >= sizeof(connection_request_response_frame)) {
        const connection_request_response_frame *r = rsp;
        *out_device_id = r->device_id;
        ESP_LOGI(TAG, "Camera %d: R-SDK identity device_id=0x%04X",
                 slot, (unsigned int)r->device_id);
    } else {
        ESP_LOGW(TAG, "Camera %d: 0x00/0x19 reply too short (%u B)",
                 slot, (unsigned)rsp_len);
        ret = ESP_ERR_INVALID_SIZE;
    }
    free(rsp);
    return (*out_device_id != 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t rsdk_record(int slot, uint8_t ctrl)
{
    record_control_command_frame_t body = {
        .device_id   = g_device_id,
        .record_ctrl = ctrl,
        .reserved    = {0},
    };
    ESP_LOGI(TAG, "Camera %d: %s recording (0x1D/0x03 ctrl=%u)",
             slot, ctrl == RSDK_RECORD_START ? "start" : "stop", ctrl);
    return rsdk_send(slot, RSDK_CMDID_RECORD_CTRL, &body);
}

static esp_err_t rsdk_record_start(int slot) { return rsdk_record(slot, RSDK_RECORD_START); }
static esp_err_t rsdk_record_stop(int slot)  { return rsdk_record(slot, RSDK_RECORD_STOP); }

/*
 * R-SDK has NO dedicated shutter opcode. The camera decides photo-vs-record
 * from the mode it is already in, so a "take photo" is Record Control start
 * issued while in Photo mode. Callers must set the mode first; unlike the media
 * engine there is no wrong-state reply to catch the mistake, so getting the
 * order wrong silently starts a video recording instead.
 */
static esp_err_t rsdk_shoot_photo(int slot)
{
    ESP_LOGI(TAG, "Camera %d: shutter via record-start (photo mode assumed)", slot);
    return rsdk_record(slot, RSDK_RECORD_START);
}

static esp_err_t rsdk_set_mode(int slot, cam_mode_t mode)
{
    uint8_t m;
    switch (mode) {
        case CAM_MODE_SLOWMO:     m = 0x00; break;
        case CAM_MODE_VIDEO:      m = 0x01; break;
        case CAM_MODE_TIMELAPSE:  m = 0x02; break;
        case CAM_MODE_PHOTO:      m = 0x05; break;
        case CAM_MODE_HYPERLAPSE: m = 0x0A; break;
        case CAM_MODE_SUPERNIGHT: m = 0x28; break;
        default:
            ESP_LOGW(TAG, "Camera %d: refusing unmapped mode %d", slot, (int)mode);
            return ESP_ERR_INVALID_ARG;
    }

    camera_mode_switch_command_frame_t body = {
        .device_id = g_device_id,
        .mode      = m,
        .reserved  = {0},
    };
    ESP_LOGI(TAG, "Camera %d: set mode 0x%02X (0x1D/0x04)", slot, m);
    return rsdk_send(slot, RSDK_CMDID_MODE_SWITCH, &body);
}

/*
 * Key report — 0x00/0x11. The remote reports a button press and the CAMERA
 * decides what it means, which is why there is nothing to look up here.
 *
 * This used to route through command_logic_send_key_report_for_slot(), which
 * during the DUML-only port had been hollowed out into a translation shim that
 * emitted DUML 0x02/0xE1 regardless of key code. That sent a media frame to an
 * Action body for both mode cycling AND highlight — the exact cross-protocol
 * failure the engine split exists to prevent.
 */
esp_err_t rsdk_key_report(int slot, uint8_t key_code, uint8_t mode, uint8_t key_value)
{
    key_report_command_frame_t body = {
        .key_code  = key_code,
        .mode      = mode,
        .key_value = key_value,
    };
    ESP_LOGI(TAG, "Camera %d: key report 0x%02X (mode 0x%02X, value 0x%02X)",
             slot, key_code, mode, key_value);
    return rsdk_send_set(slot, RSDK_CMDSET_SESSION, RSDK_CMDID_KEY_REPORT, &body);
}

/* The physical mode button. Idle -> next mode; recording -> highlight marker. */
static esp_err_t rsdk_mode_cycle(int slot)
{
    return rsdk_key_report(slot, RSDK_KEY_QS, RSDK_KEY_MODE_EVENTS, RSDK_KEY_SHORT_PRESS);
}

/*
 * The R-SDK connection handshake — 0x00/0x19, four steps, BIDIRECTIONAL.
 *
 * Transcribed from a capture of the `main` firmware, which drives an OA6
 * correctly. Getting only the first half of this is why an Action camera
 * accepted our status subscribe with ret_code 00 and then pushed nothing: it
 * had never registered us as a controller.
 *
 *   1. us  -> cam  0x00/0x19 connection_request_command_frame, CMD_WAIT_RESULT
 *   2. cam -> us   connection_request_response_frame              (handshake ok)
 *   3. cam -> us   0x00/0x19 as its OWN REQUEST, carrying the camera's
 *                  device_id (0xFF55 on an OA6) and verify_mode 2. THIS is the
 *                  "GPS remote connected" prompt on the camera screen.
 *   4. us  -> cam  0x00/0x19 connection_request_response_frame, ACK_NO_RESPONSE,
 *                  echoing the seq the CAMERA chose in step 3.
 *
 * Step 3 is a human in the loop, hence the 30 s wait — matching the reference
 * rather than trimming it, since a shorter one races the user. It does block
 * this slot's connect; that is what the reference does too.
 */
static esp_err_t rsdk_handshake(int slot)
{
    camera_state_t *cam = &g_camera_states[slot];

    /* STEP 1 */
    connection_request_command_frame req = {
        .device_id    = g_device_id,
        .mac_addr_len = g_mac_addr_len,
        .fw_version   = g_fw_version,
        .conidx       = 0,
        .verify_mode  = g_verify_mode,
        .verify_data  = cam->verify_data,
    };
    memcpy(req.mac_addr, g_mac_addr, sizeof(req.mac_addr) < 6 ? sizeof(req.mac_addr) : 6);

    uint16_t seq = rsdk_seq();
    ESP_LOGI(TAG, "Camera %d: connection request (0x00/0x19, verify_mode=%u)",
             slot, (unsigned)g_verify_mode);
    if (rsdk_send_typed(slot, RSDK_CMDSET_SESSION, RSDK_CMDID_CONN_REQ,
                        CMD_WAIT_RESULT, &req, seq, true) != ESP_OK) {
        return ESP_FAIL;
    }

    /* STEP 2 — the camera's ack of our request. */
    void  *res = NULL;
    size_t res_len = 0;
    if (data_wait_for_result_by_seq(seq, 2000, &res, &res_len) != ESP_OK || res == NULL) {
        ESP_LOGW(TAG, "Camera %d: no reply to connection request", slot);
        return ESP_ERR_TIMEOUT;
    }
    free(res);

    /* STEP 3 — the camera's own request. Approval happens on the camera. */
    ESP_LOGI(TAG, "Camera %d: waiting for the camera to confirm the connection "
                  "(approve on the camera if prompted)", slot);
    void    *creq = NULL;
    size_t   creq_len = 0;
    uint16_t cam_seq = 0;
    if (data_wait_for_result_by_cmd(RSDK_CMDSET_SESSION, RSDK_CMDID_CONN_REQ,
                                    30000, &cam_seq, &creq, &creq_len) != ESP_OK ||
        creq == NULL || creq_len < sizeof(connection_request_command_frame)) {
        ESP_LOGW(TAG, "Camera %d: camera never sent its connection request", slot);
        free(creq);
        return ESP_ERR_TIMEOUT;
    }

    const connection_request_command_frame *cr =
        (const connection_request_command_frame *)creq;
    uint32_t cam_id = cr->device_id;
    uint8_t  vmode  = cr->verify_mode;
    uint16_t vdata  = cr->verify_data;
    free(creq);

    cam->device_id = cam_id;
    const char *model = ui_get_camera_model_name(cam_id);
    strncpy(cam->model_name, model, sizeof(cam->model_name) - 1);
    cam->model_name[sizeof(cam->model_name) - 1] = '\0';
    ESP_LOGI(TAG, "Camera %d identified as %s (device_id 0x%04X, verify_mode %u)",
             slot, model, (unsigned)cam_id, (unsigned)vmode);

    if (vmode != 2) {
        ESP_LOGW(TAG, "Camera %d: unexpected verify_mode %u — not completing",
                 slot, (unsigned)vmode);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (vdata != 0) {
        ESP_LOGW(TAG, "Camera %d: connection refused (verify_data %u)",
                 slot, (unsigned)vdata);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* STEP 4 — accept, echoing the camera's seq. */
    connection_request_response_frame ack = {
        .device_id = g_device_id,
        .ret_code  = 0,
    };
    memset(ack.reserved, 0, sizeof(ack.reserved));
    ack.reserved[0] = cam->camera_reserved;

    ESP_LOGI(TAG, "Camera %d: accepting connection (ack seq 0x%04X)", slot, cam_seq);
    return rsdk_send_typed(slot, RSDK_CMDSET_SESSION, RSDK_CMDID_CONN_REQ,
                           ACK_NO_RESPONSE, &ack, cam_seq, false);
}

static esp_err_t rsdk_session_open(int slot)
{
    /*
     * Subscribe to the camera status push (0x1D/0x05).
     *
     * REQUIRED, and the reason an Action camera showed battery but never a
     * recording flag even after the SOF dispatcher landed: unlike a Nano —
     * which starts pushing 0x02/0x80 the moment notifications are enabled —
     * an R-SDK body sends 0x1D/0x02 only once asked. Without this the remote
     * can command the camera and read nothing back, so recording never
     * registers and the shutter never offers Stop.
     *
     * The subscribe call was deleted during the DUML-only port, correctly at
     * the time: DUML genuinely does not need it. It has to come back now that
     * both protocols run side by side. push_freq is in 0.1 Hz units and the
     * protocol only accepts 20 (= 2 Hz).
     */
    /*
     * The connection request comes FIRST. An OA6 accepts the subscribe with
     * ret_code 00 and then pushes nothing at all unless it has been through
     * 0x00/0x19 — being a registered controller is evidently what earns the
     * status stream, not the subscription on its own.
     *
     * This became necessary the moment identity started coming from the
     * advertisement: resolving the engine from 0x0018 meant route 3 never ran,
     * so the handshake that used to happen as a side effect of asking "who are
     * you?" silently stopped happening. Worth noting the probe only works at
     * all now that the SOF dispatcher exists — its 0xAA reply used to be
     * discarded, which is why it always reported "not an R-SDK body".
     */
    if (rsdk_handshake(slot) != ESP_OK) {
        ESP_LOGW(TAG, "Camera %d: R-SDK handshake incomplete — subscribing anyway, "
                      "but the camera will likely stay silent", slot);
    }

    camera_status_subscription_command_frame body = {
        .push_mode = PUSH_MODE_PERIODIC_WITH_STATE_CHANGE,
        .push_freq = PUSH_FREQ_2HZ,
        .reserved  = {0},
    };
    ESP_LOGI(TAG, "Camera %d: subscribing to status push (0x1D/0x05, 2 Hz)", slot);
    return rsdk_send(slot, RSDK_CMDID_STATUS_SUBSCRIBE, &body);
}

const camera_engine_t g_engine_rsdk = {
    .name         = "rsdk",
    .session_open = rsdk_session_open,
    .record_start = rsdk_record_start,
    .record_stop  = rsdk_record_stop,
    .shoot_photo  = rsdk_shoot_photo,
    .set_mode     = rsdk_set_mode,
    .mode_cycle   = rsdk_mode_cycle,
    /* GPS push and highlight are R-SDK-only. Highlight is further narrowed per
     * model at the call site (0xFF33/0xFF44/0xFF55 — highlight is unconfirmed
     * on the Osmo 360), which a per-engine bitmask cannot express on its own. */
    .caps         = CAM_CAP_GPS | CAM_CAP_HIGHLIGHT | CAM_CAP_SLEEP | CAM_CAP_EIS,
};
