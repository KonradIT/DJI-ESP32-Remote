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

/* record_ctrl — NOTE the inversion versus the media engine, where [01] starts. */
#define RSDK_RECORD_START         0x00
#define RSDK_RECORD_STOP          0x01

extern uint32_t g_device_id;   /* this remote's own id, set at boot from NVS */

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

    esp_err_t ret = data_write_without_response(slot, seq, frame, frame_len);
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
 * The QS ("quick switch") key report — the physical mode button on a DJI
 * remote. The camera picks the next mode itself, so unlike the media engine
 * there is nothing to look up and no dependence on having seen a status push.
 * This is the behaviour the mode screen has always had on Action bodies; it is
 * only moving behind the interface so a Nano stops receiving it.
 */
static esp_err_t rsdk_mode_cycle(int slot)
{
    return command_logic_send_key_report_for_slot(slot, 0x02, 0x01, 0x00);
}

static esp_err_t rsdk_session_open(int slot)
{
    /* connect_logic performs the 0x00/0x19 connection request, which is also
     * what identifies the body as R-SDK in the first place (its reply carries
     * the device_id). Nothing further is needed here. */
    (void)slot;
    return ESP_OK;
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
