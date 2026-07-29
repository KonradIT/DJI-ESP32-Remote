/*
 * Osmo Nano Remote Control - Command Logic Layer
 *
 * Camera control over the DUML-on-BLE protocol (see protocol/osmo_duml.h).
 * Frames are written to the camera's 0xFFF5 characteristic; the camera-control
 * command set (0x02) is fire-and-forget — the authoritative recording/mode
 * state comes back asynchronously via the status pushes handled in
 * status_logic.c, never from a command response.
 *
 * The public command_logic_* API keeps the signatures the UI layer depends on;
 * a non-NULL return means "command dispatched", not "camera confirmed".
 *
 * Command sourcing: MEDIA_PROTOCOL.md (Osmosis reverse-engineering project) +
 * an HCI snoop of DJI Mimo driving a real Osmo Nano.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ble.h"
#include "data.h"
#include "duml.h"
#include "osmo_duml.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"
#include "../gps/gps_reader.h"
#include "../main/ui.h"

/* Logging tag for ESP_LOG functions */
#define TAG "LOGIC_COMMAND"

/* Sequence counter. generate_seq() is called from multiple tasks (the UI
 * command task, the connect flow, and the 1 Hz keepalive task), so the
 * increment must be atomic — otherwise two frames can be issued with the same
 * seq and the notify handler (which matches responses purely by seq) delivers
 * a reply to the wrong waiter. */
uint16_t s_current_seq = 0;
static portMUX_TYPE s_seq_mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t generate_seq(void) {
    portENTER_CRITICAL(&s_seq_mux);
    uint16_t seq = ++s_current_seq;
    portEXIT_CRITICAL(&s_seq_mux);
    return seq;
}

/**
 * @brief Is the camera in this slot able to accept a control command right now?
 *
 * Camera-control frames (cmd set 0x02) are dropped while the camera sleeps.
 * Session/pairing/system frames are always allowed (they are what wake it).
 */
static bool camera_can_accept_commands(int camera_index, uint8_t dst) {
    if (dst != DUML_ADDR_CAMERA) {
        return true;   /* session / wifi / system frames are always allowed */
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    if (g_camera_states[camera_index].is_sleeping) {
        /* WARN, not DEBUG: at INFO level this dropped commands with no trace at
         * all, so a blocked press looked identical to a camera ignoring us. */
        ESP_LOGW(TAG, "Camera %d: control frame BLOCKED — camera flagged asleep", camera_index);
        return false;
    }
    return true;
}

/**
 * @brief Build a DUML frame and pace it onto the camera's 0xFFF5 write char
 */
static esp_err_t duml_write(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                            uint8_t flags, const uint8_t *payload, size_t payload_len,
                            uint16_t seq, bool track_seq) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera index: %d", camera_index);
        return ESP_ERR_INVALID_ARG;
    }
    if (!ble_is_camera_connected(camera_index)) {
        ESP_LOGE(TAG, "Camera %d: BLE not connected", camera_index);
        return ESP_ERR_INVALID_STATE;
    }
    if (!camera_can_accept_commands(camera_index, dst)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t frame[DUML_FRAME_OVERHEAD + 128];
    if (payload_len > sizeof(frame) - DUML_FRAME_OVERHEAD) {
        ESP_LOGE(TAG, "Payload too large: %zu", payload_len);
        return ESP_ERR_INVALID_SIZE;
    }
    size_t frame_len = duml_build(frame, sizeof(frame), dst, seq, flags,
                                  cmd_set, cmd_id, payload, payload_len);
    if (frame_len == 0) {
        ESP_LOGE(TAG, "Failed to build DUML frame");
        return ESP_FAIL;
    }

#if DEBUG_DUML_PACKETS
    ESP_LOGI(TAG, "TX cam%d dst=0x%02X flags=0x%02X cmd=0x%02X/0x%02X seq=0x%04X plen=%zu",
             camera_index, dst, flags, cmd_set, cmd_id, seq, payload_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, frame_len, ESP_LOG_INFO);
#endif

    if (track_seq) {
        return data_write_with_response(camera_index, seq, frame, frame_len);
    }
    return data_write_without_response(camera_index, seq, frame, frame_len);
}

esp_err_t osmo_send(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                    uint8_t flags, const uint8_t *payload, size_t payload_len) {
    return duml_write(camera_index, dst, cmd_set, cmd_id, flags,
                      payload, payload_len, generate_seq(), false);
}

esp_err_t duml_write_tracked(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                             const uint8_t *payload, size_t payload_len, uint16_t seq) {
    return duml_write(camera_index, dst, cmd_set, cmd_id, OSMO_FLAGS_REQUEST,
                      payload, payload_len, seq, true);
}

CommandResult send_command(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                           const uint8_t *payload, size_t payload_len, uint16_t seq, int timeout_ms) {
    CommandResult result = { NULL, 0 };

    esp_err_t ret = duml_write(camera_index, dst, cmd_set, cmd_id, OSMO_FLAGS_REQUEST,
                               payload, payload_len, seq, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera %d: send failed for 0x%02X/0x%02X: %s",
                 camera_index, cmd_set, cmd_id, esp_err_to_name(ret));
        return result;
    }

    void *structure_data = NULL;
    size_t structure_data_length = 0;
    ret = data_wait_for_result_by_seq(seq, timeout_ms, &structure_data, &structure_data_length);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Camera %d: no response for 0x%02X/0x%02X (seq=0x%04X): %s",
                 camera_index, cmd_set, cmd_id, seq, esp_err_to_name(ret));
        return result;
    }

    result.structure = structure_data;
    result.length = structure_data_length;
    return result;
}

/* ---- helper: allocate a one-byte {ret_code} style response struct --------- */
static void *alloc_ret_ok(size_t size) {
    uint8_t *p = calloc(1, size);   /* ret_code = 0 (success) in byte 0 */
    return p;
}

/* ==========================================================================
 * Camera control (cmd set 0x02 — fire-and-forget; state confirmed via pushes)
 * ========================================================================== */

/**
 * @brief Map the UI's camera_mode_t (R-SDK values) to the DUML 0x02/0x02 mode
 */
static uint8_t osmo_mode_from_camera_mode(camera_mode_t mode) {
    switch (mode) {
        case CAMERA_MODE_PHOTO:        return OSMO_MODE_PHOTO;
        case CAMERA_MODE_NORMAL:       return OSMO_MODE_VIDEO;
        case CAMERA_MODE_SLOW_MOTION:  return OSMO_MODE_SLOWMO;
        case CAMERA_MODE_TIMELAPSE:    return OSMO_MODE_TIMELAPSE;
        default:                       return OSMO_MODE_VIDEO;
    }
}

/* Reflect the mode we just commanded back into camera_state_t so the UI icon
 * updates immediately (the DUML camera set does not push camera_mode). */
static camera_mode_t camera_mode_from_osmo(uint8_t osmo_mode) {
    switch (osmo_mode) {
        case OSMO_MODE_PHOTO:     return CAMERA_MODE_PHOTO;
        case OSMO_MODE_VIDEO:     return CAMERA_MODE_NORMAL;
        case OSMO_MODE_SLOWMO:    return CAMERA_MODE_SLOW_MOTION;
        case OSMO_MODE_TIMELAPSE: return CAMERA_MODE_TIMELAPSE;
        default:                  return CAMERA_MODE_NORMAL;
    }
}

/*
 * Modes 0 and 1 are NOT mode switches on this body — they are the record
 * control (see the record section below: [01] starts, [00] stops).  Sending
 * them from a mode-switch press would start or stop a recording behind the
 * user's back, so they are refused here; recording goes through
 * command_logic_start_record/stop_record.  What 2..5 actually do is still
 * unverified on the Nano.
 */
static esp_err_t set_osmo_mode(int camera_index, uint8_t osmo_mode) {
    if (osmo_mode == OSMO_MODE_PHOTO || osmo_mode == OSMO_MODE_VIDEO) {
        ESP_LOGW(TAG, "Camera %d: refusing set-mode %d — that value is the record "
                      "control on this camera, not a mode", camera_index, osmo_mode);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "Camera %d: set mode osmo=%d via 0x02/0x02", camera_index, osmo_mode);
    esp_err_t ret = osmo_send(camera_index, DUML_ADDR_CAMERA,
                              OSMO_CMDSET_CAMERA, OSMO_CMDID_SET_MODE,
                              OSMO_FLAGS_REQUEST, &osmo_mode, 1);
    if (ret == ESP_OK) {
        extern camera_state_t g_camera_states[NUM_CAMERAS];
        g_camera_states[camera_index].camera_mode = camera_mode_from_osmo(osmo_mode);
        g_ui_state.display_needs_update = true;
    }
    return ret;
}

camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(int camera_index, camera_mode_t mode) {
    if (set_osmo_mode(camera_index, osmo_mode_from_camera_mode(mode)) != ESP_OK) {
        return NULL;
    }
    return (camera_mode_switch_response_frame_t *)alloc_ret_ok(sizeof(camera_mode_switch_response_frame_t));
}

/* Quick-switch cursor: each press advances through this small ring and sends
 * the DUML set-mode command (DUML has no "cycle to next mode" key-report).
 * VIDEO(1)/PHOTO(0) are deliberately absent — they are the record control on
 * this camera (set_osmo_mode refuses them), so cycling through them would
 * start/stop recordings.  The remaining two are themselves unverified. */
static const uint8_t QS_MODE_RING[] = {
    OSMO_MODE_TIMELAPSE, OSMO_MODE_SLOWMO,
};
static uint8_t s_qs_cursor[NUM_CAMERAS];

static esp_err_t quick_switch_mode(int camera_index) {
    s_qs_cursor[camera_index] = (uint8_t)((s_qs_cursor[camera_index] + 1) % (sizeof(QS_MODE_RING)));
    return set_osmo_mode(camera_index, QS_MODE_RING[s_qs_cursor[camera_index]]);
}

version_query_response_frame_t* command_logic_get_version(int camera_index) {
    /* DUML version query (0x00/0x00) over BLE is best-effort; the reply
     * format is NUL-separated ASCII, unlike the R-SDK struct. Send it as a
     * request and, if the camera answers, hand the raw bytes back. */
    ESP_LOGI(TAG, "Camera %d: querying version (0x00/0x00)", camera_index);
    uint16_t seq = generate_seq();
    CommandResult result = send_command(camera_index, DUML_ADDR_CAMERA,
                                        OSMO_CMDSET_SESSION, OSMO_CMDID_GET_VERSION,
                                        NULL, 0, seq, 2000);
    if (result.structure == NULL) {
        return NULL;
    }
    return (version_query_response_frame_t *)result.structure;
}

/* ==========================================================================
 * RECORD CONTROL — hardware-confirmed on an Osmo Nano (2026-07-28)
 *
 * `0x02/0x02` to the camera (0x01) with flags 0x40 and a 1-byte payload is the
 * record control.  MEDIA_PROTOCOL documents it as a plain "set mode", but on
 * this body the mode value drives recording directly:
 *
 *     [01] -> starts recording   reply 00, status bit7 sets ~860 ms later
 *     [00] -> stops recording    reply 00, bit7 clears ~2.4 s later
 *
 * It is NOT a toggle: re-sending [01] while recording replies df (wrong
 * parameter), so the two directions must be issued explicitly.  The UI already
 * dispatches on the decoded recording bit, so it never has to guess.
 *
 * The documented `0x02/0x20` / `0x02/0x21` record start/stop answer e0
 * (unsupported) from every receiver on this firmware — they do not exist here.
 * `0x02/0x01` [01] answers d9 both idle and recording, so it is not it either.
 * ========================================================================== */
#define OSMO_RECORD_MODE_START 0x01
#define OSMO_RECORD_MODE_STOP  0x00

static esp_err_t send_record_mode(int camera_index, uint8_t mode) {
    const uint8_t payload[1] = { mode };
    ESP_LOGI(TAG, "Camera %d: %s recording (0x02/0x02 [%02X])", camera_index,
             mode == OSMO_RECORD_MODE_START ? "start" : "stop", mode);
    return osmo_send(camera_index, DUML_ADDR_CAMERA, OSMO_CMDSET_CAMERA,
                     OSMO_CMDID_SET_MODE, OSMO_FLAGS_REQUEST, payload, sizeof(payload));
}

esp_err_t command_logic_start_record_async(int camera_index) {
    return send_record_mode(camera_index, OSMO_RECORD_MODE_START);
}

record_control_response_frame_t* command_logic_start_record(int camera_index) {
    if (send_record_mode(camera_index, OSMO_RECORD_MODE_START) != ESP_OK) {
        return NULL;
    }
    return (record_control_response_frame_t *)alloc_ret_ok(sizeof(record_control_response_frame_t));
}

esp_err_t command_logic_stop_record_async(int camera_index) {
    return send_record_mode(camera_index, OSMO_RECORD_MODE_STOP);
}

record_control_response_frame_t* command_logic_stop_record(int camera_index) {
    if (send_record_mode(camera_index, OSMO_RECORD_MODE_STOP) != ESP_OK) {
        return NULL;
    }
    return (record_control_response_frame_t *)alloc_ret_ok(sizeof(record_control_response_frame_t));
}

/*
 * SHUTTER — 0x02/0x01 with a 1-byte payload [01].
 *
 * The 01 is a generic "shoot" trigger, not a photo type: in a Mimo datalink
 * capture a single shot and a burst both used [01], and the camera applied
 * whichever photo mode it was already set to.  Fire-and-forget — the camera
 * finishes a burst or interval sequence on its own, so there is no stop.
 *
 * Two earlier failures here were both self-inflicted, and the reply byte said
 * so each time:
 *   - an EMPTY payload answers e3 (parameter bad/missing) — the [01] is required
 *   - [01] while the camera is in a VIDEO mode answers d9 (wrong state)
 * d9 was never "unsupported" (that is e0); the camera has to be in the photo
 * shooting mode first, via 0x02/0xE1 [05].
 */
esp_err_t command_logic_take_photo(int camera_index) {
    const uint8_t payload[1] = { 0x01 };
    ESP_LOGI(TAG, "Camera %d: shutter (0x02/0x01 [01])", camera_index);
    return osmo_send(camera_index, DUML_ADDR_CAMERA,
                     OSMO_CMDSET_CAMERA, OSMO_CMDID_TAKE_PHOTO,
                     OSMO_FLAGS_REQUEST, payload, sizeof(payload));
}

/*
 * The big button means "capture" — which command that is depends on the mode
 * the camera reports at status offset 57.  Photo shoots; every other mode
 * (video, timelapse, hyperlapse, supernight, slowmo) records.
 */
esp_err_t command_logic_shutter_async(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    if (g_camera_states[camera_index].shoot_mode == OSMO_MODE_PHOTO) {
        return command_logic_take_photo(camera_index);
    }
    return send_record_mode(camera_index, OSMO_RECORD_MODE_START);
}

/**
 * @brief Quick-switch mode key report
 *
 * The R-SDK exposed a 0x00/0x11 key-report; the DUML camera set has no direct
 * equivalent, so quick-switch cycles modes with the set-mode command. Kept for
 * API compatibility — it advances to Video as a safe default.
 */
key_report_response_frame_t* command_logic_key_report_qs(int camera_index) {
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        return NULL;
    }
    if (quick_switch_mode(camera_index) != ESP_OK) {
        return NULL;
    }
    return (key_report_response_frame_t *)alloc_ret_ok(sizeof(key_report_response_frame_t));
}

/* ==========================================================================
 * Power / session
 * ========================================================================== */

camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_sleep(int camera_index) {
    /* There is no verified BLE sleep command for the Nano; stop the session
     * keepalive path by simply not driving it. Best-effort: send a session
     * ping so the camera's own idle timeout can take it to sleep. */
    ESP_LOGI(TAG, "Camera %d: sleep requested (best-effort)", camera_index);
    esp_err_t ret = osmo_send(camera_index, DUML_ADDR_SESSION,
                              OSMO_CMDSET_SESSION, OSMO_CMDID_SESSION_PING,
                              OSMO_FLAGS_REQUEST, OSMO_SESSION_OPEN, sizeof(OSMO_SESSION_OPEN));
    if (ret != ESP_OK) {
        return NULL;
    }
    return (camera_power_mode_switch_response_frame_t *)alloc_ret_ok(sizeof(camera_power_mode_switch_response_frame_t));
}

camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_wake(int camera_index) {
    ESP_LOGI(TAG, "Camera %d: wake (0x53/0x10)", camera_index);
    esp_err_t ret = osmo_send(camera_index, DUML_ADDR_SYSTEM,
                              OSMO_CMDSET_SYSTEM, OSMO_CMDID_SYSTEM_WAKE,
                              OSMO_FLAGS_REQUEST, (const uint8_t[]){0, 0, 0, 0}, 4);
    if (ret != ESP_OK) {
        return NULL;
    }
    return (camera_power_mode_switch_response_frame_t *)alloc_ret_ok(sizeof(camera_power_mode_switch_response_frame_t));
}

/* ==========================================================================
 * GPS push — not part of the verified Nano DUML set; kept as a no-op stub so
 * the GPS transmission task and its callers still link.
 * ========================================================================== */
gps_data_push_response_frame_t* command_logic_push_gps_data(int camera_index, const gps_data_push_command_frame_t* frame) {
    (void)camera_index;
    (void)frame;
    return NULL;
}

/* ==========================================================================
 * Slot query helpers (unchanged — read g_camera_states)
 * ========================================================================== */

bool command_logic_slot_supports_highlight(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    uint32_t device_id = g_camera_states[slot_index].device_id;
    if (device_id == 0) {
        return false;
    }
    if (device_id == 0xFF33 || device_id == 0xFF44 || device_id == 0xFF55) {
        return true;
    }
    return false;
}

bool command_logic_slot_is_paired(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_paired;
}

bool command_logic_slot_is_connected(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_connected &&
           g_camera_states[slot_index].connection_state == CAM_STATE_CONNECTED;
}

bool command_logic_slot_is_awake(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].power_mode != 3;
}

bool command_logic_slot_is_recording(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return false;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    return g_camera_states[slot_index].is_recording;
}

/* ==========================================================================
 * Key reporting — the DUML camera set has no key-report; these map to the
 * closest camera-control action so the UI's highlight/snapshot buttons keep
 * working (highlight -> take photo, a harmless best-effort marker).
 * ========================================================================== */

/* ==========================================================================
 * DIAGNOSTIC: parameter sweep  (PARAM_DUMP_ON_QS)
 *
 * 0x02/0x02 turned out to be DJI's *work* mode (0 capture / 1 record /
 * 2 playback / 3 download — 4 is rejected with df), NOT the shooting mode, so
 * Video/Photo/Timelapse lives somewhere else.  The 0x02/0x8E parameter channel
 * demonstrably works over BLE (a GET of pid 0x0014 answers with a value), so
 * sweep it and find the shooting mode by ground-truth diffing:
 *
 *   1. press QS  -> dump #1
 *   2. change the mode ON THE CAMERA by hand
 *   3. press QS  -> dump #2
 *   4. diff: whichever pid changed is the shooting mode, then SET it
 *
 * This is the same method that pinned the recording bit, and it beats guessing
 * payloads because the camera itself tells us which field moved.
 * ========================================================================== */
/*
 * ★ SHOOTING MODE = 0x02/0xE1 with a 1-byte payload (osmo_mode_t).
 *
 * Recovered from the full Mimo WiFi capture (Osmosis/capture_full.pcap) by
 * CRC-scanning every frame and tallying *all* app->camera commands rather than
 * grepping for expected opcodes.  It appears in no DJI doc or reference repo.
 * An empty payload answers e3 (bad parameter), which is why probing the opcode
 * alone had looked like a dead end.  Every value was then confirmed by picking
 * that mode by hand and reading it back from the status push.
 */
static const uint8_t MODE_CAROUSEL[] = OSMO_MODE_CAROUSEL;

esp_err_t command_logic_set_shoot_mode(int camera_index, uint8_t mode) {
    ESP_LOGI(TAG, "Camera %d: set shooting mode %s (0x%02X)",
             camera_index, osmo_mode_name(mode), mode);
    return osmo_send(camera_index, DUML_ADDR_CAMERA, OSMO_CMDSET_CAMERA,
                     OSMO_CMDID_SET_SHOOT_MODE, OSMO_FLAGS_REQUEST, &mode, 1);
}

/*
 * Advance from whatever the camera *reports* rather than a local cursor, so the
 * remote stays in step when the mode is changed on the camera itself.
 */
static esp_err_t cycle_shoot_mode(int camera_index) {
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    const uint8_t n = (uint8_t)(sizeof(MODE_CAROUSEL) / sizeof(MODE_CAROUSEL[0]));
    const uint8_t cur = g_camera_states[camera_index].shoot_mode;

    uint8_t at = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (MODE_CAROUSEL[i] == cur) {
            at = (uint8_t)(i + 1);
            break;
        }
    }
    return command_logic_set_shoot_mode(camera_index, MODE_CAROUSEL[at % n]);
}

/* ==========================================================================
 * Camera parameters — 0x02/0x8E
 *
 *   GET = 00 01 <pid:u16-LE>                   -> 00 00 01 <pid> <len> <value…>
 *   SET = 01 01 <pid:u16-LE> <len:u8> <value…> -> 00
 *
 * Known pids, each pinned by an A-B-A sweep diff (change it by hand, sweep,
 * change back, sweep) and then confirmed by writing it:
 *   0x0009  field of view   0x05 Natural-Wide · 0x01 Wide
 *   0x000F  ISO limit       0x04 100-800      · 0x05 100-1600
 * Values are stored *per shooting mode*, so changing the mode swaps in that
 * mode's saved settings — expect several pids to move at once, and never treat
 * a mere correlation as the control.
 * ========================================================================== */

esp_err_t command_logic_set_param(int camera_index, uint16_t pid,
                                  const uint8_t *value, uint8_t len) {
    uint8_t set[5 + 16];
    if (value == NULL || len == 0 || len > sizeof(set) - 5) {
        return ESP_ERR_INVALID_ARG;
    }
    set[0] = 0x01;
    set[1] = 0x01;
    set[2] = (uint8_t)(pid & 0xFF);
    set[3] = (uint8_t)(pid >> 8);
    set[4] = len;
    memcpy(&set[5], value, len);
    ESP_LOGI(TAG, "Camera %d: set param 0x%04X (%u B)", camera_index, pid, len);
    return osmo_send(camera_index, DUML_ADDR_CAMERA, OSMO_CMDSET_CAMERA,
                     OSMO_CMDID_PARAM, OSMO_FLAGS_REQUEST, set, (size_t)(5 + len));
}

esp_err_t command_logic_set_fov(int camera_index, osmo_fov_t fov) {
    const uint8_t v = (uint8_t)fov;
    return command_logic_set_param(camera_index, OSMO_PID_FOV, &v, 1);
}

esp_err_t command_logic_set_iso_limit(int camera_index, osmo_iso_limit_t iso) {
    const uint8_t v = (uint8_t)iso;
    return command_logic_set_param(camera_index, OSMO_PID_ISO_LIMIT, &v, 1);
}

/*
 * NOTE: the diagnostic sweeps that mapped this protocol have been removed from
 * the firmware — they live in tools/re/ (which documents the method) and can be
 * re-added temporarily when hunting a new setting.  Two of them must NOT come
 * back as-is:
 *   - enumerating every 0x02/0xE1 value FROZE a Nano solid (power-cycle
 *     required); unknown values are not harmlessly rejected.
 *   - any probe wired into the connect path fires on every boot and flash, so
 *     it mutates camera state before you have even started testing.
 */

esp_err_t command_logic_send_key_report_for_slot(int camera_index, uint8_t key_code, uint8_t mode, uint8_t key_value) {
    (void)mode;
    (void)key_value;
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    camera_state_t *cam_state = &g_camera_states[camera_index];
    if (!cam_state->is_paired || !cam_state->is_connected ||
        cam_state->connection_state != CAM_STATE_CONNECTED ||
        !ble_is_camera_connected(camera_index)) {
        ESP_LOGD(TAG, "Camera %d: not connected, cannot send key report", camera_index);
        return ESP_ERR_INVALID_STATE;
    }

    /* key_code 0x02 = QS (cycle mode), 0x03 = snapshot -> take photo */
    if (key_code == 0x03) {
        ESP_LOGI(TAG, "Camera %d: snapshot key -> take photo", camera_index);
        return command_logic_take_photo(camera_index);
    }
    return cycle_shoot_mode(camera_index);
}

esp_err_t command_logic_send_snapshot_key_for_slot(int camera_index) {
    return command_logic_send_key_report_for_slot(camera_index, 0x03, 0x01, 0x00);
}

esp_err_t command_logic_send_highlight_for_slot(int slot_index) {
    if (slot_index < 0 || slot_index >= NUM_CAMERAS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!command_logic_slot_supports_highlight(slot_index)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return command_logic_send_key_report_for_slot(slot_index, 0x02, 0x01, 0x00);
}

esp_err_t command_logic_send_highlight_for_all_active(void) {
    int success_count = 0;
    for (int i = 0; i < NUM_CAMERAS; i++) {
        if (command_logic_slot_is_paired(i) &&
            command_logic_slot_is_connected(i) &&
            command_logic_slot_is_awake(i) &&
            command_logic_slot_is_recording(i) &&
            command_logic_slot_supports_highlight(i)) {
            if (command_logic_send_highlight_for_slot(i) == ESP_OK) {
                success_count++;
            }
        }
    }
    return success_count > 0 ? ESP_OK : ESP_FAIL;
}
