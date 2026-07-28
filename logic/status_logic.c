/*
 * DJI Camera Remote Control - Status Logic Layer
 * 
 * This file handles camera status updates received from DJI cameras via BLE.
 * It processes status push messages (1D02 and 1D06) and updates camera state
 * structures for the UI system.
 * 
 * Key features:
 * - Status subscription management
 * - Camera state update callbacks
 * - Recording state detection
 * - Sleep mode tracking
 * - Multi-camera status synchronization
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "data.h"
#include "duml.h"
#include "osmo_duml.h"
#include "../main/ui.h"

static const char *TAG = "LOGIC_STATUS";

// Global variables to store various camera status information
uint8_t current_camera_mode = 0;
uint8_t current_camera_status = 0;
uint8_t current_video_resolution = 0;
uint8_t current_fps_idx = 0;
uint8_t current_eis_mode = 0;
uint8_t current_user_mode = 0;
uint8_t current_camera_mode_next_flag = 0;
uint16_t current_record_time = 0;
uint16_t current_timelapse_interval = 0;
uint32_t current_remain_capacity = 0;
uint32_t current_remain_time = 0;
uint8_t current_camera_bat_percentage = 0;
uint8_t current_power_mode = 0;  // Sleep mode tracking: 0=normal, 3=sleep
bool camera_status_initialized = false;
uint32_t g_last_status_push_timestamp = 0;  // Timestamp of last camera status push (milliseconds)

// Global variables for new camera status push command frame
uint8_t current_type_mode_name = 0;
uint8_t current_mode_name_length = 0;
uint8_t current_mode_name[20] = {0};
uint8_t current_type_mode_param = 0;
uint8_t current_mode_param_length = 0;
uint8_t current_mode_param[20] = {0};


/**
 * @brief Check if camera is recording
 * 
 * Check if camera is in recording or pre-recording state, and status is initialized.
 * 
 * @return bool Returns true if camera is recording, false otherwise
 */
bool is_camera_recording() {
    if ((current_camera_status == CAMERA_STATUS_PHOTO_OR_RECORDING || current_camera_status == CAMERA_STATUS_PRE_RECORDING) && camera_status_initialized) {
        return true;
    }
    return false;
}

/**
 * @brief Print current camera status (partial status, other status can be printed as needed)
 * 
 * Print camera mode, status, resolution, frame rate and electronic image stabilization mode.
 */
void print_camera_status() {
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status has not been initialized.");
        return;
    }

    const char *mode_str = camera_mode_to_string((camera_mode_t)current_camera_mode);
    const char *status_str = camera_status_to_string((camera_status_t)current_camera_status);
    const char *resolution_str = video_resolution_to_string((video_resolution_t)current_video_resolution);
    const char *fps_str = fps_idx_to_string((fps_idx_t)current_fps_idx);
    const char *eis_str = eis_mode_to_string((eis_mode_t)current_eis_mode);

    ESP_LOGI(TAG, "[1D02] =========== Camera Status Push ===========");
    ESP_LOGI(TAG, "  Mode: %s", mode_str);
    ESP_LOGI(TAG, "  Status: %s", status_str);
    ESP_LOGI(TAG, "  Resolution: %s (value: %d)", resolution_str, current_video_resolution);
    ESP_LOGI(TAG, "  FPS: %s", fps_str);
    ESP_LOGI(TAG, "  EIS: %s", eis_str);
    ESP_LOGI(TAG, "  User mode: %d", current_user_mode);
    ESP_LOGI(TAG, "  Camera mode next flag: %d", current_camera_mode_next_flag);
    ESP_LOGI(TAG, "  Record time: %d", current_record_time);
    ESP_LOGI(TAG, "  Timelapse interval: %d", current_timelapse_interval);
    ESP_LOGI(TAG, "=================================================");
}

/*
 * There is no status subscription on this transport: the Osmo Nano starts
 * pushing 0x02/0x80 camera status (~10 Hz), 0x02/0xDC storage and 0x0D/0x02
 * battery the moment notifications are enabled on 0xFFF4, before we send
 * anything (hardware-confirmed). The old R-SDK 0x1D/0x05 subscribe does not
 * apply, so no call is needed after connect. Named-config values are a
 * separate channel — see osmo_build_cfg_subscribe() in connect_logic.c.
 */

/* Little-endian reads that stay inside the payload */
static uint32_t rd_u32_le(const uint8_t *p, size_t off, uint16_t len) {
    if (off + 4 > len) return 0;
    return (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
           ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
}
static uint16_t rd_u16_le(const uint8_t *p, size_t off, uint16_t len) {
    if (off + 2 > len) return 0;
    return (uint16_t)p[off] | ((uint16_t)p[off + 1] << 8);
}

/*
 * Ingest one Osmo Nano status push (delivered as an osmo_push_blob_t by data.c)
 * and update the shared camera_state_t the UI renders.  We map only the fields
 * the UI consumes: recording flag, remaining SD capacity, elapsed record time,
 * and battery percent.  Frame layouts are from MEDIA_PROTOCOL.md.
 */
void update_camera_state_handler(int camera_index, void *data) {
    if (!data) {
        ESP_LOGE(TAG, "Camera %d: status update received NULL data", camera_index);
        return;
    }
    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera_index %d in status update", camera_index);
        free(data);
        return;
    }

    const osmo_push_blob_t *blob = (const osmo_push_blob_t *)data;
    const uint8_t *p = blob->payload;
    uint16_t len = blob->payload_len;
    camera_state_t *cam = &g_camera_states[camera_index];

    g_last_status_push_timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    cam->last_status_timestamp = g_last_status_push_timestamp;
    bool changed = false;

    if (blob->cmd_set == OSMO_CMDSET_CAMERA && blob->cmd_id == OSMO_CMDID_STATUS_PUSH) {
        /* 0x02/0x80 camera status — offsets ground-truthed on hardware, see
         * osmo_duml.h: recording = byte0 bit7, free MiB @9, remaining
         * recordable seconds @17, elapsed record seconds @29. */
        if (len > OSMO_STATUS_FLAGS_BYTE) {
            bool rec = (p[OSMO_STATUS_FLAGS_BYTE] & OSMO_STATUS_RECORDING_MASK) != 0;
            if (cam->is_recording != rec) {
                cam->is_recording = rec;
                cam->camera_status = rec ? CAMERA_STATUS_PHOTO_OR_RECORDING
                                         : CAMERA_STATUS_LIVE_STREAMING;
                changed = true;
                ESP_LOGI(TAG, "Camera %d: recording -> %s", camera_index, rec ? "ON" : "OFF");
                if (camera_index == 0) current_camera_status = cam->camera_status;
            }
        }
        uint32_t free_mib = rd_u32_le(p, OSMO_STATUS_STORE_FREE_MIB, len);
        if (free_mib != 0 && cam->remain_capacity != free_mib) {
            cam->remain_capacity = free_mib;
            changed = true;
            if (camera_index == 0) current_remain_capacity = free_mib;
        }
        uint16_t rec_s = rd_u16_le(p, OSMO_STATUS_RECORD_TIME_S, len);
        if (cam->record_time != rec_s) {
            cam->record_time = rec_s;
            changed = true;
            if (camera_index == 0) current_record_time = rec_s;
        }
        uint16_t remain_s = rd_u16_le(p, OSMO_STATUS_REMAIN_TIME_S, len);
        if (remain_s != 0 && cam->remain_time != remain_s) {
            cam->remain_time = remain_s;
            changed = true;
            if (camera_index == 0) current_remain_time = remain_s;
        }
        /* @57 echoes the last 0x02/0xE1 written, so it tracks the mode whether
         * we set it or the user did it on the camera. */
        if (len > OSMO_STATUS_MODE && cam->shoot_mode != p[OSMO_STATUS_MODE]) {
            cam->shoot_mode = p[OSMO_STATUS_MODE];
            changed = true;
            ESP_LOGI(TAG, "Camera %d: mode -> %s (0x%02X)", camera_index,
                     osmo_mode_name(cam->shoot_mode), cam->shoot_mode);
        }
    } else if (blob->cmd_set == OSMO_CMDSET_SESSION && blob->cmd_id == OSMO_CMDID_CFG_ITEM) {
        /*
         * Named config push (0x00/0x99 from 0x28), self-describing:
         *   02 06 00 00 | idx:u32-LE | 00 00 00 | total_len:u16-LE
         *   | name_len:u16-LE | name | 00 x6 | value_len:u16-LE | value
         * Arrives only after a per-parameter subscribe (verb 0x02) — see
         * osmo_build_cfg_subscribe().
         */
        const char *name = NULL;
        const uint8_t *val = NULL;
        uint16_t name_len = 0, val_len = 0;
        if (len >= 15) {
            name_len = (uint16_t)(p[13] | (p[14] << 8));
            size_t vlen_off = (size_t)15 + name_len + 6;
            if (name_len > 0 && vlen_off + 2 <= len) {
                name = (const char *)&p[15];
                val_len = (uint16_t)(p[vlen_off] | (p[vlen_off + 1] << 8));
                if (vlen_off + 2 + val_len <= len) {
                    val = &p[vlen_off + 2];
                }
            }
        }
        if (name && val) {
            /* Log every named push once so unmapped parameters can be decoded
             * from the log without another firmware round-trip. */
#if DEBUG_DUML_PACKETS
            /* Names + values of every config push — this is how an unmapped
             * setting (EIS, colour mode, …) gets decoded from a log. */
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, val, val_len > 24 ? 24 : val_len, ESP_LOG_INFO);
            ESP_LOGI(TAG, "cfg cam%d '%.*s' (%u B)", camera_index, (int)name_len, name, val_len);
#endif
            /* cam_video_param_v2: [resolution:u8][fps_idx:u8]… — the current
             * video setting. (camcap_video_format is the *capability* list of
             * supported pairs, not the active one.) */
            if (name_len == 18 && strncmp(name, "cam_video_param_v2", 18) == 0 && val_len >= 2) {
                if (cam->video_resolution != val[0] || cam->fps_idx != val[1]) {
                    cam->video_resolution = val[0];
                    cam->fps_idx = val[1];
                    changed = true;
                    ESP_LOGI(TAG, "Camera %d: video %u @ fps_idx %u",
                             camera_index, val[0], val[1]);
                    if (camera_index == 0) {
                        current_video_resolution = val[0];
                        current_fps_idx = val[1];
                    }
                }
            }
        }
    } else if (blob->cmd_set == OSMO_CMDSET_CAMERA && blob->cmd_id == OSMO_CMDID_STATE_QUERY) {
        /* 0x02/0xA0 state query response: record_time u16 @6 */
        uint16_t rt = rd_u16_le(p, OSMO_STATE_RECORD_TIME_S, len);
        if (cam->record_time != rt) {
            cam->record_time = rt;
            changed = true;
            if (camera_index == 0) current_record_time = rt;
        }
    } else if (blob->cmd_set == OSMO_CMDSET_CAMERA && blob->cmd_id == OSMO_CMDID_STORAGE_PUSH) {
        /* 0x02/0xDC storage: SD free @10 if a card is present, else internal @28 */
        uint32_t sd_total = rd_u32_le(p, OSMO_STORAGE_SD_TOTAL, len);
        uint32_t free_mib = (sd_total > 0) ? rd_u32_le(p, OSMO_STORAGE_SD_FREE, len)
                                           : rd_u32_le(p, OSMO_STORAGE_INT_FREE, len);
        if (free_mib != 0 && cam->remain_capacity != free_mib) {
            cam->remain_capacity = free_mib;
            changed = true;
            if (camera_index == 0) current_remain_capacity = free_mib;
        }
    } else if (blob->cmd_set == OSMO_CMDSET_BATTERY && blob->cmd_id == OSMO_CMDID_BATTERY_PUSH) {
        /* 0x0D/0x02 battery: percent u8 @20 */
        if (len > OSMO_BATT_PERCENT) {
            uint8_t pct = p[OSMO_BATT_PERCENT];
            if (pct <= 100 && cam->camera_bat_percentage != pct) {
                cam->camera_bat_percentage = pct;
                cam->battery_percentage = pct;
                changed = true;
                ESP_LOGI(TAG, "Camera %d: battery -> %d%%", camera_index, pct);
                if (camera_index == 0) current_camera_bat_percentage = pct;
            }
        }
    } else {
        ESP_LOGD(TAG, "Camera %d: unhandled status 0x%02X/0x%02X (%u B)",
                 camera_index, blob->cmd_set, blob->cmd_id, len);
    }

    if (!cam->is_initialized) {
        cam->is_initialized = true;
        changed = true;
        if (camera_index == 0) camera_status_initialized = true;
        ESP_LOGI(TAG, "Camera %d status initialized", camera_index);
    }

    if (changed) {
        g_ui_state.display_needs_update = true;
    }

    free(data);
}

void update_new_camera_state_handler(int camera_index, void *data) {
    if (!data) {
        ESP_LOGE(TAG, "Camera %d: update_new_camera_state_handler: Received NULL data.", camera_index);
        return;
    }

    if (camera_index < 0 || camera_index >= NUM_CAMERAS) {
        ESP_LOGE(TAG, "Invalid camera_index %d in new status update", camera_index);
        free(data);
        return;
    }

    const new_camera_status_push_command_frame *parsed_data = (const new_camera_status_push_command_frame *)data;

    // Get reference to this camera's state
    extern camera_state_t g_camera_states[NUM_CAMERAS];
    camera_state_t *cam_state = &g_camera_states[camera_index];

    ESP_LOGI(TAG, "Camera %d [1D06] =============New Status Push============", camera_index);

    // Store mode_name (ensure null termination)
    memset(cam_state->mode_name, 0, sizeof(cam_state->mode_name));
    size_t mode_name_len = (parsed_data->mode_name_length < 20) ? parsed_data->mode_name_length : 20;
    memcpy(cam_state->mode_name, parsed_data->mode_name, mode_name_len);
    cam_state->mode_name[20] = '\0';  // Ensure null termination
    ESP_LOGI(TAG, "[1D06] Mode name: %s (length: %d)", cam_state->mode_name, parsed_data->mode_name_length);
    
    // Store mode_param (ensure null termination)
    memset(cam_state->mode_param, 0, sizeof(cam_state->mode_param));
    size_t mode_param_len = (parsed_data->mode_param_length < 20) ? parsed_data->mode_param_length : 20;
    memcpy(cam_state->mode_param, parsed_data->mode_param, mode_param_len);
    cam_state->mode_param[20] = '\0';  // Ensure null termination
    ESP_LOGI(TAG, "[1D06] Mode parameters: '%s' (length: %d)", cam_state->mode_param, parsed_data->mode_param_length);

    // Mark this camera as supporting new status push (1D06)
    // Once set to true, this flag stays true for the entire pairing session.
    // Video Mode Area will now be updated ONLY from 1D06, not from 1D02.
    cam_state->camera_supports_new_status_push = true;
    ESP_LOGI(TAG, "[1D06] Camera %d now uses new status push (1D06)", camera_index);

    // Trigger UI update
    extern ui_state_t g_ui_state;
    g_ui_state.display_needs_update = true;

    ESP_LOGI(TAG, "[1D06] ================================");

    free(data);
}