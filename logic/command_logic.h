#ifndef __COMMAND_LOGIC_H__
#define __COMMAND_LOGIC_H__

#include "enums_logic.h"
#include "osmo_duml.h"   /* osmo_mode_t / osmo_fov_t / osmo_iso_limit_t */

#include "dji_protocol_data_structures.h"

// External global device ID from ui.c
extern uint32_t g_device_id;

uint16_t generate_seq(void);

typedef struct {
    void *structure;   // malloc'd copy of the camera's response payload (caller frees)
    size_t length;     // response payload length in bytes
} CommandResult;

/*
 * Fire-and-forget DUML write to a camera slot. Builds the frame (auto seq),
 * paces it onto 0xFFF5, and returns without waiting for a response.
 *   dst      — DUML receiver byte (DUML_ADDR_CAMERA, _SESSION, _WIFI, _SYSTEM)
 *   flags    — OSMO_FLAGS_NOTIFY (0x00) or OSMO_FLAGS_REQUEST (0x40)
 *   payload  — may be NULL when payload_len == 0
 */
esp_err_t osmo_send(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                    uint8_t flags, const uint8_t *payload, size_t payload_len);

/*
 * Like osmo_send() with flags 0x40, but uses a caller-supplied seq and registers
 * it so the reply can be collected later with data_wait_for_result_by_seq().
 * Lets the caller send now and wait on its own (short) schedule.
 */
esp_err_t duml_write_tracked(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                             const uint8_t *payload, size_t payload_len, uint16_t seq);

/*
 * DUML request (flags 0x40) that waits up to timeout_ms for the camera's
 * response frame (flags 0xC0), matched by seq. Returns {malloc'd payload,
 * len} on success or {NULL, 0} on timeout/failure. Caller frees .structure.
 */
CommandResult send_command(int camera_index, uint8_t dst, uint8_t cmd_set, uint8_t cmd_id,
                           const uint8_t *payload, size_t payload_len, uint16_t seq, int timeout_ms);

camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(int camera_index, camera_mode_t mode);

version_query_response_frame_t* command_logic_get_version(int camera_index);

record_control_response_frame_t* command_logic_start_record(int camera_index);
esp_err_t command_logic_start_record_async(int camera_index);

record_control_response_frame_t* command_logic_stop_record(int camera_index);
esp_err_t command_logic_stop_record_async(int camera_index);

esp_err_t command_logic_take_photo(int camera_index);

/* Set the shooting mode (osmo_mode_t) via 0x02/0xE1. The camera echoes it back
 * in its status push, so g_camera_states[].shoot_mode follows automatically. */
esp_err_t command_logic_set_shoot_mode(int camera_index, uint8_t mode);

/* Camera settings over the 0x02/0x8E parameter channel. */
esp_err_t command_logic_set_param(int camera_index, uint16_t pid,
                                  const uint8_t *value, uint8_t len);
esp_err_t command_logic_set_fov(int camera_index, osmo_fov_t fov);
esp_err_t command_logic_set_iso_limit(int camera_index, osmo_iso_limit_t iso);

/* Parameter sweeps and other RE probes live in tools/re/, not in the firmware. */


key_report_response_frame_t* command_logic_key_report_qs(int camera_index);

camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_sleep(int camera_index);
camera_power_mode_switch_response_frame_t* command_logic_power_mode_switch_wake(int camera_index);

gps_data_push_response_frame_t* command_logic_push_gps_data(int camera_index, const gps_data_push_command_frame_t* frame);

// Key Reporting functions
esp_err_t command_logic_send_key_report_for_slot(int camera_index, uint8_t key_code, uint8_t mode, uint8_t key_value);
esp_err_t command_logic_send_snapshot_key_for_slot(int camera_index);

// Highlight tag functions
bool command_logic_slot_supports_highlight(int slot_index);
bool command_logic_slot_is_paired(int slot_index);
bool command_logic_slot_is_connected(int slot_index);
bool command_logic_slot_is_awake(int slot_index);
bool command_logic_slot_is_recording(int slot_index);
esp_err_t command_logic_send_highlight_for_slot(int slot_index);
esp_err_t command_logic_send_highlight_for_all_active(void);

#endif
