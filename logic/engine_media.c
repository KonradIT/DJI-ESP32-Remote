/*
 * Media (DUML) engine — Osmo Nano, Pocket 3.
 *
 * A thin adapter: every frame below is already hardware-confirmed on a Nano and
 * implemented in command_logic.c. This file exists so the UI can call
 * record_start()/shoot_photo() without knowing which protocol answers.
 */

#include "camera_engine.h"
#include "command_logic.h"
#include "osmo_duml.h"
#include "ui.h"          /* g_camera_states — for the mode read-back */
#include "esp_log.h"

#define TAG "ENG_MEDIA"

static esp_err_t media_session_open(int slot)
{
    /* The DUML session (0x00/0x2B -> 0xF0), pairing, wake and the config
     * subscriptions are driven by connect_logic during the connect sequence,
     * so there is nothing extra to do once the link is up. */
    (void)slot;
    return ESP_OK;
}

/*
 * 0x02/0x02 [01] starts, [00] stops — NOT a toggle: re-sending [01] while
 * recording answers df. Note the inversion versus R-SDK (0 = start there),
 * which is exactly why the interface has two calls instead of record(bool).
 */
static esp_err_t media_record_start(int slot) { return media_record_start_raw(slot); }
static esp_err_t media_record_stop(int slot)  { return media_record_stop_raw(slot); }

/* 0x02/0x01 [01]. The [01] is required (empty -> e3) and the camera must
 * already be in photo mode (-> d9 otherwise). */
static esp_err_t media_shoot_photo(int slot)  { return command_logic_take_photo(slot); }

static esp_err_t media_set_mode(int slot, cam_mode_t mode)
{
    uint8_t osmo;
    switch (mode) {
        case CAM_MODE_VIDEO:      osmo = OSMO_MODE_VIDEO;      break;
        case CAM_MODE_PHOTO:      osmo = OSMO_MODE_PHOTO;      break;
        case CAM_MODE_TIMELAPSE:  osmo = OSMO_MODE_TIMELAPSE;  break;
        case CAM_MODE_SLOWMO:     osmo = OSMO_MODE_SLOWMO;     break;
        case CAM_MODE_HYPERLAPSE: osmo = OSMO_MODE_HYPERLAPSE; break;
        case CAM_MODE_SUPERNIGHT: osmo = OSMO_MODE_SUPERNIGHT; break;
        default:
            /* ⚠ Never send an unlisted value on 0x02/0xE1 — sweeping this
             * opcode's value space froze a Nano solid and needed a power
             * cycle. Refuse rather than pass something through. */
            ESP_LOGW(TAG, "Camera %d: refusing unmapped mode %d", slot, (int)mode);
            return ESP_ERR_INVALID_ARG;
    }
    return command_logic_set_shoot_mode(slot, osmo);
}

cam_mode_t cam_mode_from_dji_wire(uint8_t wire)
{
    switch (wire) {
        case OSMO_MODE_VIDEO:      return CAM_MODE_VIDEO;
        case OSMO_MODE_PHOTO:      return CAM_MODE_PHOTO;
        case OSMO_MODE_TIMELAPSE:  return CAM_MODE_TIMELAPSE;
        case OSMO_MODE_SLOWMO:     return CAM_MODE_SLOWMO;
        case OSMO_MODE_HYPERLAPSE: return CAM_MODE_HYPERLAPSE;
        case OSMO_MODE_SUPERNIGHT: return CAM_MODE_SUPERNIGHT;
        default:                   return CAM_MODE_UNKNOWN;
    }
}

cam_photo_size_t media_photo_size_from_wire(uint8_t wire)
{
    switch (wire) {
        case OSMO_PHOTO_SIZE_M: return CAM_PHOTO_SIZE_M;
        case OSMO_PHOTO_SIZE_L: return CAM_PHOTO_SIZE_L;
        default:                return CAM_PHOTO_SIZE_UNKNOWN;
    }
}

cam_photo_aspect_t media_photo_aspect_from_wire(uint8_t wire)
{
    switch (wire) {
        case OSMO_PHOTO_ASPECT_4_3:  return CAM_PHOTO_ASPECT_4_3;
        case OSMO_PHOTO_ASPECT_16_9: return CAM_PHOTO_ASPECT_16_9;
        default:                     return CAM_PHOTO_ASPECT_UNKNOWN;
    }
}

/*
 * The camera's on-screen carousel order, in neutral terms. NOT numeric order —
 * the wire enum is sparse and unordered (0,1,2,5,0x0A,0x28), so the next mode
 * can only ever come from a table.
 */
static const cam_mode_t MEDIA_CAROUSEL[] = {
    CAM_MODE_VIDEO, CAM_MODE_PHOTO, CAM_MODE_TIMELAPSE,
    CAM_MODE_HYPERLAPSE, CAM_MODE_SUPERNIGHT, CAM_MODE_SLOWMO,
};

/*
 * Cycle by table lookup on the mode the camera last reported.
 *
 * ⚠ Do NOT reach for 0x02/0x02 here. Its documented meaning is DJI's four-value
 * *work* mode, but on a Nano that opcode IS record control — a "Video" entry
 * mapped to [01] would start a recording behind the user's back. Shooting mode
 * is 0x02/0xE1 and only 0x02/0xE1.
 *
 * The current mode comes from the camera's own 0x02/0x80 push (byte @57), so it
 * stays correct even when the user changes mode on the camera itself.
 */
static esp_err_t media_mode_cycle(int slot)
{
    const int n = (int)(sizeof(MEDIA_CAROUSEL) / sizeof(MEDIA_CAROUSEL[0]));
    cam_mode_t cur = g_camera_states[slot].shoot_mode;

    int idx = -1;
    for (int i = 0; i < n; i++) {
        if (MEDIA_CAROUSEL[i] == cur) { idx = i; break; }
    }
    if (idx < 0) {
        /* No status push seen yet, or the camera is in a mode we have never
         * observed. Advancing from an unknown position would jump the user
         * somewhere arbitrary, so do nothing and say why. */
        ESP_LOGW(TAG, "Camera %d: current mode %s not in carousel, not cycling",
                 slot, cam_mode_name(cur));
        return ESP_ERR_INVALID_STATE;
    }

    cam_mode_t next = MEDIA_CAROUSEL[(idx + 1) % n];
    ESP_LOGI(TAG, "Camera %d: mode %s -> %s",
             slot, cam_mode_name(cur), cam_mode_name(next));
    return media_set_mode(slot, next);
}

const camera_engine_t g_engine_media = {
    .name         = "media",
    .session_open = media_session_open,
    .record_start = media_record_start,
    .record_stop  = media_record_stop,
    .shoot_photo  = media_shoot_photo,
    .set_mode     = media_set_mode,
    .mode_cycle   = media_mode_cycle,
    /* No GPS push and no highlight opcode is known for this family. Photo size
     * and aspect ARE reported (cam_photo_param_new). EIS is not: three A-B-A
     * runs found no parameter that tracks RockSteady — see TODO.md #2. */
    .caps         = CAM_CAP_PHOTO_SIZE,
};
