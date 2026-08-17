/*
 * Engine assignment — positive identification only.
 *
 * There is deliberately no default engine. Picking one when identification
 * failed sends R-SDK frames to a DUML body (or the reverse), which does not
 * fail loudly: the camera simply ignores everything while still streaming
 * status, so the remote looks connected and does nothing. Refusing is the
 * behaviour that surfaces the problem.
 */

#include "camera_engine.h"
#include "ui.h"
#include "esp_log.h"
#include <string.h>
#include <ctype.h>

#define TAG "CAM_ENGINE"

/* Case-insensitive substring test; `needle` must be lowercase. */
static bool name_has_ci(const char *hay, const char *needle)
{
    if (hay == NULL || needle == NULL) return false;
    size_t nl = strlen(needle);
    if (nl == 0) return false;
    for (const char *p = hay; *p; p++) {
        size_t i = 0;
        while (i < nl && p[i] && (char)tolower((unsigned char)p[i]) == needle[i]) i++;
        if (i == nl) return true;
    }
    return false;
}

const camera_engine_t *camera_engine_from_advert(uint32_t adv_model_id,
                                                 const char *adv_name)
{
    /*
     * The model id off the air — every id below is hardware-confirmed from our
     * own scan logs, not inferred from a family or a naming pattern.
     */
    switch (adv_model_id) {
        case CAM_ADV_MODEL_OSMO_NANO:      /* 0x0019, confirmed */
        case CAM_ADV_MODEL_OSMO_POCKET3:   /* 0x0020, confirmed */
            return &g_engine_media;

        case CAM_ADV_MODEL_OSMO_ACTION6:   /* 0x0018, confirmed */
            return &g_engine_rsdk;

        default:
            break;
    }

    /*
     * The name-based Pocket exception that used to live here is GONE. It
     * existed on the belief that a Pocket 3 sends no manufacturer data, so a
     * name match was the only pre-connect signal. A Pocket 3 capture disproved
     * that outright — it advertises 0x0020, before we connect — so the id
     * above replaces it. Matching on a name is guessing dressed up as
     * identification; do not reintroduce it.
     */
    (void)name_has_ci;
    (void)adv_name;

    /* Not identifiable from the advertisement. The caller should ask via the
     * R-SDK connection request rather than assume. */
    return NULL;
}

const camera_engine_t *camera_engine_from_rsdk_device_id(uint32_t device_id)
{
    switch (device_id) {
        case RSDK_DEVICE_ID_ACTION4:
        case RSDK_DEVICE_ID_ACTION5:
        case RSDK_DEVICE_ID_ACTION6:
        case RSDK_DEVICE_ID_OSMO360:
            return &g_engine_rsdk;
        default:
            /* A body that answered the connection request with an id we do not
             * know. Do NOT assume R-SDK: report it so the id can be added
             * deliberately, with a model behind it. */
            if (device_id != 0) {
                ESP_LOGW(TAG, "Unrecognised R-SDK device_id 0x%04X — refusing",
                         (unsigned)device_id);
            }
            return NULL;
    }
}

const char *cam_mode_name(cam_mode_t mode)
{
    switch (mode) {
        case CAM_MODE_VIDEO:      return "Video";
        case CAM_MODE_PHOTO:      return "Photo";
        case CAM_MODE_TIMELAPSE:  return "TimeLapse";
        case CAM_MODE_SLOWMO:     return "SlowMo";
        case CAM_MODE_HYPERLAPSE: return "HyperLapse";
        case CAM_MODE_SUPERNIGHT: return "SuperNight";
        default:                  return "Unknown";
    }
}

const char *cam_photo_size_name(cam_photo_size_t size)
{
    switch (size) {
        case CAM_PHOTO_SIZE_M: return "M";
        case CAM_PHOTO_SIZE_L: return "L";
        default:               return NULL;   /* omit, never guess */
    }
}

const char *cam_photo_aspect_name(cam_photo_aspect_t aspect)
{
    switch (aspect) {
        case CAM_PHOTO_ASPECT_4_3:  return "4:3";
        case CAM_PHOTO_ASPECT_16_9: return "16:9";
        default:                    return NULL;
    }
}

const camera_engine_t *camera_engine_for_slot(int slot)
{
    if (slot < 0 || slot >= NUM_CAMERAS) return NULL;
    return g_camera_states[slot].engine;
}

bool camera_engine_slot_has_cap(int slot, uint32_t cap)
{
    const camera_engine_t *e = camera_engine_for_slot(slot);
    return e != NULL && (e->caps & cap) != 0;
}
