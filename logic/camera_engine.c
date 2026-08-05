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
    /* Route 1 — the model id off the air. Hardware-confirmed for the Nano:
     * mfg[2..3] little-endian under DJI company id 0x08AA reads 0x0019. */
    if (adv_model_id == CAM_ADV_MODEL_OSMO_NANO) {
        return &g_engine_media;
    }

    /*
     * Route 2 — the one name-based exception. A Pocket 3 reportedly sends no
     * manufacturer data at all, so no model id can ever appear for it and a
     * name match is the only pre-connect signal available.
     *
     * ⚠ UNVERIFIED: we have never captured a Pocket 3 advertisement. If one
     * turns out to carry mfg data, delete this branch and add its model id to
     * route 1 instead — a name match is weaker and should not outlive its
     * necessity.
     */
    if (name_has_ci(adv_name, "pocket")) {
        ESP_LOGW(TAG, "'%s' matched the name-based Pocket exception -> media",
                 adv_name ? adv_name : "?");
        return &g_engine_media;
    }

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
