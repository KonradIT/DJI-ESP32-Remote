/*
 * Osmo Nano command layer — constant payloads and builders.  See osmo_duml.h.
 */

#include "osmo_duml.h"
#include <string.h>

const char *osmo_mode_name(uint8_t mode)
{
    switch (mode) {
        case OSMO_MODE_SLOWMO:     return "SlowMo";
        case OSMO_MODE_VIDEO:      return "Video";
        case OSMO_MODE_TIMELAPSE:  return "TimeLapse";
        case OSMO_MODE_PHOTO:      return "Photo";
        case OSMO_MODE_HYPERLAPSE: return "HyperLapse";
        case OSMO_MODE_SUPERNIGHT: return "SuperNight";
        default:                   return "?";
    }
}

/*
 * NULL rather than "?" for an unobserved code: the caller then omits the field
 * instead of printing a placeholder next to a real value, which would read as
 * though we knew the setting and it was unset.
 */
const char *osmo_photo_size_name(uint8_t size)
{
    switch (size) {
        case OSMO_PHOTO_SIZE_M: return "M";
        case OSMO_PHOTO_SIZE_L: return "L";
        default:                return NULL;
    }
}

const char *osmo_photo_aspect_name(uint8_t aspect)
{
    switch (aspect) {
        case OSMO_PHOTO_ASPECT_4_3:  return "4:3";
        case OSMO_PHOTO_ASPECT_16_9: return "16:9";
        default:                     return NULL;
    }
}

const uint8_t OSMO_SESSION_OPEN[2]      = { 0x04, 0x00 };
const uint8_t OSMO_SESSION_KEEPALIVE[2] = { 0x01, 0x01 };
const uint8_t OSMO_PARAM_GET_0014[4] = { 0x00, 0x01, 0x14, 0x00 };

/*
 * Per-parameter config subscribe — the form DJI Mimo actually uses.
 *
 *   02 02 00 00 | sub_id:u32-LE | 00 00 00 | name_len+6:u16-LE | name_len:u16-LE
 *   | name ascii | 00 00 00 00
 *
 * Decoded from 53 such frames in a Mimo capture (tools/re/dump99.py). Our
 * earlier group-subscribe (verb 0x01, payload `01 00 06 00 "camera"`) was
 * malformed: the camera ACKed it with plen=0 and never sent a single item,
 * which looked exactly like an unsupported channel.
 */
size_t osmo_build_cfg_subscribe(uint8_t *out, size_t out_cap, uint32_t sub_id,
                                const char *name)
{
    size_t nlen = strlen(name);
    size_t need = 4 + 4 + 3 + 2 + 2 + nlen + 4;
    if (out == NULL || name == NULL || nlen == 0 || nlen > 0xFF || out_cap < need) {
        return 0;
    }
    uint8_t *p = out;
    *p++ = OSMO_CFG_VERB_SUBSCRIBE; *p++ = 0x02; *p++ = 0x00; *p++ = 0x00;
    *p++ = (uint8_t)(sub_id);        *p++ = (uint8_t)(sub_id >> 8);
    *p++ = (uint8_t)(sub_id >> 16);  *p++ = (uint8_t)(sub_id >> 24);
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
    *p++ = (uint8_t)((nlen + 6) & 0xFF); *p++ = (uint8_t)((nlen + 6) >> 8);
    *p++ = (uint8_t)(nlen & 0xFF);       *p++ = (uint8_t)(nlen >> 8);
    memcpy(p, name, nlen); p += nlen;
    *p++ = 0x00; *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
    return (size_t)(p - out);
}

/*
 * The subscriptions worth having. Mimo subscribes 53; these are the ones that
 * map to something we display or control. camcap_video_format is resolution +
 * fps, which is NOT reachable through the 0x02/0x8E pid space (a clean 3-sweep
 * A-B-A over 0x00..0x7F showed zero change when resolution was toggled).
 */
const char *const OSMO_CFG_NAMES[] = {
    "camcap_video_format",
    "cam_video_param_v2",
    "cam_status",
    "cam_record_time",
    "cam_storage",
    /* cam_* = current value, camcap_* = the capability list. Subscribing to
     * camcap_fov/camcap_eis alone gives supported modes, never what is set. */
    "cam_fov",
    "cam_lens_state",
    "cam_image_effect",
    "camcap_iso",
    "camcap_color_mode",
    "camcap_wb",
    "cam_custom_mode_params",
    /* Photo-mode counterparts of cam_video_param_v2 / camcap_video_format.
     * Subscribed but not yet decoded — while the camera is in photo mode the
     * video names keep reporting the VIDEO setting, so these are the only
     * source for the photo resolution the UI should be showing. */
    "cam_photo_param_new",
    "camcap_photo_size",
};
const size_t OSMO_CFG_NAMES_COUNT =
    sizeof(OSMO_CFG_NAMES) / sizeof(OSMO_CFG_NAMES[0]);

/*
 * "APP" identity blob: 00 "APP" 00*37 02 00*8 02 08 00*10 (62 bytes).
 * Mirrors what DJI Mimo / osmo-download present as the app's device info.
 */
const uint8_t OSMO_APP_DEVICE_INFO[OSMO_APP_DEVICE_INFO_LEN] = {
    /* [0..3]  type byte + "APP" */
    0x00, 'A', 'P', 'P',
    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0,
    0,    0,
    /* [34] */ 0x02,
    0,    0,    0,    0,    0, 0, 0,
    /* [42..43] */ 0x02, 0x08,
    0,    0,    0,    0,    0, 0, 0, 0, 0, 0,
    0,    0,    0,    0,    0, 0, 0, 0, 0, 0,
};

/*
 * Stable app identifier sent as the first PackString of SetPairingPIN. The
 * camera keys its "already paired" answer to this value, so it must be stable
 * across boots AND unique to this remote — this one is DJI-Remote's own, so
 * the camera no longer confuses it with the Osmosis app (whose
 * "284ae5b8d76b3375a04a6417ad71bea3" identity our cameras had otherwise been
 * auto-pairing to). Changing this forces a fresh on-screen approval once.
 */
static const char OSMO_PAIR_IDENTIFIER[] = "9c2d494210e7e58c72ada9a0ed06e573";

static size_t pack_string(uint8_t *out, size_t cap, const char *s)
{
    size_t len = strlen(s);
    if (len > 0xFF || cap < len + 1) {
        return 0;
    }
    out[0] = (uint8_t)len;
    memcpy(&out[1], s, len);
    return len + 1;
}

size_t osmo_build_pairing_payload(uint8_t *out, size_t out_cap, const char *token)
{
    size_t n = pack_string(out, out_cap, OSMO_PAIR_IDENTIFIER);
    if (n == 0) {
        return 0;
    }
    size_t m = pack_string(out + n, out_cap - n, token);
    if (m == 0) {
        return 0;
    }
    return n + m;
}
