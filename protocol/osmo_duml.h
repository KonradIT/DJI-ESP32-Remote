/*
 * Osmo Nano command layer on top of the DUML frame codec (duml.h).
 *
 * Command inventory, addressing and payloads come from the Osmosis
 * reverse-engineering project (MEDIA_PROTOCOL.md + an HCI snoop of DJI Mimo
 * driving a real Osmo Nano).  Three facts rule this protocol:
 *
 *  1. The receiver byte decides everything.  Session commands go to 0xF0,
 *     the wake command to 0x1C, pairing/WiFi to 0x07, camera control to 0x01.
 *     Mis-addressed frames are answered `e0` (reject) with no other hint.
 *  2. 0xFFF5 is write-without-response: back-to-back writes are silently
 *     dropped.  All writes must be paced >= ~100 ms apart (data.c enforces).
 *  3. The camera sends some frames as REQUESTS (flags 0x40) and tears the
 *     link down (~6 s) if they are not answered.  data.c auto-replies.
 *
 * Camera-control over BLE (cmd set 0x02) is derived from WiFi-datalink
 * captures and reference repos; it is NOT yet hardware-verified over BLE on
 * the Nano — this firmware is the experiment.
 */

#ifndef OSMO_DUML_H
#define OSMO_DUML_H

#include <stdint.h>
#include <stddef.h>
#include "duml.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Wire-level packet logging: every TX/RX frame and payload as hex, plus the
 * named-config pushes. Off by default — at ~10 Hz status plus config pushes it
 * is a large amount of log traffic and contributes to notification-queue
 * pressure. Build with -DDEBUG_DUML_PACKETS=1 to turn it on; the RE tooling in
 * tools/re/ parses exactly these lines, so it is required for protocol work
 * (parameter sweeps, A-B-A diffs, decoding an unmapped setting, or bringing up
 * a new camera body such as the Xtra Edge Pro).
 */
#ifndef DEBUG_DUML_PACKETS
#define DEBUG_DUML_PACKETS 0
#endif

/* Flags (cmd_type byte) as seen on the BLE wire */
#define OSMO_FLAGS_NOTIFY     0x00   /* push / fire-and-forget            */
#define OSMO_FLAGS_REQUEST    0x40   /* expects a response                */
#define OSMO_FLAGS_RESPONSE   0xC0   /* response to a 0x40 request        */
#define OSMO_FLAGS_IS_ACK_BIT 0x80   /* set on any response frame         */

/* Session / wake (HCI-snoop-verified on a real Nano) */
#define OSMO_CMDSET_SESSION       0x00
#define OSMO_CMDID_SESSION_PING   0x2B   /* -> DUML_ADDR_SESSION (0xF0)   */
#define OSMO_CMDSET_SYSTEM        0x53
#define OSMO_CMDID_SYSTEM_WAKE    0x10   /* -> DUML_ADDR_SYSTEM (0x1C)    */

/* Device info exchange — the camera asks US via 0x00/0x81 */
#define OSMO_CMDID_DEVICE_INFO    0x81
#define OSMO_CMDID_GET_VERSION    0x00
#define OSMO_CMDID_SESSION_INFO   0x32   /* Mimo sends this to 0x88 post-wake */
/*
 * 0x00/0x99 — named key/value config channel, sent to DUML_ADDR_DM368_1 (0x28).
 * Subscribing to a group makes the camera dump (and then push) that group's
 * whole settings table: cam_status, cam_record_time, cam_video_param_v2,
 * shutter_param, cam_custom_mode_params, camcap_* … 27 items in all. This is
 * the only writable control surface the camera exposes over BLE.
 *
 * Verbatim from a Mimo BLE snoop:
 *   55 17 04 38 02 28 15 c7 40 00 99 | 01 00 06 00 "camera" | e5 af
 * i.e. payload = [verb 0x01 = subscribe][name_len u16-LE][group name ASCII]
 */
#define OSMO_CMDID_CFG_ITEM       0x99
#define OSMO_CFG_VERB_SUBSCRIBE   0x02   /* NOT 0x01 — see osmo_duml.c */

/*
 * Build one per-parameter subscribe payload for 0x00/0x99 -> DUML_ADDR_DM368_1.
 * `sub_id` must differ per subscription (Mimo increments it). Returns the
 * payload length, or 0 if `out` is too small.
 */
size_t osmo_build_cfg_subscribe(uint8_t *out, size_t out_cap, uint32_t sub_id,
                                const char *name);

/* Names we subscribe to (see osmo_duml.c for why these). */
extern const char *const OSMO_CFG_NAMES[];
extern const size_t OSMO_CFG_NAMES_COUNT;

/* Pairing / WiFi subsystem (-> DUML_ADDR_WIFI, 0x07) */
#define OSMO_CMDSET_WIFI          0x07
#define OSMO_CMDID_SET_PAIRING    0x45   /* resp payload[1]: 1=paired, 2=approval popup */
#define OSMO_CMDID_PAIR_APPROVED  0x46   /* arrives as a REQUEST after user approves    */
#define OSMO_CMDID_GET_SSID       0x07
#define OSMO_CMDID_GET_PASSWORD   0x0E
#define OSMO_CMDID_GET_WIFI_MAC   0x0C
#define OSMO_CMDID_WIFI_ENABLE    0x39   /* Mimo sends it; camera rejects (0xE0) even for Mimo */

/* Camera control (-> DUML_ADDR_CAMERA, 0x01), flags 0x40 */
#define OSMO_CMDSET_CAMERA        0x02
#define OSMO_CMDID_TAKE_PHOTO     0x01   /* d9 idle AND recording — not the shutter */
/*
 * 0x02/0x02 is THE RECORD CONTROL on this body, hardware-confirmed:
 * payload [01] starts a recording, [00] stops it (both reply 00).  Documented
 * upstream as a plain "set mode"; it is not.  Not a toggle — re-sending [01]
 * while recording replies df.  See command_logic.c.
 */
#define OSMO_CMDID_SET_MODE       0x02   /* payload [mode:u8]             */
/*
 * Documented record start/stop.  Answer e0 (unsupported) from EVERY receiver
 * (0x01/0x08/0x28/0x48/0x88/0x1C) on Nano + Xtra firmware — they do not exist
 * here.  Kept only so they are not re-derived from the docs as a lead.
 */
#define OSMO_CMDID_RECORD_START   0x20
#define OSMO_CMDID_RECORD_STOP    0x21
#define OSMO_CMDID_STATUS_POLL    0x61
#define OSMO_CMDID_STATUS_PUSH    0x80   /* push: 60 B                    */
/*
 * 0x8E is a camera PARAMETER GET/SET, not a heartbeat (decoded from 3096
 * request/response pairs in a Mimo datalink capture):
 *   GET = 00 01 <pid:u16-LE>          -> 00 00 01 <pid> <len> <value...> | <err>
 *   SET = 01 01 <pid:u16-LE> <len> <value...> -> 00
 * MEDIA_PROTOCOL's "heartbeat 00 01 14 00" is really GET of pid 0x0014.
 * The real session keepalive is 0x00/0x2b [01 01] -> DUML_ADDR_SESSION.
 */
#define OSMO_CMDID_PARAM          0x8E
#define OSMO_CMDID_STATE_QUERY    0xA0   /* resp: 28 B, rec time u16 @6   */
#define OSMO_CMDID_STORAGE_PUSH   0xDC   /* push: both stores             */

/* Battery subsystem pushes (sender DUML_ADDR_BATTERY, 0x05) */
#define OSMO_CMDSET_BATTERY       0x0D
#define OSMO_CMDID_BATTERY_PUSH   0x02   /* 34 B, ~1 Hz                   */

/*
 * ★ SHOOTING MODE — `0x02/0xE1` with a 1-byte payload, to DUML_ADDR_CAMERA.
 *
 * Found by CRC-scanning Mimo's full WiFi capture and tallying every app→camera
 * command (it is absent from the docs).  The camera echoes the same value back
 * in its 0x02/0x80 push at OSMO_STATUS_MODE, so mode is both settable and
 * readable — verified over ~20 writes with zero mismatches, and every value
 * below was then confirmed by selecting that mode by hand and reading the byte.
 *
 * NOTE the enum is sparse and unordered; do not compute it, table it.  Also
 * note ⚠ sweeping unknown values here FROZE a Nano solid — only send these.
 */
#define OSMO_CMDID_SET_SHOOT_MODE 0xE1

typedef enum {
    OSMO_MODE_SLOWMO     = 0x00,
    OSMO_MODE_VIDEO      = 0x01,
    OSMO_MODE_TIMELAPSE  = 0x02,
    OSMO_MODE_PHOTO      = 0x05,
    OSMO_MODE_HYPERLAPSE = 0x0A,
    OSMO_MODE_SUPERNIGHT = 0x28,
} osmo_mode_t;

/* The camera's own carousel order, so cycling matches what the screen does. */
#define OSMO_MODE_CAROUSEL \
    { OSMO_MODE_VIDEO, OSMO_MODE_PHOTO, OSMO_MODE_TIMELAPSE, \
      OSMO_MODE_HYPERLAPSE, OSMO_MODE_SUPERNIGHT, OSMO_MODE_SLOWMO }

const char *osmo_mode_name(uint8_t mode);

/*
 * Camera settings reachable through the 0x02/0x8E parameter channel.
 * Each was pinned by sweeping the pid space, changing the setting by hand and
 * diffing, then confirmed by writing it back.  Settings are stored per shooting
 * mode, so switching modes changes several of these at once.
 */
#define OSMO_PID_FOV        0x0009
#define OSMO_PID_ISO_LIMIT  0x000F

typedef enum {
    OSMO_FOV_WIDE         = 0x01,
    OSMO_FOV_NATURAL_WIDE = 0x05,
} osmo_fov_t;

typedef enum {
    OSMO_ISO_LIMIT_800  = 0x04,
    OSMO_ISO_LIMIT_1600 = 0x05,
} osmo_iso_limit_t;

/*
 * 0x02/0x80 status push payload offsets (60 B, active store only).
 *
 * GROUND-TRUTHED on an Xtra Edge Pro by recording with the camera's own button
 * and diffing the payload (idle -> recording):
 *   byte0   0x01 -> 0x81   ... bit7 = RECORDING
 *   @9      59274 -> 58572 ... free MiB falls while recording
 *   @17     7278  -> 7192  ... remaining recordable seconds, counts down
 *   @29     0x00  -> 0x61  ... elapsed record time, counts up
 * NOTE: the older "recording = byte1 & 0x01" reading (taken from a Pocket 3
 * repo) is WRONG for these cameras — byte1 is 0x02 both idle and recording.
 */
#define OSMO_STATUS_FLAGS_BYTE         0   /* bit7 (0x80) = recording      */
#define OSMO_STATUS_RECORDING_MASK     0x80
#define OSMO_STATUS_STORE_TOTAL_MIB    5   /* u32-LE                       */
#define OSMO_STATUS_STORE_FREE_MIB     9   /* u32-LE                       */
#define OSMO_STATUS_REMAIN_TIME_S     17   /* u16-LE, recordable seconds   */
#define OSMO_STATUS_RECORD_TIME_S     29   /* u16-LE, elapsed seconds      */
/*
 * @57 mirrors the last 0x02/0xE1 value written — i.e. the current shooting
 * mode, in the same osmo_mode_t encoding.  Ground-truthed by selecting each
 * mode on the camera and reading this byte.  Other offsets that move with the
 * mode but are NOT the mode: @4 (1 = video-ish, 0 = photo), @13 u16-LE photos
 * remaining, @17 u16-LE recordable seconds (0 in photo mode).
 */
#define OSMO_STATUS_MODE              57   /* u8, osmo_mode_t              */

/* 0x02/0xA0 state query response offsets (28 B) */
#define OSMO_STATE_RECORD_TIME_S       6   /* u16-LE                      */

/* 0x02/0xDC storage push offsets (both stores, u32-LE MiB) */
#define OSMO_STORAGE_SD_TOTAL          6
#define OSMO_STORAGE_SD_FREE          10
#define OSMO_STORAGE_INT_TOTAL        24
#define OSMO_STORAGE_INT_FREE         28

/* 0x0D/0x02 battery push offsets (34 B) */
#define OSMO_BATT_VOLTAGE_MV           1   /* u16-LE                      */
#define OSMO_BATT_CURRENT_MA           5   /* i32-LE, + charging          */
#define OSMO_BATT_PERCENT             20   /* u8                          */
#define OSMO_BATT_DOCKED              27   /* 0x40 = docked               */
#define OSMO_BATT_CHARGING            32   /* 1 / 0                       */

/* Session ping payloads */
extern const uint8_t OSMO_SESSION_OPEN[2];       /* 04 00 — once, pre-pair */
extern const uint8_t OSMO_SESSION_KEEPALIVE[2];  /* 01 01 — ~1 Hz forever  */
/* 00 01 14 00 = GET camera parameter 0x0014 (see OSMO_CMDID_PARAM) */
extern const uint8_t OSMO_PARAM_GET_0014[4];

/* 62-byte "APP" identity blob answering the camera's 0x00/0x81 request */
#define OSMO_APP_DEVICE_INFO_LEN 62
extern const uint8_t OSMO_APP_DEVICE_INFO[OSMO_APP_DEVICE_INFO_LEN];

/*
 * SetPairingPIN payload: PackString(identifier) + PackString(token).
 * The token is displayed verbatim on the camera screen next to the approve
 * button; the identifier is a stable per-app blob the camera remembers.
 * Returns payload length, 0 if `out` too small.
 */
size_t osmo_build_pairing_payload(uint8_t *out, size_t out_cap, const char *token);

#ifdef __cplusplus
}
#endif

#endif /* OSMO_DUML_H */
