/*
 * Camera engine interface — one remote, two protocols, side by side.
 *
 * A DJI Osmo body speaks one of two command vocabularies over the same BLE
 * transport (service 0xFFF0, notify 0xFFF4, write 0xFFF5):
 *
 *   "rsdk"  — DJI's documented Osmo GPS-Controller SDK. Frames start 0xAA.
 *             Osmo Action 4 / 5 Pro / 6 / Osmo 360. Also carries GPS push.
 *   "media" — DUML. Frames start 0x55. Osmo Nano, Pocket 3.
 *
 * Everything above this header (UI, logic, screens) talks ONLY to the interface
 * and must never include osmo_duml.h or dji_protocol_*.h. That is the whole
 * point: with a mixed fleet, protocol-specific symbols leaking into the UI make
 * one body render another's semantics.
 *
 * ⚠ The transport is shared and that is HARDWARE-CONFIRMED (OA5 + OA6 + Nano on
 *   one remote, 2026-08-01): all three tolerate the same GATT bring-up, and an
 *   Action's 0xFFF5 reports props=0x36 (WRITE_NO_RSP) exactly like the Nano's.
 *   So engines differ in what they SAY, not in how bytes are moved.
 *
 * ⚠ The engine split is NOT the frame split. An Action camera emits BOTH
 *   framings on one link — 0x55 DUML status pushes AND 0xAA R-SDK command
 *   responses. So the RECEIVE path dispatches on the SOF byte, while COMMANDS
 *   dispatch on the slot's engine. Keying frame decode off the engine would
 *   throw away the DUML status an Action already sends.
 */

#ifndef CAMERA_ENGINE_H
#define CAMERA_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Protocol-neutral shooting mode. Both bodies happen to use the same numeric
 * enum on the wire, but the interface does not promise that — engines map.
 */
typedef enum {
    CAM_MODE_VIDEO,
    CAM_MODE_PHOTO,
    CAM_MODE_TIMELAPSE,
    CAM_MODE_SLOWMO,
    CAM_MODE_HYPERLAPSE,
    CAM_MODE_SUPERNIGHT,
    CAM_MODE_UNKNOWN,
} cam_mode_t;

/* What a body can actually do — replaces device-id special-casing at call sites. */
#define CAM_CAP_GPS         (1u << 0)   /* accepts GPS push (R-SDK only)        */
#define CAM_CAP_HIGHLIGHT   (1u << 1)   /* highlight/tag while recording        */
#define CAM_CAP_SLEEP       (1u << 2)   /* remote sleep/wake                    */
#define CAM_CAP_PHOTO_SIZE  (1u << 3)   /* reports photo size + aspect          */
#define CAM_CAP_EIS         (1u << 4)   /* reports a live EIS/RockSteady value  */

typedef struct camera_engine {
    const char *name;                                  /* "rsdk" | "media" */

    esp_err_t (*session_open)(int slot);               /* post-connect handshake */

    /*
     * Capture. record_start/record_stop are DELIBERATELY separate calls rather
     * than one record(bool): the wire byte is INVERTED between protocols
     * (R-SDK record_ctrl 0 = start, media payload [01] = start). An interface
     * carrying a direction byte would silently do the opposite on one engine.
     */
    esp_err_t (*record_start)(int slot);
    esp_err_t (*record_stop)(int slot);
    esp_err_t (*shoot_photo)(int slot);
    esp_err_t (*set_mode)(int slot, cam_mode_t mode);

    /*
     * Advance to the next shooting mode — what the remote's mode button does.
     *
     * Separate from set_mode because the two families cycle by opposite means:
     * R-SDK sends the QS key report and lets the CAMERA pick the next mode,
     * while media has no "next" opcode at all and must read the current mode
     * back, look it up, and set the successor explicitly. Expressing that as
     * set_mode(current + 1) in shared UI code would be wrong twice over — the
     * media enum is sparse and unordered (0,1,2,5,0x0A,0x28) and its numeric
     * order is not the carousel order the camera shows on screen.
     */
    esp_err_t (*mode_cycle)(int slot);

    uint32_t caps;
} camera_engine_t;

extern const camera_engine_t g_engine_rsdk;
extern const camera_engine_t g_engine_media;

/*
 * Engine assignment. Both routes are POSITIVE identifications — there is no
 * default and no fallback, because guessing wrong sends one protocol's frames
 * to a body that speaks the other.
 *
 *   1. Advert carries DJI mfg data with model id 0x0019 (Osmo Nano)  -> media.
 *   2. Advertised name matches a known media body (Pocket 3, which reportedly
 *      sends no mfg data)                                            -> media.
 *   3. Otherwise ask: the R-SDK Connection Request (0x00/0x19) answers with a
 *      device_id in {0xFF33,0xFF44,0xFF55,0xFF66}                    -> rsdk.
 *   4. Neither -> NULL. Refuse to connect and say so; never pick one.
 *
 * Route 3 is why we need no catalogue of Action-family advertisement ids: the
 * handshake self-identifies and hands us the exact model for free.
 */
#define CAM_ADV_MODEL_OSMO_NANO   0x0019

#define RSDK_DEVICE_ID_ACTION4    0xFF33
#define RSDK_DEVICE_ID_ACTION5    0xFF44
#define RSDK_DEVICE_ID_ACTION6    0xFF55
#define RSDK_DEVICE_ID_OSMO360    0xFF66

/* Decided from the advertisement alone, before connecting. NULL = ask (route 3). */
const camera_engine_t *camera_engine_from_advert(uint32_t adv_model_id,
                                                 const char *adv_name);

/* Decided from an R-SDK connection-request reply. NULL = not an R-SDK body. */
const camera_engine_t *camera_engine_from_rsdk_device_id(uint32_t device_id);

/*
 * Ask a connected body to identify itself via the R-SDK Connection Request
 * (0x00/0x19). ESP_OK plus a device_id means it speaks R-SDK; ESP_ERR_NOT_FOUND
 * means it stayed silent — which is a real answer ("not R-SDK"), not a reason
 * to retry with different bytes.
 */
esp_err_t rsdk_probe_identity(int slot, uint32_t *out_device_id);

/* The engine bound to a slot, or NULL if unresolved. Never guesses. */
const camera_engine_t *camera_engine_for_slot(int slot);
bool camera_engine_slot_has_cap(int slot, uint32_t cap);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_ENGINE_H */
