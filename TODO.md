# TODO — Osmo Nano / DUML port

Working: record start/stop, all 6 shooting modes + readback, FOV, ISO limit,
resolution/fps, storage, battery. See `docs/osmo-nano-protocol.md` and
`tools/re/README.md` (RE method + tooling).

## 1. Notification queue overflow — **blocks other work**
`Failed to queue notification data` fires constantly once the `0x00/0x99` config
subscriptions are active, and with two cameras connected. Dropped pushes are why
the EIS A-B-A diff below found nothing — a lost frame is indistinguishable from
"nothing changed".
→ `data/data.c`, the notification queue feeding `process_notification_data()`
(depth//producer in `ble_notify_cb` path). Enlarge, or drain faster, or coalesce.

## 2. EIS (RockSteady) always shows "Off"
`camera_state_t.eis_mode` is never populated, so `eis_mode_to_string(0)` = "Off"
even with RockSteady on. Not in the two mapped bytes of `cam_video_param_v2`
(`67 02 | 00 00 04 02 01 00 01 01` — three unmapped `01`s; do NOT guess).
→ Decode candidates already subscribed: `cam_image_effect` (16 B), `cam_lens_state`
(66 B). Method: fix #1, then toggle RockSteady off/on and diff — `tools/re/`.
Handler: `logic/status_logic.c`, the `OSMO_CMDID_CFG_ITEM` branch.

## 3. Xtra Edge Pro ignores all camera commands
Not a pairing/encryption/sleep problem. The Xtra is app-paired (`0x07/0x45` →
`00 01`), answers the `0x53/0x10` wake, and streams `0x02/0x80` status we decode
correctly (its record indicator + timer work). But **every frame sent to receiver
`0x01` gets no reply at all** — 4× `0x02/0x02` record, zero responses, while the
Nano on the same firmware/session answered each one. Per the reply-byte oracle,
silence = that receiver isn't answering, so the Action-family command set (and
possibly the camera receiver address) differs from the Nano's.
→ `0x02/0x02` record and `0x02/0xE1` mode are **Nano-verified only**. No Xtra
capture exists (all of `Osmosis/*.pcap` checked — `btsnoop_nano_xtra.log` is
misnamed, both sessions are the Nano). Needs a btsnoop of Mimo driving the Xtra,
or read-only receiver probing. ⚠ Never value-sweep a command — that froze a Nano.

Needs to enable debugging:

```
 -DDEBUG_DUML_PACKETS=1
```

## 4. Take photo — command unknown
`0x02/0x01` exists but rejects every argument (`e3` empty, `df` `[00]`, `d9`
`[01]`). Both reference repos send `0x02/0x01` empty and admit it's an unverified
guess. Lead: `lib-osmo-ble/PROTOCOL.md` documents `0x02/0xE1` payload `[0x1A]` as
*PrepareToLiveStream* — the same cmdId we proved sets the shooting mode, so
`0xE1` is likely a general camera-action command and photo may be another value.
→ Shutter currently sends record-start in Photo mode: `logic/command_logic.c`
`command_logic_take_photo()`. Retest `0x02/0x01` **while in Photo mode** (its
`d9` = wrong-state replies were all recorded in video modes).

## 5. Misc stuff:
- Resolution code `0x0c` (12) unidentified; the other 5 map to the existing enum.

# 6. Sleep command doesn't work on Nano:

Need to find the command for sleep camera, it doesn't work now.

# 7. Highlights:

Maybe the command written in the osmo r-sdk works in media protocol too.