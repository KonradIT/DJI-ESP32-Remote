# TODO — Osmo Nano / DUML port

Working: record start/stop, take photo, all 6 shooting modes + readback, FOV,
ISO limit, video resolution/fps, photo size + aspect, storage, battery. See
`docs/osmo-nano-protocol.md` and `tools/re/README.md` (RE method + tooling).

## 1. Notification queue overflow — **blocks other work**
`Failed to queue notification data` fires constantly once the `0x00/0x99` config
subscriptions are active, and with two cameras connected. Dropped pushes are why
the EIS A-B-A diff below found nothing — a lost frame is indistinguishable from
"nothing changed".
→ `data/data.c`, the notification queue feeding `process_notification_data()`
(depth//producer in `ble_notify_cb` path). Enlarge, or drain faster, or coalesce.

## 2. EIS (RockSteady) always shows "Off" — **PARKED, don't re-walk this**
`camera_state_t.eis_mode` is never populated, so `eis_mode_to_string(0)` = "Off"
even with RockSteady on.

**Exhausted 2026-07-30.** Three A-B-A runs on hardware, cycling the camera
through Off / RockSteady / HorizonCorrection / HorizonBalancing and back to Off,
with `tools/re/cfgvals.py` and `tools/re/statusdiff.py`. Everything below is
**bit-identical across all states** — these are settled, not "probably not":

| watched | result |
|---|---|
| `cam_image_effect` 16 B | identical |
| `cam_lens_state` 66 B | identical (verified at FULL length) |
| `cam_custom_mode_params` 161 B | identical (verified at FULL length) |
| `v_quality_enhance_status` 1 B | identical |
| `cam_video_param_v2` 10 B | identical (3 unmapped `01`s never move) |
| `cam_fov`, `cam_status`, `cam_storage`, `cam_record_time`, `cam_photo_param_new` | identical |
| `cam_expo_param` 46 B | byte 15 moves — **auto-exposure gain, not EIS** (values interleave in time instead of forming one run per state) |
| `0x02/0x80` status push 60 B | only @17/@18 move = the remaining-seconds countdown, already mapped |

Two false-negative bugs were found and fixed while doing this, so earlier
"nothing changed" results are not trustworthy: the config dump was **capped at
24 B** (so `cam_lens_state` was only ever compared on its first third — cap is
now 192 in `status_logic.c`, keep `cfgvals.py` DUMP_CAP in sync), and
`statusdiff.py` still matched the pre-`cam%d` log format, so it had been
silently finding zero samples.

→ **Only remaining approach:** subscribe to **all 53** names Mimo uses (list is in
`Osmosis/MEDIA_PROTOCOL.md` §8) and run ONE A-B-A — not another candidate guess,
which failed three times. There is no `cam_eis` in those 53 names, only the
`camcap_eis` capability table, so the live value may simply not be exposed on
the BLE config channel; if all 53 come back flat, that is the answer and this
item should be closed as "not available over BLE".
Handler if found: `logic/status_logic.c`, the `OSMO_CMDID_CFG_ITEM` branch.

## 3. Sleep command doesn't work on Nano
Need to find the command for sleep camera, it doesn't work now.

## 4. Highlights
Maybe the command written in the osmo r-sdk works in media protocol too.

---

Scope: **DJI Osmo bodies over BLE only.** Non-DJI rebrands are out — one was
tested and its BLE surface exposes pairing, wake, config and status but no
command receiver, so it cannot be controlled over this transport at all. Adding
one back means restoring its advertising match in `ble/ble.c is_dji_camera_adv()`
and expecting control to need the WiFi datalink instead.

Debug logging for protocol work:

```
 -DDEBUG_DUML_PACKETS=1
```