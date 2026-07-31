# PLAN — R-SDK and Media protocol, side by side

**Goal.** One remote drives a mixed fleet: e.g. 2× Osmo Action 5 Pro on **R-SDK**
(GPS push + control) and 1× Osmo Nano on the **Media protocol** (DUML). Pressing
"all cameras" fires the right frames to each. The backend is **hard-pinned** from
camera identity — never guessed, never defaulted.

**Method.** Stages, each a branch off `main`, each independently buildable and
testable. `media-protocol` is the source of the DUML implementation; `main` is
the source of the R-SDK implementation.

---

## 1. Identity model — two ids, different layers, do not conflate

| | source | known | values | decides |
|---|---|---|---|---|
| **`adv_model_id`** | BLE advert, DJI mfg data `mfg[2..3]` u16-LE, company id `0x08AA` | at **scan**, before connecting | `0x0019` Osmo Nano | **which backend** |
| **`device_id`** | R-SDK Connection Request/Result (`0x00/0x19`), 4-byte field at payload offset 0 | after **connecting** | `0xFF33` A4 · `0xFF44` A5 Pro · `0xFF55` A6 · `0xFF66` Osmo 360 | model name, capabilities (e.g. highlight) |

- `0x0019` is **hardware-confirmed** in our own scan logs (`OsmoNano-C2D8`,
  MAC `8C:58:23:7B:C2:D9`). `0xFF33/44/55/66` are confirmed in DJI's docs **and**
  in `main`'s code (`ui_get_camera_model_name()`, the highlight gate).
- Other adv model ids (`0x0014` A4, `0x0015` A5P, `0x0018` A6, `0x0020` Pocket 3)
  come from an **external scanner catalogue, not this repo** — a search of both
  branches found none of them. Treat as unverified until seen on air.
- `main` assigns `camera_state_t.device_id` from `camera_request->device_id`
  (`connect_logic.c:522`) and persists it with the NVS pairing. **The media
  branch dropped this** (`(void)device_id;`), which is why the highlight gate
  (`0xFF33/44/55`) never fires and the Tag button never appears on a Nano.

### Resolution rules (no fallback)

1. `adv_model_id == 0x0019` → **MEDIA**. Pin at pairing, persist with the pairing.
2. Advert carries DJI mfg data with any other model id → **RSDK**; confirm the
   model from the connection result's `device_id` after connecting.
3. **Pocket 3 exception** — reportedly advertises *no* manufacturer data, so it
   is pinned by advertised **name**. This is the one name-based route and is
   documented as such. ⚠ **Test this before relying on it** — the claim is
   unverified and we expect to find adv data.
4. Anything else → **UNKNOWN**: refuse to connect and say so in the UI. No
   default backend.

---

## 2. Command matrix — what actually differs

| op | R-SDK | Media (hardware-confirmed) |
|---|---|---|
| take photo | **No dedicated opcode.** Record Control `0x1D/0x03` with `record_ctrl=0x00` *while the camera is in Photo mode*; the camera decides photo-vs-record from its own mode. Separate sleep path: Key Report `0x00/0x11` `key_code=0x03` (SNAPSHOT). | `0x02/0x01` payload `[01]`. Dedicated. Empty payload → `e3`; wrong mode → `d9`. |
| start record | `0x1D/0x03`, `record_ctrl = 0x00` | `0x02/0x02` payload `[01]` |
| stop record | `0x1D/0x03`, `record_ctrl = 0x01` | `0x02/0x02` payload `[00]` |
| change mode | `0x1D/0x04`, `[device_id:u32][mode:u8][reserved:4]`, response `ret_code` | `0x02/0xE1` payload `[mode:u8]` |

**⚠ The start/stop byte is INVERTED between protocols.** R-SDK `0` = start / `1` =
stop; Media `[01]` = start / `[00]` = stop. Any abstraction that passes a raw
"direction" byte through will silently do the opposite on one backend. The seam
must take an *intent* (`START`/`STOP`), never a byte.

**Mode enum values are shared** — SlowMo `0x00`, Video `0x01`, TimeLapse `0x02`,
Photo `0x05`, HyperLapse `0x0A`, SuperNight `0x28` — same in R-SDK's
`camera_mode_t` and the Nano's `0x02/0xE1`. Only the opcode differs.

**Dead code to route around:** `command_logic_switch_camera_mode()` still calls
`OSMO_CMDID_SET_MODE` (`0x02/0x02`) — which on the Nano is the *record control*,
not a mode switch. The correct path is `command_logic_set_shoot_mode()`
(`0x02/0xE1`). Delete or fix during Stage 3.

**Neither protocol's command ack means the state changed.** Both give a
synchronous accept/reject and confirm asynchronously via a status push (~860 ms
to set the record bit, ~2.4 s to clear). Build the seam around that.

**R-SDK only:** GPS push `0x00/0x17`, power/sleep `0x00/0x1A`, highlight
(Key Report `0x00/0x11` `key_code=0x02`, gated to `0xFF33/44/55`), explicit
status subscribe `0x1D/0x05`.

---

## 3. Verified blockers (found by audit, confirmed in code)

| # | blocker | evidence |
|---|---|---|
| B1 | **`send_command()` / `generate_seq()` collide.** Non-static in both `command_logic.c`s with different arity. Restoring one alongside the other is a linker failure. | `main:command_logic.c:68,126` vs `command_logic.c:47,128` |
| B2 | **Stage 0 is not independent of the ble.c work.** `main`'s advert filter requires `mfg[4] == 0xFA`, which the Nano does not set — so on pristine `main` the Nano is rejected *before* `adv_model_id` is computed. | `main:ble/ble.c:193` and `:204` |
| B3 | **`ble_register_notify()` is shared and unconditional** — dual-CCCD subscribe + the `01 00` arm write to `0xFFF4` fire for every camera. It is called from `connect_logic.c` before any command-layer dispatch, so a seam in `command_logic_*` does not cover it. | `ble.c` register path; `connect_logic.c:332,427` |
| B4 | **`data/data.c` can only exist once** — it owns the single global BLE notify callback and discards any frame not starting `0x55`. It cannot be "restored from main" *and* have a DUML twin. | `data.c:1097`, single `ble_set_notify_callback()` |
| B5 | **UI reaches past the seam into DUML directly** — `ui_screen_main.c`, `ui_screen_mode_switch.c`, `ui.c` call `osmo_mode_name()`, `OSMO_MODE_PHOTO`, `osmo_photo_size_name()`. With a mixed fleet these render DUML names for R-SDK cameras. | `ui_screen_mode_switch.c:15,185,197,201`; `ui_screen_main.c:428-441`; `ui.c:2588` |

**Shared-struct hazards** (`camera_state_t`, silent corruption not compile errors):

- `camera_mode` — written by every status push on R-SDK; on media only written
  after *we* send a mode command, so it goes stale if the user changes mode on
  the camera. Already a live bug today.
- `shoot_mode` — media-only, permanently `0` for R-SDK.
- `eis_mode` — never assigned on media; permanently `0`.
- `camera_supports_new_status_push` / `mode_name` / `mode_param` — R-SDK-era; the
  callback that would set them is registered but never invoked on media.
- `video_resolution` / `fps_idx` — different *sources*, but the resolution code
  space appears shared (the `short_resolution()` table carried over unchanged and
  only gained code `12`). Photo sizes are **not** shared — hence `photo_size` /
  `photo_aspect`.

**Not a problem:** flash/RAM. 16 MB flash, 2 MB factory partition, currently
~1.1 MB. `protocol/dji_protocol_data_structures.h` was never deleted and is
already included by the media `command_logic.h`, so Stage 1 is a clean re-add of
6 files + 4 CMakeLists lines.

---

## 4. Stages

Each stage: branch off `main` (or the previous stage), build green, and where
noted, verify on hardware before proceeding.

### Stage 0 — Trustworthy identity
- Cherry-pick the `dji_mfg_matches()` relaxation (drop `mfg[4] == 0xFA`) — **B2**.
- Add `adv_model_id` to `camera_state_t`, distinct from `device_id`; merge advert
  records per MAC so the mfg-bearing one wins (the Nano emits two records under
  one MAC and only `DJI Camera` carries mfg data).
- Persist `adv_model_id` with the NVS pairing.
- Add `cam_backend_t { RSDK, MEDIA, UNKNOWN }` and the resolution rules above.
  UNKNOWN refuses to connect.
- Restore `device_id` assignment from the R-SDK connection result.
- **Hardware gate:** pair a Nano and an Action; confirm each resolves to the right
  backend across a reboot. **Test whether a Pocket 3 really sends no mfg data.**

### Stage 1 — Restore the R-SDK layer
- `git checkout main -- protocol/dji_protocol_*` (6 files), re-add CMakeLists SRCS.
- Keep `main`'s R-SDK `logic/*_logic.c` as the R-SDK implementations.
- Build green with both trees present but only R-SDK wired.

### Stage 2 — Relocate the media protocol + break the symbol collision
- `protocol/{duml,osmo_duml}.{c,h}` → `mediaprotocol/`.
- **Rename the media internals during the move** — `send_command` →
  `media_send_command`, `generate_seq` → `media_generate_seq` — **B1**. Do this
  as part of the move, not later.
- The DUML command/status implementations become `mediaprotocol/*_media.c`.

### Stage 3 — Dispatch seam
- Per-slot backend resolved once, at pairing.
- Public `command_logic_*` API unchanged; each entry point dispatches.
- **Intent-based, not byte-based** — `RECORD_START`/`RECORD_STOP`, never a raw
  direction byte (see the inversion above).
- Extend the seam **below** command_logic: `ble_register_notify()` gains backend
  awareness so R-SDK cameras don't get the Nano's arm/dual-CCCD sequence — **B3**.
- Delete/fix `command_logic_switch_camera_mode()`.

### Stage 4 — Receive path
- `data/data.c` is rewritten as a **new dispatcher** — not a restore of either
  version — **B4**. Per-slot: `0x55` → DUML parse, `0xAA` → R-SDK parse.
- Keep the existing seq/cmd-id matching table; it is already protocol-agnostic.
- Each backend fills the shared `camera_state_t`; add per-backend "field is live"
  discipline so dead fields aren't read as data.

### Stage 5 — Commands and fan-out
- "All cameras" paths iterate slots and dispatch per backend.
- **Hardware gate:** 2× R-SDK + 1× Media, record start/stop and mode change all
  fire correctly and concurrently.

### Stage 6 — Capabilities and UI
- Per-backend capability flags replace device-id special-casing (highlight, GPS,
  sleep, EIS-liveness).
- **UI files must branch on backend** — `ui_screen_main.c`,
  `ui_screen_mode_switch.c`, `ui.c` — **B5**. This is real scope, not a tidy-up.
- GPS stays R-SDK-only.

---

## 5. The open risk that sizes the job

`ble/ble.c` is shared and diverged by 476 lines. Three changes are media-specific
and unconditional: **dual-CCCD subscribe**, the **`01 00` arm write to `0xFFF4`**,
and **MTU 500** (down from 517). Two more are global posture changes with no
R-SDK verification: the **20 s supervision-timeout floor** and **SM/bonding
enabled** (`main` sets `sm_bonding = 0` deliberately: "DJI cameras operate without
BLE-level encryption"). `main` also uses **Write Request** on `0xFFF5` while the
media path uses **Write Without Response** — the Nano's `0xFFF5` is
`props=0x36` (WRITE_NO_RSP only), but Action-series characteristic properties
have not been checked.

None of this is answerable by reading code. **Put an Action 4/5/6 on the current
branch's `ble.c` and see what happens.** If it connects and records, `ble/` stays
shared and this is the 7-stage job above. If it doesn't, Stage 3 grows a
per-backend transport profile and the estimate roughly doubles.

Do that test **before** Stage 1.
