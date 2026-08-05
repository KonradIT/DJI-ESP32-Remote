# Osmo Nano DUML protocol port

This firmware was originally written against DJI's **R-SDK** (frame SOF `0xAA`,
CRC16+CRC32, CmdSets `0x00`/`0x1D`), which the DJI Osmo Nano does **not**
support. It now speaks the **DUML "MEDIA" protocol** (frame SOF `0x55`,
CRC8+CRC16) that the Nano actually uses, reverse-engineered in the
[Osmosis](https://github.com/KonradIT/Osmosis) project (`MEDIA_PROTOCOL.md`).

The BLE transport is unchanged — the Nano exposes the same GATT service the
old code already used: service `0xFFF0`, write-without-response `0xFFF5`,
notify `0xFFF4`.

## Frame format (`mediaprotocol/duml.{c,h}`)

```
55 | len:u16(10-bit, ver=1) | crc8(bytes 0-2) | src | dst | seq:u16-BE
   | flags | cmd_set | cmd_id | payload | crc16:u16-LE(whole frame)
```

- **CRC8**: poly `0x8C` (reflected `0x31`), seed `0x77`, over bytes 0–2.
- **CRC16**: poly `0x8408` (reflected `0x1021`, KERMIT), seed `0x3692`, over
  the whole frame minus the trailing 2 bytes.
- **seq** is big-endian on the BLE wire; the camera echoes it in responses.
- Both CRC parameters and the layout were verified byte-for-byte against eight
  captured DJI Mimo frames (see the host test in the commit that added
  `duml.c`).

### Addressing (`(id << 5) | type`)

The **destination byte decides everything**; a mis-addressed frame is answered
`e0` (reject) with no other diagnostic.

| Target | dst byte | Used for |
|--------|----------|----------|
| Camera | `0x01` | camera control (record, mode, photo, status) |
| WiFi   | `0x07` | pairing (`SetPairingPIN`) |
| Session| `0xF0` | session open / keepalive (`0x00/0x2b`) |
| System | `0x1C` | wake (`0x53/0x10`) |

## Connection flow (`logic/connect_logic.c`)

`connect_logic_protocol_connect()` replaces the old R-SDK `0x00/0x19`
handshake with the Nano session sequence:

1. **Session open** — `0x00/0x2b [04 00]` → `0xF0` (before pairing).
2. **SetPairingPIN** — `0x07/0x45` → `0x07`, payload = `PackString(id) +
   PackString("DRMT")`. Reply `00 01` = already paired, `00 02` = approval
   popup (the token `DRMT` is shown on the camera screen); on approval the
   camera sends a `0x07/0x46` **request** which the data layer auto-acks. The
   identifier is DJI-Remote's own (`9c2d…e573`), distinct from the Osmosis
   app's, so cameras pair to this remote as a separate device rather than
   reusing Osmosis's "already paired" identity.
3. **Wake** — `0x53/0x10 [00 00 00 00]` → `0x1C`.
4. **Keepalive** — a 1 Hz task pings `0x00/0x2b [01 01]` → `0xF0` for every
   connected slot, forever. Without it the Nano drops the paired link after
   ~5–6 s of silence.

The camera also issues its own **requests** (`flags 0x40`), e.g. a `0x00/0x81`
device-info exchange. `data.c` auto-answers every request (`flags 0xC0`, echoed
seq, echoed payload — or the "APP" identity blob for `0x00/0x81`); if it does
not, the camera tears down the link.

## Camera control (`logic/command_logic.c`)

CmdSet `0x02`, `flags 0x00` (fire-and-forget — matches the captured frames):

| Action | cmd | UI entry point |
|--------|-----|----------------|
| Start recording | `0x02/0x20` | `command_logic_start_record[_async]` |
| Stop recording | `0x02/0x21` | `command_logic_stop_record[_async]` |
| Take photo | `0x02/0x01` | `command_logic_take_photo` |
| Set mode | `0x02/0x02 [mode]` | `command_logic_switch_camera_mode` |
| Quick-switch mode | `0x02/0x02` (ring) | `command_logic_key_report_qs` |

Mode byte for `0x02/0x02`: `0` Photo, `1` Video, `2` Playback, `3` SlowMo,
`4` Timelapse, `5` Panorama. The UI's `camera_mode_t` (R-SDK values) is mapped
to these in `osmo_mode_from_camera_mode()`. Quick-switch cycles a small ring
(Video → Photo → Timelapse → SlowMo) because DUML has no "next mode" key.

Recording/mode state is **never** inferred from a command response (these are
fire-and-forget); it comes from the status pushes below.

## Status (`logic/status_logic.c`, `data.c`)

The Nano pushes status on its own once the session is alive. `data.c` routes
each push to `update_camera_state_handler()` as a small tagged blob:

| Push | Fields consumed |
|------|-----------------|
| `0x02/0x80` camera status (60 B) | recording bit `@1`, SD free MiB `@9` |
| `0x02/0xA0` state query (28 B) | record time (s) `@6` |
| `0x02/0xDC` storage | SD free `@10` / internal free `@28` MiB |
| `0x0D/0x02` battery (34 B) | battery percent `@20` |

## What is intentionally best-effort

- **Sleep** (`command_logic_power_mode_switch_sleep`) has no verified BLE
  command on the Nano; it nudges the session and lets the camera's own idle
  timeout take over.
- **GPS push** is a no-op stub (the R-SDK `0x00/0x17` frame is not part of the
  verified Nano set).

## R-SDK code removed from the build

`protocol/dji_protocol_parser.*`, `dji_protocol_data_processor.*`,
`dji_protocol_data_descriptors.*`, `dji_protocol_data_structures.c` and the
DJI CRC16/CRC32 utils are no longer compiled. `dji_protocol_data_structures.h`
is kept — its packed structs are still the return types of the
`command_logic_*` API the UI depends on.
