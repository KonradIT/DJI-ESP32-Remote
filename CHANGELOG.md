# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Added

- **DJI Osmo Nano support via the DUML "MEDIA" protocol** — the camera
  protocol was reverse-engineered from the [Osmosis](https://github.com/KonradIT/Osmosis)
  project and reimplemented on-device. See `docs/osmo-nano-protocol.md`.
  - New DUML frame codec (`protocol/duml.*`) — SOF `0x55`, CRC8 (seed `0x77`)
    + CRC16 (seed `0x3692`), big-endian sequence. Verified against captured
    DJI Mimo frames.
  - New Osmo command layer (`protocol/osmo_duml.*`) — addressing, pairing
    payload, session/wake/keepalive constants.
  - Connect flow replaced with the Nano session sequence: session-open →
    SetPairingPIN (on-screen approval) → wake → 1 Hz keepalive, plus auto-ack
    of camera-originated requests.
  - Camera control over DUML: start/stop recording (`0x02/0x20` / `0x02/0x21`),
    take photo (`0x02/0x01`), set/cycle mode (`0x02/0x02`).
  - Status ingestion rewritten for the Nano pushes (`0x02/0x80` recording +
    storage, `0x02/0xA0` record time, `0x02/0xDC` storage, `0x0D/0x02`
    battery).
  - BLE scan classification rewritten to detect if a scanned hit is a DJI camera
    only if it carries the DJI company id (`0x08AA`) or a camera keyword in its
    advertised name, no longer the R-SDK-only `0xFA` byte.
  - Pairing uses this remote's own identifier and the on-screen token `DRMT`
    (not the Osmosis app's shared `osmo` identity).
- **PlatformIO build support** — `platformio.ini` with per-board environments
  alongside the existing ESP-IDF/CMake flow. See `docs/platformio.md`.

### Removed

- **DJI R-SDK protocol layer** — `protocol/dji_protocol_parser.*`,
  `dji_protocol_data_processor.*` and `dji_protocol_data_descriptors.*` are
  deleted, and the DJI CRC16/CRC32 utils are dropped from the build (the Osmo
  Nano does not speak R-SDK). The packed structs in
  `dji_protocol_data_structures.h` are retained as the `command_logic_*`
  return types.

## [v1.2.0]

### Added

- **Waveshare ESP32-S3-LCD-1.9 support** — New HAL, ESP32-S3 target,
  320x170 landscape IPS display (ST7789V2).
- **Dual-target release builds** — Two merged binaries produced per release,
  one per hardware target.
- **UI layout for 320x170** — Adaptive layout system supporting both
  320x240 (M5Stack) and 320x170 (Waveshare) screen resolutions.
- **GPS Kconfig** — GPS UART pins and baud rate configurable per board
  via Kconfig (no hardcoded pin assignments).

### Removed

- **Manual flash ZIP and scripts** — Web-flash only from this release onward.

## [v1.1.0]

### Added

- **LVGL 9.5.0 UI rendering** — All screens replaced with LVGL widget-based
  rendering via esp_lvgl_port 2.7.2. Replaces manual TFT/SPI drawing.
- **NimBLE BLE stack** — Apache NimBLE replaces Bluedroid as the GATT client
  stack.
- **Boot splash screen** — LVGL-rendered splash screen replaces the raw bitmap
  boot logo.
- **Release packaging automation** — GitHub Actions workflow and
  `build_release.sh` produce merged binary and ZIP for every tagged release.

### Fixed

- **NimBLE scan fixes** — Device names now shown during scan; duplicate filter
  disabled; reconnect-on-unpair prevented.

### Changed

- **Production log level** — Default log level set to ERROR for production
  builds.

## [v1.0.0]

Initial public release. Firmware for M5Stack Basic V2.7 (ESP32).

### Features

- Control up to three DJI Osmo Action cameras simultaneously over BLE
- Live GPS forwarding to all connected cameras (10 Hz)
- Start/stop recording, highlight tags, sleep/wake, snapshot-while-sleeping
- Mode switching via QS button emulation
- Automatic boot-time scanning and reconnection
- Multi-camera action coordination with sequential wake queue
- Optional external hardware buttons (GPIO26, GPIO21, GPIO22)
- Supported cameras: Action 4, Action 5 Pro, Action 6, Osmo 360
