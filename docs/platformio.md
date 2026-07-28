# Building with PlatformIO

Besides the plain ESP-IDF workflow (`idf.py`), the project can be built with
[PlatformIO](https://platformio.org/) (Core >= 6.1). The configuration lives in
[`platformio.ini`](../platformio.ini) at the repo root.

## Platform / IDF version

The environments use the [pioarduino fork](https://github.com/pioarduino/platform-espressif32)
of the `espressif32` platform, because the official PlatformIO platform ships
an ESP-IDF that is too old for this project (needs IDF >= 5.0, CI builds with
5.5.x). The `.../releases/download/stable/platform-espressif32.zip` URL always
resolves to the latest stable pioarduino release (IDF 5.5.x); PlatformIO
downloads it automatically on the first build. To pin an exact version, replace
`stable` in the URL with a release tag from their releases page.

The IDF Component Manager runs as part of the build, so the managed components
declared in `main/idf_component.yml` (`led_strip`, `lvgl`, `esp_lvgl_port`) are
fetched into `managed_components/` automatically. Note: the component manager
rewrites `dependencies.lock` with the IDF version it built against — don't
commit that churn unless intended.

## Environments

| Environment         | Board                                              | Kconfig defaults                        |
| ------------------- | -------------------------------------------------- | --------------------------------------- |
| `m5stack-basic-v27` | M5Stack Basic V2.7 (ESP32, 16 MB flash)            | `sdkconfig.defaults.m5stack_basic_v27`  |
| `waveshare-s3-lcd19`| Waveshare ESP32-S3-LCD-1.9 (16 MB flash, 8 MB OPI PSRAM) | `sdkconfig.defaults.waveshare_s3_lcd19` |

Each environment passes its per-board defaults file to ESP-IDF via
`-DSDKCONFIG_DEFAULTS` (see `board_build.cmake_extra_args`), and both use the
custom `partitions.csv` partition table.

## Commands

```sh
# Build one board
pio run -e m5stack-basic-v27
pio run -e waveshare-s3-lcd19

# Build both (default_envs)
pio run

# Flash + monitor (auto-detects the serial port; add --upload-port COMx if needed)
pio run -e m5stack-basic-v27 -t upload
pio device monitor -b 115200

# The Waveshare board flashes over its native USB Serial/JTAG port
pio run -e waveshare-s3-lcd19 -t upload
```

On Windows, run `pio` from PowerShell/cmd, not from an MSYS/Git-Bash shell —
ESP-IDF's `idf_tools.py` refuses to run under MSYS and the esptool install step
fails there.

## sdkconfig handling

PlatformIO generates a per-environment config file at the repo root
(`sdkconfig.m5stack-basic-v27` / `sdkconfig.waveshare-s3-lcd19`, both
gitignored). The `SDKCONFIG_DEFAULTS` file is only applied when that generated
file does not exist yet. After changing a `sdkconfig.defaults.*` file, delete
the generated per-env file or run:

```sh
pio run -e <env> -t fullclean
```

## Notes

- The project's `CMakeLists.txt` enables `MINIMAL_BUILD`; the environments
  override the component list with `-DCOMPONENTS=main;__pio_env` because the
  PlatformIO builder requires its injected `__pio_env` dummy component. The
  resulting CMake warning ("MINIMAL_BUILD property is disregarded") is
  expected and harmless — the build stays trimmed to `main` + dependencies.
- Build output goes to `.pio/build/<env>/` (`firmware.bin`, `bootloader.bin`,
  `partitions.bin`), which is gitignored. The plain `idf.py` workflow and its
  `build/` directory are unaffected.
