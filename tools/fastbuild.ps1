# Fast incremental build + flash for the M5Stack target.
#
# Why: `pio run` re-runs a full CMake configure on every invocation ("Reading
# CMake configuration...") which costs ~2-4 min even for a one-line change.
# idf.py + ninja rebuilds only what changed, typically in seconds.
#
#   .\tools\fastbuild.ps1            # build only
#   .\tools\fastbuild.ps1 -Flash     # build, then flash COM3
#   .\tools\fastbuild.ps1 -Flash -Port COM5
#   .\tools\fastbuild.ps1 -Clean     # force a fresh configure (after sdkconfig edits)
#
# Keep using `pio run` for release/CI verification; this is for iteration.

param(
    [switch]$Flash,
    [switch]$Clean,
    [string]$Port = "COM3",
    [string]$BuildDir = "build_fast"
)

$ErrorActionPreference = "Continue"
$env:IDF_TOOLS_PATH = "C:\Espressif"
$env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.3.2"
$venv = "C:\Espressif\python_env\idf5.3_py3.11_env"
$py = "$venv\Scripts\python.exe"
$env:IDF_PYTHON_ENV_PATH = $venv
$env:PATH = "$venv\Scripts;$env:PATH"

# ccache gives a big win on the first build after a clean and on branch switches.
$env:IDF_CCACHE_ENABLE = "1"

& $py "$env:IDF_PATH\tools\idf_tools.py" export --format key-value | ForEach-Object {
    if ($_ -match '^([A-Za-z_][A-Za-z0-9_]*)=(.*)$') { Set-Item -Path "env:$($matches[1])" -Value $matches[2] }
}

Set-Location $PSScriptRoot\..

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Output "=== removing $BuildDir for a fresh configure ==="
    Remove-Item -Recurse -Force $BuildDir
}

# First run configures; later runs skip straight to ninja.
if (-not (Test-Path "$BuildDir/CMakeCache.txt")) {
    Write-Output "=== configuring ($BuildDir, esp32) ==="
    & $py "$env:IDF_PATH\tools\idf.py" -B $BuildDir `
        -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.m5stack_basic_v27" set-target esp32
}

$sw = [Diagnostics.Stopwatch]::StartNew()
& $py "$env:IDF_PATH\tools\idf.py" -B $BuildDir build
$code = $LASTEXITCODE
$sw.Stop()
Write-Output "=== BUILD_EXIT $code in $([int]$sw.Elapsed.TotalSeconds)s ==="
if ($code -ne 0) { exit $code }

if ($Flash) {
    Write-Output "=== flashing $Port ==="
    & $py -m esptool --chip esp32 --port $Port --baud 460800 `
        --before default_reset --after hard_reset write_flash `
        --flash_mode dio --flash_size 16MB --flash_freq 80m `
        0x1000  "$BuildDir\bootloader\bootloader.bin" `
        0x8000  "$BuildDir\partition_table\partition-table.bin" `
        0x10000 "$BuildDir\dji_camera_bluetooth_control.bin"
    Write-Output "=== FLASH_EXIT $LASTEXITCODE ==="
}
