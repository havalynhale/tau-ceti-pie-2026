#!/bin/bash
# Compile a sketch on Windows, then flash it from WSL (the ESP32 is attached to WSL via usbipd).
#   Windows:  arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir build inverse_v2_balance_ros
#   WSL:      ./ros2/flash_esp32.sh build inverse_v2_balance_ros
# Needs esptool:  python3 -m venv ~/.venvs/esptool && ~/.venvs/esptool/bin/pip install esptool
set -e
B=${1:?build dir}
S=${2:?sketch name}
PORT=${PORT:-/dev/ttyUSB0}
WINUSER=${WINUSER:-$(cmd.exe /c "echo %USERNAME%" 2>/dev/null | tr -d '\r')}
A=/mnt/c/Users/$WINUSER/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.17/tools/partitions/boot_app0.bin
~/.venvs/esptool/bin/esptool --chip esp32 --port "$PORT" --baud 921600 write-flash -z \
  0x1000 "$B/$S.ino.bootloader.bin" \
  0x8000 "$B/$S.ino.partitions.bin" \
  0xe000 "$A" \
  0x10000 "$B/$S.ino.bin"
