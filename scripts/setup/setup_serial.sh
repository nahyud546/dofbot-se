#!/usr/bin/env bash
# setup_serial.sh — install udev rule for STM32/CH340 on /dev/ttyUSB0 + /dev/myserial.
set -eu
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$ROBOT_ARM_ROOT/config/udev/99-myserial.rules"
SRC="${SRC:-$ROOT/config/udev/99-myserial.rules}"
DST="/etc/udev/rules.d/99-myserial.rules"
echo "[INFO] $SRC -> $DST"
sudo cp "$SRC" "$DST"
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyUSB* /dev/myserial 2>/dev/null || echo "[WARN] device not plugged in"
