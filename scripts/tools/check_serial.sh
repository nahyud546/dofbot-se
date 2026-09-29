#!/usr/bin/env bash
# check_serial.sh — verify STM32 controller reachable, no servo motion.
set -u
PORT="${1:-/dev/ttyUSB0}"
"$(dirname "${BASH_SOURCE[0]}")/../../.venv/bin/python" - "$PORT" <<'PY'
import sys
port = sys.argv[1]
try:
    import Arm_Lib
    arm = Arm_Lib.Arm_Device(port)
    print(f"[OK] Arm_Lib open {port}")
except Exception as e:
    print(f"[FAIL] {port}: {e}")
    sys.exit(1)
PY
