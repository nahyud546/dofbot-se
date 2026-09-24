#!/bin/bash
# run_yahboom.sh — chạy YahboomArm.pyc ổn định trên Ubuntu 22.04 bare-metal
# - Dùng .venv làm Python chính
# - Không move/xóa source gốc; chỉ set PYTHONPATH + source ROS khi cần
# - Không gửi lệnh servo ngẫu nhiên; chỉ start server Yahboom
set -u

PROJECT="/home/jloy/Desktop/robot-arm"
VENV="$PROJECT/.venv"
PYBIN="$VENV/bin/python"
PYC="$PROJECT/rosmaster/YahboomArm.pyc"
UDEV_SRC="$PROJECT/99-myserial.rules"
UDEV_DST="/etc/udev/rules.d/99-myserial.rules"

echo "===== YahboomArm launcher ====="
echo "[INFO] PROJECT=$PROJECT"
echo "[INFO] Python chính: $PYBIN"
"$PYBIN" -V || { echo "[FAIL] không thấy .venv python"; exit 1; }

# 1) Activate .venv
# shellcheck disable=SC1091
source "$VENV/bin/activate"
echo "[OK] activated .venv: $(which python) ($(python -V 2>&1))"

# 2) Source ROS / workspace — chỉ khi thật sự cần, tránh override Arm_Lib serial trong .venv
# - /opt/ros/humble: cần cho rclpy, cv_bridge... (đã có sẵn qua ~/.bashrc nhưng source lại cho chắc)
# - dofbot_ws/install: cần cho dofbot_interface.srv Kinemarics (snake_ctrl/color_identify/color_stacking/garbage)
# - KHÔNG source colcon_ws/install: nó chứa Arm_Lib 1.0.3 có thể shadow Arm_Lib 0.0.5 trong .venv
#   (cả 2 đều là bản serial, nhưng giữ .venv làm chuẩn theo yêu cầu #6)
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # setup.bash của ROS dùng biến chưa set (AMENT_TRACE_SETUP_FILES) nên phải tắt set -u tạm thời
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/humble/setup.bash"
  set -u
  echo "[OK] sourced /opt/ros/humble/setup.bash"
else
  echo "[WARN] thiếu /opt/ros/humble/setup.bash (rclpy có thể fail)"
fi
if [ -f "$PROJECT/dofbot_ws/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "$PROJECT/dofbot_ws/install/setup.bash" 2>/dev/null || true
  set -u
  echo "[OK] sourced dofbot_ws/install/setup.bash (cho dofbot_interface)"
else
  echo "[WARN] thiếu $PROJECT/dofbot_ws/install/setup.bash"
fi
echo "[INFO] KHÔNG source colcon_ws/install để tránh override Arm_Lib trong .venv"

# 3) PYTHONPATH — map /home/yahboom/* sang /home/jloy/Desktop/robot-arm/* (scan từ disassembly YahboomArm.pyc)
# Yahboom gốc sys.path.insert/append:
#   /home/yahboom/Dofbot/0.py_install
#   .../dofbot_color_identify/scripts, .../dofbot_color_follow, .../face_follow/scripts,
#   .../color_stacking/scripts, .../garbage_yolov11, .../snake_follow/scripts,
#   .../pro_interface, .../color_sorting/scripts, .../dofbot_utils/src,
#   .../color_grab/scripts, .../dofbot_action_group, .../dofbot_gesture
# Local tương ứng + bổ sung:
#   - rosmaster (cho camera.VideoCamera)
#   - dofbot_ws/src/dofbot_sorting_3d (cho dofbot_sorting_3d.media_library mà gesture_stack import)
#   - dofbot_color_follow/scripts (bổ sung; gốc chỉ có root)
#   - BỎ dofbot_pro_interface (chỉ là msg/srv ROS, không phải Python import)
#   - BỎ dofbot_action_group (không tồn tại local; action_group.py nằm trong color_grab/scripts)
#   - BỎ build/install/log/__pycache__/.git theo yêu cầu #4
PP_ROSMASTER="$PROJECT/rosmaster"
PP_PYINSTALL="$PROJECT/Dofbot/0.py_install"
PP_CALIB="$PROJECT/colcon_ws/src/dofbot_color_identify/scripts"
PP_FOLLOW="$PROJECT/colcon_ws/src/dofbot_color_follow"
PP_FOLLOW_SCRIPTS="$PROJECT/colcon_ws/src/dofbot_color_follow/scripts"
PP_FACE="$PROJECT/colcon_ws/src/dofbot_face_follow/scripts"
PP_STACK="$PROJECT/colcon_ws/src/dofbot_color_stacking/scripts"
PP_GARBAGE="$PROJECT/colcon_ws/src/dofbot_garbage_yolov11"
PP_SNAKE="$PROJECT/colcon_ws/src/dofbot_snake_follow/scripts"
PP_SORTING="$PROJECT/colcon_ws/src/dofbot_color_sorting/scripts"
PP_UTILS="$PROJECT/colcon_ws/src/dofbot_utils/src"
PP_GRAB="$PROJECT/colcon_ws/src/dofbot_color_grab/scripts"
PP_GESTURE="$PROJECT/colcon_ws/src/dofbot_gesture"
PP_SORT3D="$PROJECT/dofbot_ws/src/dofbot_sorting_3d"

export PYTHONPATH="$PP_ROSMASTER:$PP_PYINSTALL:$PP_CALIB:$PP_FOLLOW:$PP_FOLLOW_SCRIPTS:$PP_FACE:$PP_STACK:$PP_GARBAGE:$PP_SNAKE:$PP_SORTING:$PP_UTILS:$PP_GRAB:$PP_GESTURE:$PP_SORT3D:${PYTHONPATH:-}"
echo "[OK] PYTHONPATH set:"
echo "$PYTHONPATH" | tr ':' '\n' | sed 's/^/  - /'

# 4) Kiểm tra /dev/myserial (CH340 1a86:7523)
echo "===== Serial check ====="
ls -l /dev/ttyUSB0 /dev/myserial 2>&1 || true
if [ -e "/dev/myserial" ]; then
  echo "[OK] /dev/myserial tồn tại -> $(readlink -f /dev/myserial)"
  if [ -r "/dev/myserial" ] && [ -w "/dev/myserial" ]; then
    echo "[OK] /dev/myserial readable+writable (user $(whoami) dialout: $(groups | grep -o dialout || echo 'NOT in dialout'))"
  else
    echo "[FAIL] /dev/myserial không rw cho user $(whoami). Kiểm tra group dialout + MODE 0660."
    ls -l /dev/myserial /dev/ttyUSB0 2>&1 || true
  fi
else
  echo "[FAIL] /dev/myserial chưa tồn tại."
  if [ -e "/dev/ttyUSB0" ]; then
    echo "[INFO] /dev/ttyUSB0 có, sẽ thử symlink tạm (không bền vững, cần udev):"
    echo "  sudo ln -sf /dev/ttyUSB0 /dev/myserial"
  fi
fi
# Kiểm tra rule udev bền vững
if [ -f "$UDEV_DST" ]; then
  echo "[OK] udev rule đã cài: $UDEV_DST"
  cat "$UDEV_DST"
else
  echo "[WARN] chưa có $UDEV_DST (symlink hiện tại sẽ mất sau reboot)."
  echo "[INFO] rule mẫu đã tạo tại $UDEV_SRC. Cài 1 lần bằng:"
  echo "  sudo cp $UDEV_SRC $UDEV_DST"
  echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
  echo "  ls -l /dev/myserial"
fi
lsusb -d 1a86:7523 2>&1 | head -n 5 || echo "[WARN] không thấy CH340 1a86:7523 qua lsusb"

# 5) Kiểm tra Arm_Lib đúng bản serial (không phải I2C)
echo "===== Arm_Lib check (phải là bản serial) ====="
"$PYBIN" -c "
import inspect, Arm_Lib
from Arm_Lib import Arm_Device
src = open(inspect.getsourcefile(Arm_Device)).read()
ok_serial = ('import serial' in src)
ok_com = ('/dev/myserial' in src)
ok_baud = ('115200' in src)
print('[OK] Arm_Lib file:', inspect.getsourcefile(Arm_Device) if ok_serial else 'FAIL')
print(('[OK]' if ok_serial else '[FAIL]') + ' có import serial')
print(('[OK]' if ok_com else '[FAIL]') + ' default com=/dev/myserial')
print(('[OK]' if ok_baud else '[FAIL]') + ' baud 115200')
import sys
sys.exit(0 if (ok_serial and ok_com and ok_baud) else 1)
" || echo "[FAIL] Arm_Lib không đúng bản serial — kiểm tra lại .venv (không để ROS override)"

# 6) Test import 10 module bắt buộc, in [OK]/[FAIL] rõ ràng
echo "===== Import check (10 module bắt buộc) ====="
"$PYBIN" -c "
mods = ['flask','cv2','serial','Arm_Lib','camera','action_group','color_grab','dofbot_utils','dofbot_sorting_3d','gesture_stack']
import importlib
fail = 0
for m in mods:
    try:
        importlib.import_module(m)
        print(f'[OK] {m}')
    except Exception as e:
        fail += 1
        print(f'[FAIL] {m}: {type(e).__name__}: {e}')
raise SystemExit(fail)
"
IMPORT_RC=$?
if [ $IMPORT_RC -ne 0 ]; then
  echo "[FAIL] Có $IMPORT_RC/10 module fail. Không pip install bừa module local Yahboom."
  echo "[INFO] Tự tìm file/package tương ứng trong project rồi sửa PYTHONPATH."
  echo "[INFO] Chỉ pip install nếu là package ngoài thật sự (mediapipe/scipy/cv_bridge/ultralytics/torch/baidu-aip/demjson3...)."
fi

# 7) Test mở rộng: 15 import thật sự mà YahboomArm.pyc dùng (giúp debug, không bắt buộc pass 100% để chạy Flask?)
echo "===== Import mở rộng (15 import của YahboomArm.pyc) ====="
"$PYBIN" -c "
tests=[
 ('action_group','action_group'),
 ('color_grab','Color_Grab'),
 ('gesture_stack','gesture_stack'),
 ('gesture_action','gesture_action'),
 ('snake_ctrl','snake_ctrl'),
 ('snake_target','snake_target'),
 ('color_sorting','color_sorting'),
 ('face_follow','Face_Follow'),
 ('single_garbage_identify','single_garbage_identify'),
 ('garbage_identify','garbage_identify'),
 ('color_identify','color_identify'),
 ('color_stacking','color_stacking'),
 ('color_follow_ctrl','color_follow'),
 ('Calibration','update_hsv'),
 ('Calibration','Arm_Calibration'),
]
fail=0
for mod, attr in tests:
    try:
        m=__import__(mod, fromlist=[attr])
        getattr(m, attr)
        print(f'[OK] from {mod} import {attr}')
    except Exception as e:
        fail+=1
        print(f'[FAIL] from {mod} import {attr}: {type(e).__name__}: {e}')
raise SystemExit(fail)
" || echo "[WARN] import mở rộng có fail — xem chi tiết trên (thường do thiếu dofbot_ws source hoặc model YOLO)."

if [ $IMPORT_RC -ne 0 ]; then
  echo "[ABORT] Dừng trước khi chạy main app vì 10 module bắt buộc còn fail."
  exit 1
fi

# 8) Chạy main app — phải cd vào rosmaster vì YahboomArm dùng './config.ini' + templates/
echo "===== Start YahboomArm.pyc ====="
echo "[INFO] Flask 0.0.0.0:6500 + TCP socket 6000 (lấy từ disassembly: start_tcp_server(ip,6000), app.run(host=0.0.0.0,port=6500))"
if [ ! -f "$PYC" ]; then echo "[FAIL] thiếu $PYC"; exit 1; fi
cd "$PROJECT/rosmaster" || exit 1
echo "[INFO] cwd=$(pwd), config.ini: $(ls -l config.ini 2>&1 | head -n 1)"

# Chạy app nền để health-check port, sau đó foreground (giữ process chạy)
"$PYBIN" "$PYC" &
APP_PID=$!
echo "[INFO] YahboomArm PID=$APP_PID, chờ 12s để Flask+TCP bind..."
sleep 12
if ! kill -0 $APP_PID 2>/dev/null; then
  echo "[FAIL] process đã thoát sớm. Đọc log trên để xem ModuleNotFoundError tiếp theo."
  wait $APP_PID
  exit $?
fi
echo "[OK] process còn chạy (PID $APP_PID)"

# Kiểm tra port 6000/6500
echo "===== Port check ====="
(ss -tlnp 2>/dev/null | grep -E ':6000|:6500' && echo "[OK] thấy listen 6000/6500") || echo "[WARN] chưa thấy listen 6000/6500 (app có thể cần thêm thời gian hoặc bind IP LAN khác)"
echo "--- ss listen ---"
ss -tlnp 2>/dev/null | head -n 20 || netstat -tlnp 2>/dev/null | head -n 20 || true
echo "--- IP LAN hiện tại ---"
hostname -I 2>/dev/null || ip route get 1.1.1.1 2>/dev/null | head -n 5 || ifconfig 2>/dev/null | head -n 20 || true
echo "[INFO] Nếu server chạy: mở http://<IP-LAN>:6500  (Flask video_feed) + TCP 6000 cho app điều khiển"
echo "[INFO] Giữ process chạy foreground. Ctrl+C để dừng."
wait $APP_PID
