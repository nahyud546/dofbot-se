# Runbook: Thao tac co ban tay that (chua choi co)

> Moi lenh deu `cd` vao workspace truoc. Copy-paste nguyen khoi la chay.
> Rule: SIM (RViz) -> GATE -> probe STM32 -> gui HW cham + monitor.

## 0. Smoke-test (khong can robot, khong can ROS)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
python3 hardware/safety_gate.py --self-test
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --dry-run
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/gripper_test.json --dry-run
```
- PASS: `ALL PASS` + 2 dong `DRY-RUN PASS`.
- `can't open file ... hardware/...`: ban quen `cd` (xem bang loi muc D).

## 1. Mo phong tren RViz (plan + nhin, chua gui gi ra tay that)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dofbot_moveit demo.launch.py use_rviz:=true
```
Trong RViz panel MotionPlanning: keo tay -> **Plan** -> xem duong di co suot ban/khung khong.
Toc do de mac dinh 0.1. Xong tat RViz (Ctrl-C o terminal launch).

## 2. Probe STM32 (moi buoi chay, read-only, khong di chuyen)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
python3 hardware/check_stm32.py
# neu khong thay /dev/myserial, tim port that:
dmesg | tail
ls /dev | grep -E 'USB|serial|ACM'
python3 hardware/check_stm32.py --port /dev/ttyUSB0
```
- PASS: `bus OK — 6/6 servo phan hoi`, so rad khop tu the tay ban dang thay.
- Xac nhan port dung truoc khi sang buoc 3 (ghi nho so ttyUSB).

## 3. Test servo dau tien: ve HOME cham (tay trong, tay ban tren cong tac nguon)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
python3 hardware/check_stm32.py --port /dev/ttyUSB0 --no-read-only --park --time-ms 2000
```
- Neu bo `--port` thi dung `/dev/myserial` (may Yahboom). Thay port dung tu buoc 2.
- Quan sat tay di chuyen cham ve HOME roi moi lam tiep.

## 4. Gui motion co ban ra tay that (da xem RViz buoc 1 + da probe buoc 2)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
# kho truoc (khong cham HW) — luon chay:
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --dry-run
# gui that — motion vung trong nen kem --allow-uncalibrated + --yes:
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --yes --allow-uncalibrated
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/gripper_test.json --yes --allow-uncalibrated --time-ms 2000
```
- Bridge tu: doc servo that -> gate (limit, scaling <=0.25, jump <=0.6rad, start khop 0.05rad, state tuoi) -> gui tung waypoint cham -> doc readback sau moi diem, lech >0.15rad la DUNG.
- Thu tu leo thang: `home_hold` -> `basic_wiggle` -> `gripper_test` -> moi toi co.
- `--allow-uncalibrated` CHI cho motion vung trong. Gap/dat gan ban: CAM, phai calibrate TCP truoc (xem `hardware/RUNBOOK.md` muc 0).

## A. Khoi dong nhanh moi ngay (da biet port, da xem RViz)

```bash
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
python3 hardware/check_stm32.py --port /dev/ttyUSB0
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --yes --allow-uncalibrated
```

## B. Khi co su co: E-stop

- `Ctrl-C`: bridge tu halt (giu vi tri). Tay rung/ket -> **CAT NGUON**, khong keo tay khi con dien.
- Tracking error / timeout / servo miss -> DUNG, kiem tra ket co khi, nguon yeu, `time_ms` qua ngan, roi chay lai tu buoc 1 (plan lai, khong execute tiep traj cu).
- Cam forward `/joint_states` cua FakeSystem ra servo.

## C. Thong so an toan hien tai (`hardware/safety_config.yaml`)

- Servo: s1-s4,s6 0-180 do, s5 0-270 do; grip 30=mo, 180=dong.
- Rad 0 <-> servo 90. Tran toc do HW 0.25 (chay 0.1). `time_ms` commissioning 2000, min 500.
- `tcp_offset_calibrated: false` (chan gui HW gap/dat; motion co ban vung trong di `--allow-uncalibrated`).

## E. Soi tay thật trong RViz song song (mirror một chiều, read-only)

> Nguyên tắc: RViz KHÔNG BAO GIỜ lái tay thật. Lệnh duy nhất ra servo vẫn là
> `arm_hw_bridge.py --yes` (qua gate + `--yes`). Mirror chỉ đọc + vẽ.
> Serial chỉ 1 chủ tại 1 thời điểm: `real_joint_mirror.py` rời và bridge `--yes`
> không chạy cùng lúc (trong lúc bridge gửi, dùng `--mirror-topic` của bridge).

```bash
# T1 — RViz sim (plan + nhìn như mục 1, giữ mở suốt buổi):
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dofbot_moveit demo.launch.py use_rviz:=true

# T2 — bóng tay thật (đọc servo -> /real_joint_states -> TF real_*):
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dofbot_moveit real_mirror.launch.py
python3 hardware/real_joint_mirror.py --port /dev/ttyUSB0
# RViz -> Add -> RobotModel -> Description Topic: /real_mirror/robot_description,
#   TF Prefix: real_  => thấy bóng tay thật cạnh tay sim, lệch là thấy ngay.

# T3 — gửi HW như mục 4, TẮT mirror T2 trước (Ctrl-C), bật mirror trong bridge:
cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --dry-run
python3 hardware/arm_hw_bridge.py --traj ~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/basic_wiggle.json --yes --allow-uncalibrated --mirror-topic /real_joint_states
# Xong: Ctrl-C bridge, bật lại mirror T2 để soi tư thế nghỉ.
```
- Tốc độ mirror thực tế ~2Hz (giới hạn bởi 6 lần đọc serial STM32), đủ để soi.
- `s6=29` đọc về khi gripper mở hết là sai số servo (limit 30): mirror chỉ warn,
  gate trong bridge vẫn chặn HW như cũ.
- Cấm forward `/joint_states` của FakeSystem ra servo (kiểu `dofbot_driver` gốc).

## F. Xử lý sự cố nhanh

| Hien tuong | Nguyen nhan | Cach sua |
|---|---|---|
| `can't open file '.../hardware/safety_gate.py'` | Dang dung o `~/Desktop/robot-arm`, file nam trong `dofbot_robot_arm_6dof/hardware/` | Chay `cd ~/Desktop/robot-arm/dofbot_robot_arm_6dof` truoc moi lenh |
| `No such file ... examples/...` | Dung `--traj` tuong doi sai thu muc | Dung duong dan day du `~/Desktop/robot-arm/dofbot_robot_arm_6dof/hardware/examples/...` nhu muc 0/4 |
| `Khong thay serial STM32` | Chua cam cap / sai port / thieu quyen | `dmesg \| tail`, thu `--port /dev/ttyUSB0`, them nhom `dialout` (`sudo usermod -aG dialout $USER`, logout/login) |
| `tcp_offset_calibrated=false — chan gui HW` | Chua calibrate TCP (dung cho gap/dat) | Motion co ban: them `--allow-uncalibrated --yes`. Gap/dat: do OFFSET that roi bat `true` |
| `thieu --yes` | Chua xac nhan operator | Xem lai RViz, tay tren E-stop, them `--yes` |
| Gate `joint jump / vel` FAIL (vd. gripper_test cu) | Waypoint nhay qua lon/qua nhanh | Chia nho buoc (moi buoc <=0.4rad, cach nhau >=2s o scale 0.1) |
| `Thieu Arm_Lib` | Chay sai venv | Dung venv ROS py3.10 / image Yahboom (da co Arm_Lib 0.0.5) |
| `start mismatch` | Tay that dang o tu the khac point 0 | Dua tay ve gan HOME truoc (`--park`), hoac sua point 0 theo readback buoc 2 |
