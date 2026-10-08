#!/usr/bin/env python3
"""Hệ tọa độ của DOFBOT + các camera, khai báo một chỗ (dùng `cube_vision.frames.FrameGraph`).

    world ≡ base_link ─ arm1 ─ arm2 ─ arm3 ─ arm4 ─ arm5 ─ tcp
                │                          └─ wrist_cam ─ wrist_optical        (camera tay, gắn trên arm4)
                ├─ table
                ├─ ext_optical                                                  (webcam laptop, cố định)
                └─ phone_optical                                                (iPhone, cố định)

Góc khớp `q` là 5 góc SERVO tính bằng độ (0–180, 90 = thẳng), đúng như đọc từ tay máy.
Số được lấy từ nguồn đang dùng thật, không chép lại: động học từ `cube_search_center_math` (đã khớp URDF),
hand-eye từ `config/robot/hand_eye.json`, camera ngoài từ `config/robot/external_camera.json`.

    python projects/vision_experiments/dofbot_frames.py --dump        # bảng hệ tọa độ + ma trận hiện tại (markdown)
    python projects/vision_experiments/dofbot_frames.py --chain world wrist_optical
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "projects", ROOT / "projects" / "t8_pipeline", Path(__file__).resolve().parent):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import cube_search_center_math as M  # noqa: E402
import dofbot_ik  # noqa: E402
from cube_vision import frames as F, intrinsics as I  # noqa: E402
from cube_vision.external_camera import ExternalCalibration  # noqa: E402

READY = [90.0, 125.0, 0.0, 0.0, 90.0]
# Quy ước, không hiệu chuẩn: hệ thân camera (wrist_cam, trục song song arm4) -> hệ quang học của ẢNH THÔ.
WRIST_CAM_T_OPTICAL = M.MOUNT_RZ90
# URDF Gripping_Joint (arm5 -> Gripping_point_Link): xyz + rpy(3.1416, -1.5708, 0).
ARM5_T_TCP = M._trans(dofbot_ik.GRIP_O, (3.1416, -1.5708, 0.0))
JOINTS = (("base_link", "arm1", M.J1_O, (0, 0, 1)), ("arm1", "arm2", M.J2_O, (0, 1, 0)),
          ("arm2", "arm3", M.J3_O, (0, 1, 0)), ("arm3", "arm4", M.J4_O, (0, 1, 0)),
          ("arm4", "arm5", M.J5_O, (0, 0, 1)))


def _joint(index, origin, axis, cal):
    def parent_T_child(q):
        servo = float(q[index])
        if index == 0 and cal:
            servo = M.true_j1(servo, float(cal.get("j1_scale", 1.0)), float(cal.get("j1_offset_deg", 0.0)))
        return M._trans(origin) @ M._rot_axis(axis, M.servo_to_q(servo))
    return parent_T_child


def load_phone(path=None):
    """File camera của iPhone khi VỊ TRÍ đã hiệu chuẩn đạt (`pose_accepted`), ngược lại None."""
    data = I.load_camera("phone", path)
    try:
        ok = bool(data) and data.get("pose_accepted") is True and F.is_rigid(
            np.asarray(data["base_T_optical"], float).reshape(4, 4))
    except (KeyError, TypeError, ValueError):
        ok = False
    return data if ok else None


def build(cal="auto", external="auto", phone="auto") -> F.FrameGraph:
    """Đồ thị hệ tọa độ. Mặc định đọc các file hiệu chuẩn; truyền None để coi như chưa hiệu chuẩn."""
    cal = M.load_calibration() if cal == "auto" else cal
    external = ExternalCalibration.load() if external == "auto" else external
    phone = load_phone() if phone == "auto" else phone
    g = F.FrameGraph()
    for name, note in (
        ("world", "Hệ gốc chung của mọi camera và mọi tọa độ báo ra. Chọn trùng base_link."),
        ("base_link", "Đế tay máy, gắn cố định với bàn. Gốc ở tâm đế, z lên; vùng làm việc ở phía −x."),
        ("arm1", "Sau khớp J1 (xoay quanh z)."), ("arm2", "Sau khớp J2 (vai, quanh y)."),
        ("arm3", "Sau khớp J3 (khuỷu, quanh y)."),
        ("arm4", "Sau khớp J4 (cổ tay gập, quanh y). CAMERA TAY GẮN Ở ĐÂY, không phải ở arm5."),
        ("arm5", "Sau khớp J5 (xoay kẹp quanh z). Mặt bích gắn kẹp; J5 KHÔNG làm xoay camera."),
        ("tcp", "Điểm kẹp (URDF Gripping_point_Link): mục tiêu của IK gắp/thả."),
        ("wrist_cam", "Thân camera tay, trục song song arm4."),
        ("wrist_optical", "Hệ quang học camera tay (ảnh 640×480): z ra trước, x phải, y xuống."),
        ("table", "Mặt bàn: cùng hướng base_link, gốc hạ xuống mặt bàn (z = 0 là mặt bàn)."),
        ("ext_optical", "Hệ quang học webcam laptop (1280×720), đứng yên ngoài tay máy."),
        ("phone_optical", "Hệ quang học iPhone qua DroidCam (1280×720 sau khi xoay ảnh)."),
    ):
        g.frame(name, note)
    g.add("world", "base_link", np.eye(4), kind="quy ước", source="dofbot_frames.build",
          note="Đơn vị: world trùng base. Đổi ở đây nếu sau này world là phòng/bản đồ lớn hơn.")
    for index, (parent, child, origin, axis) in enumerate(JOINTS):
        g.add(parent, child, fn=_joint(index, origin, axis, cal), kind="FK",
              source=f"cube_search_center_math.J{index + 1}_O (URDF arm{index + 1}_Joint)",
              note=f"servo J{index + 1}; q = (servo − 90)°")
    g.add("arm5", "tcp", ARM5_T_TCP, kind="CAD", source="dofbot_ik.GRIP_O (URDF Gripping_Joint)")
    mount = None if cal is None else np.asarray(cal["arm4_T_optical"], float) @ F.invert(WRIST_CAM_T_OPTICAL)
    g.add("arm4", "wrist_cam", mount, kind="hiệu chuẩn hand-eye",
          source="config/robot/hand_eye.json: arm4_T_optical · inv(wrist_cam_T_wrist_optical)",
          how="python projects/vision_experiments/calibrate_hand_eye.py",
          note="Chỉ tin trong J1 35–140°. URDF ghi camera ở −x của arm4, đo thật là +x.")
    g.add("wrist_cam", "wrist_optical", WRIST_CAM_T_OPTICAL, kind="quy ước",
          source="cube_search_center_math.MOUNT_RZ90")
    table_z = None
    try:
        table_z = float(json.loads(M.CALIB_FILE.read_text())["table_z"]) if cal is not None else None
    except (OSError, ValueError, KeyError, TypeError):
        table_z = None
    g.add("base_link", "table", None if table_z is None else M._trans((0.0, 0.0, table_z)),
          kind="hiệu chuẩn hand-eye", source="config/robot/hand_eye.json: table_z (= tag_top_z − 0,030)",
          how="python projects/vision_experiments/calibrate_hand_eye.py")
    g.add("base_link", "ext_optical", None if external is None else external.base_T_ext,
          kind="hiệu chuẩn camera ngoài", source="config/robot/external_camera.json: base_T_ext",
          how="python projects/vision_experiments/calibrate_external.py --collect (×6) rồi --solve",
          note="Webcam dễ bị chạm: kiểm --status trước khi dùng, lệch thì --relocalize.")
    g.add("base_link", "phone_optical",
          None if phone is None else np.asarray(phone["base_T_optical"], float).reshape(4, 4),
          kind="hiệu chuẩn camera ngoài", source="config/robot/cameras/phone.json: base_T_optical",
          how="python projects/vision_experiments/calibrate_external.py --camera phone")
    return g


def cameras(cal="auto", external="auto", phone="auto") -> dict:
    """{tên hệ optical: CameraModel} cho các camera đã có thông số nội."""
    cal = M.load_calibration() if cal == "auto" else cal
    external = ExternalCalibration.load() if external == "auto" else external
    phone = load_phone() if phone == "auto" else phone
    out = {}
    if cal is not None:
        out["wrist_optical"] = F.CameraModel("wrist", tuple(cal["K"]), M.IMAGE_SIZE, float(cal.get("k1", 0.0)),
                                             source="config/robot/hand_eye.json: K, k1 (fit chung với hand-eye)")
    if external is not None:
        f, cx, cy = external.K
        out["ext_optical"] = F.CameraModel("ext", (f, f, cx, cy), tuple(external.image_size), external.k1,
                                           source="config/robot/external_camera.json: K, k1 (fit chung với pose)")
    model = I.model_from(I.load_camera("phone") if phone is not None else None, "phone")
    if model is not None:
        out["phone_optical"] = model
    return out


def world_T_optical(graph, optical: str, q=None) -> np.ndarray:
    """Pose của một camera trong world (camera tay cần q)."""
    return graph.lookup("world", optical, q)


# ------------------------------------------------------------------ in bảng
def _fmt(T) -> str:
    T = np.asarray(T, float)
    r, p, y = F.rpy_deg(T)
    return (f"xyz = ({T[0, 3] * 1000:+.1f}, {T[1, 3] * 1000:+.1f}, {T[2, 3] * 1000:+.1f}) mm; "
            f"rpy = ({r:+.1f}, {p:+.1f}, {y:+.1f})°")


def dump(graph=None, q=READY) -> str:
    g = graph or build()
    lines = ["| Hệ tọa độ | Ý nghĩa |", "|---|---|"]
    lines += [f"| `{name}` | {g.notes[name]} |" for name in g.frames()]
    lines += ["", "| Phép biến đổi | Loại | Trạng thái | Nguồn số | Giá trị (khớp ở READY nếu theo khớp) |",
              "|---|---|---|---|---|"]
    for e in g.edges:
        try:
            value = _fmt(e.value(q))
        except F.MissingTransform:
            value = f"chưa có — `{e.how}`"
        lines.append(f"| `{e.name}` | {e.kind} | {e.state} | {e.source} | {value} |")
    lines += ["", f"Chuỗi tới từng camera (q = {q}):", ""]
    for optical in ("wrist_optical", "ext_optical", "phone_optical"):
        text = f"- `world_T_{optical}` = {g.describe('world', optical)}"
        lines.append(text + (f"  \n  → {_fmt(g.lookup('world', optical, q))}" if g.available("world", optical)
                             else "  \n  → chưa hiệu chuẩn"))
    lines += ["", "| Camera | Cỡ ảnh | fx, fy | cx, cy | k1, k2 | Xoay luồng | Nguồn |", "|---|---|---|---|---|---|---|"]
    for optical, cam in cameras().items():
        fx, fy, cx, cy = cam.K
        lines.append(f"| `{optical}` | {cam.image_size[0]}×{cam.image_size[1]} | {fx:.1f}, {fy:.1f} | "
                     f"{cx:.1f}, {cy:.1f} | {cam.k1:+.3f}, {cam.k2:+.3f} | {cam.rotate}° | {cam.source} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", action="store_true", help="in bảng hệ tọa độ và ma trận hiện tại (markdown)")
    ap.add_argument("--chain", nargs=2, metavar=("A", "B"), help="in chuỗi và ma trận A_T_B")
    ap.add_argument("--q", nargs=5, type=float, default=READY, help="5 góc servo (độ), mặc định READY")
    args = ap.parse_args()
    g = build()
    if args.chain:
        a, b = args.chain
        print(f"{a}_T_{b} = {g.describe(a, b)}")
        with np.printoptions(precision=5, suppress=True):
            print(g.lookup(a, b, args.q))
    else:
        print(dump(g, args.q))


if __name__ == "__main__":
    main()
