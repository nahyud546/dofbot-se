#!/usr/bin/env python3
# coding: utf-8
import math
import rclpy
from rclpy.node import Node
import Arm_Lib
import cv2 as cv
from math import pi
from time import sleep
from collections import deque
from statistics import median
from stacking_grap import stacking_grap
from dofbot_interface.srv import Kinemarics

# --- Pixel -> robot calibration (nguon goc Yahboom, khong phai mo hinh camera) ---
# Goc: dofbot_sorting/dofbot_sorting/color_sorting.py:120,
#      objection_grasp.py:75, get_offset.py:69:
#        a = (320 - cx)/4000 + y_offset        (lateral, m)
#        b = ((480 - cy)/3000)*0.8 + 0.15 + x_offset   (forward, m)
#        pos.x = b, pos.y = a
# Y nghia: 1/4000 = 0.25 mm/px ngang; 0.8/3000 ~= 0.267 mm/px doc (0.8 bu
# perspective do camera nhin xien); 0.15 m = khoang cach tu goc base toi
# mep duoi anh o pose calib chuan. Chi dung tai init_joints luc calib
# (sorting: [90,120~130,0,0,90,0]) + do cao ban + brightness co dinh.
# Offset duoc calib bang AprilTag tai world (0.20, 0.0): ox = 0.20 - b,
# oy = 0.0 - a (xem get_offset.py:71-78, offset.yaml hien tai -0.0187).
# Ban stacking hien tai dung KDL moi (ban truoc la -X, da verify FK
# grasp-down [90,35,65,15,90] -> X=-0.2069): tar_x = -(b), tar_y = a.
# Bien the goc colcon_ws/.../color_stacking.py:153-158 dung 0.17 thay 0.15.
PIXEL_TO_M_Y = 1.0 / 4000.0
PIXEL_TO_M_X = 0.8 / 3000.0
X_BASE_M = 0.15
# --- Bu meo ong kinh xuyen tam (barrel) ---
# Camera USB re tien nen rìa: diem hien thi gan tam hon thuc te (nen)
# -> |lat|/|fwd| do duoc NHO hon that -> kep lech ve phia tam anh. Tam anh
# (px~320,py~240) khong anh huong; cang ra ria bu cang nhieu nen mac dinh
# nay KHONG doi hanh vi o giua ban (an toan).
# Mo hinh 1 tham so Brown: u = c + (p-c)*(1 + K*rn2),
# rn2 = ((px-320)/320)^2 + ((py-240)/240)^2 (0 tam -> 2 goc).
# K>0 mo rong ria (barrel, pho bien); K<0 thu hep (pincushion, hiem).
# Mac dinh 0.06 (THAN TRONG, ha tu 0.12 ngay 2026-09-24). Ly do: log cho thay
# muc tieu xa-tren (red py~95, undist 6% ~ +15mm day ra xa) van lech RA XA
# robot -> nghi K=0.12 bu qua tay o vung xa. Ha mot nua, giu huong barrel.
# Neu van lech xa: thu K=0.0 (tuyen tinh thuan) va bao lai. Neu chuyen sang
# lech thieu (tay dap truoc cube, ve phia robot): tang dan 0.02/buoc.
# Chan ly van la hieu chuan bang thuoc (1 lan):
#  1. Dat 1 cube THANG tai giua anh (px0~=320, cung hang py), ghi lat0.
#  2. Doi cube do sang px1~=550 (giu nguyen hang py), ghi lat1.
#  3. Do khoang cach THAT D (met) cube dich chuyen bang thuoc.
#  4. K = ((D / |lat1-lat0|) - 1) / rn2, voi rn2 tinh tai (px1,py).
#  VD: lat1-lat0=0.055, D=0.060, rn2=0.55 -> K=((0.060/0.055)-1)/0.55~=0.165.
#  Neu K am (pincushion) thi giu nguyen dau am do duoc.
K_DISTORTION = 0.06
Y_OFFSET_M = 0.0
# Do cao TCP luc gắp (m). Cube 30 mm; de 0.039 (an toan, tranh quet ban).
# Neu kep chi trung mep tren (rim) ma X da giua cube: ha 0.002/buoc (min
# 0.033). Neu cham ban/IK fail nhieu: de yen, co che retry z lo.
PICK_Z_M = 0.069 - 0.03
# Pitch IK (rad) = goc cua TCP so voi mat ban. 1.04 ~= 60 do (nghien, dung de
# bo thung sorting); 1.396 ~= 80 do; 1.57 ~= 90 do (chuc thang xuong).
# Voi cube stacking can kep dinh -> thu 1.35~1.45 neu van di qua.
# Goc goc largemodel_arm/grasp_desktop.py:133 co ghi chu y nghia nay.
PICK_PITCH = 1.04
# --- J5 bam theo goc xoay cube ---
# Truoc day J5 bi ghi de cung (90, roi J5=J1): cang luon thang theo BAN.
# Ban co tinh xoay cube -> cang kep goc -> hut/xoay cube -> thap do.
# Giai phap: do huong canh dai cua contour (minAreaRect -> boxPoints, khong
# dung w/h vi quy uoc OpenCV doi theo version), doi ra yaw world roi cho J5
# xoay theo: J5_pick = J1 - SIGN*delta.
# Chieu dau: anh +px = world +Y, anh +py(down) = world +X (xem mapping tren)
# => yaw_world_edge = 90 - alpha_img => cube xoay delta_world thi anh xoay
# -delta => delta = axis_gan_nhat - alpha (wrap ve [-45,45]).
# Do bang FK full-chain: yaw_mo_cang = J1 + 90 - J5, muon mo theo phap tuyen
# mat cube (90 + delta) => J5 = J1 - delta. SIGN=1.0 theo dao ham tren; neu
# test thay cang xoay NGUOC huong cube thi doi thanh -1.0 (1 hang, 1 test).
# Nhieu so lieu: mask gom mat tren + mat ben (chieu xien) nen goc lech vai do;
# clamp +-40 + median 5 frame nhu tam. Luc DAT giu J5=J1 (thang theo ban) de
# nan cube thang lai -> thap ngay.
CUBE_YAW_SIGN = 1.0
CUBE_YAW_MAX_DEG = 40.0

if not rclpy.ok():
    rclpy.init()

class stacking_GetTarget(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('dofbot_stacking')
        self.image = None
        self.detection_image = None
        self.color_name = None
        self.color_status = True
        self.center_history = {}
        self.angle_history = {}
        # 机械臂初始位置
        self.xy = [90, 135]
        self.arm = Arm_Lib.Arm_Device("/dev/ttyUSB0")
        self.grap = stacking_grap()
        
        # 创建ROS2服务客户端
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        
        # 等待服务端启动
        self.get_logger().info("Waiting for dofbot_kinemarics service...")
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service not available, waiting again...')
        self.get_logger().info("Service is available!")

    def target_run(self, msg, xy=None, color_order=None, completed=None):
        '''
        抓取函数 
        :param msg: [颜色,位置] [color, location]
        '''
        if xy is not None:
            self.xy = xy
        if completed is None:
            completed = []
        if color_order is None:
            color_order = list(msg.keys())
        joints_home = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        for name in color_order:
            if name in completed:
                continue
            layer = len(completed) + 1
            entry = msg.get(name)
            if entry is None:
                self.get_logger().error(
                    f"{name} not detected; stopped before layer {layer}. "
                    "Press SPACE to retry after it is visible."
                )
                return
            # msg value la dict {"pos","yaw"} (bam goc xoay); giu tuong thich
            # tuple cu neu gap.
            if isinstance(entry, dict):
                pos, cube_yaw = entry.get("pos"), float(entry.get("yaw", 0.0))
            else:
                pos, cube_yaw = entry, 0.0

            # Solve and execute one cube at a time. A later cube with invalid
            # IK must not prevent earlier valid layers from being placed.
            joints = self.server_joint(pos)
            if joints is None:
                self.get_logger().error(
                    f"IK invalid for {name} (layer {layer}); stopped at ready pose. "
                    "Adjust that cube and press SPACE to retry."
                )
                return
            # J5 bam theo goc xoay cube: mo cang thang phap tuyen mat cube.
            # Dat thi arm_run tu dung J5=J1 (thang ban) de nan cube thang.
            j5_pick = min(270.0, max(0.0, joints[0] - CUBE_YAW_SIGN * cube_yaw))
            joints[4] = j5_pick
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
            self.get_logger().info(
                f"Layer {layer} ({name}) IK joints: {joints} "
                f"(cube_yaw={cube_yaw:+.1f}deg -> J5={j5_pick:.1f})")
            self.grap.arm_run(str(layer), joints, joints_home)
            completed.append(name)
            self.get_logger().info(
                f"Layer {layer} ({name}) placed; arm is back at ready pose."
            )
            # Let the arm settle at ready before solving the next cube.
            sleep(1.0)

    def select_color(self, image, color_hsv, color_list):
        '''
        选择识别颜色
        :param image:输入图像 input image
        :param color_list: 颜色序列:['0'：无 '1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]
        :return: 输出处理后的图像,(颜色,位置)
        '''
        # 规范输入图像大小
        self.image = cv.resize(image, (640, 480))
        # Keep masks independent of the annotations drawn for earlier colors.
        self.detection_image = self.image.copy()
        msg = {}
        if len(color_list) == 0:
            return self.image, msg
        # Moi entry: {"pos": (tar_x, tar_y), "yaw": delta_world_deg} de J5
        # bam theo goc xoay cube luc gap.
        if '1' in color_list:
            self.color_name = color_list['1']
            det = self.get_Sqaure(color_hsv[self.color_name])
            if det is not None:
                msg[self.color_name] = {"pos": det[0], "yaw": det[1]}
        if '2' in color_list:
            self.color_name = color_list['2']
            det = self.get_Sqaure(color_hsv[self.color_name])
            if det is not None:
                msg[self.color_name] = {"pos": det[0], "yaw": det[1]}
        if '3' in color_list:
            self.color_name = color_list['3']
            det = self.get_Sqaure(color_hsv[self.color_name])
            if det is not None:
                msg[self.color_name] = {"pos": det[0], "yaw": det[1]}
        if '4' in color_list:
            self.color_name = color_list['4']
            det = self.get_Sqaure(color_hsv[self.color_name])
            if det is not None:
                msg[self.color_name] = {"pos": det[0], "yaw": det[1]}
        return self.image, msg

    def get_Sqaure(self, hsv_lu):
        '''
        颜色识别
        :param hsv_lu:(lowerb, upperb)
        :return: ((tar_x, tar_y), cube_yaw_deg) - vi tri + goc xoay cube
                 (world, CCW+, wrap [-45,45]) de J5 bam theo luc gap
        '''
        (lowerb, upperb) = hsv_lu
        # Threshold directly in HSV, then use the largest clean color region.
        HSV_img = cv.cvtColor(self.detection_image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(HSV_img, lowerb, upperb)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, kernel)
        binary = cv.morphologyEx(binary, cv.MORPH_OPEN, kernel)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = find_contours[1] if len(find_contours) == 3 else find_contours[0]

        candidates = [(cv.contourArea(cnt), cnt) for cnt in contours
                      if cv.contourArea(cnt) > 1000]
        if not candidates:
            return None
        _, cnt = max(candidates, key=lambda item: item[0])
        moments = cv.moments(cnt)
        if moments["m00"] <= 0:
            return None

        # Polygon centroid is less biased by perspective than the center of
        # its axis-aligned bounding rectangle. Median of recent frames damps
        # pixel jitter while a cube is stationary.
        center_x = moments["m10"] / moments["m00"]
        center_y = moments["m01"] / moments["m00"]
        history = self.center_history.setdefault(self.color_name, deque(maxlen=5))
        history.append((center_x, center_y))
        point_x = median(p[0] for p in history)
        point_y = median(p[1] for p in history)

        # Goc xoay cube: lay canh DAI nhat tu boxPoints (khong dung w/h vi
        # quy uoc OpenCV doi theo version). alpha = huong canh trong anh
        # (px-right, py-down), [0,180). Mask gom mat tren+ben nen co nhieu
        # vai do -> clamp +-40 + median 5 frame.
        cube_yaw = 0.0
        try:
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)
            best_len, alpha = -1.0, 0.0
            for i in range(4):
                dx = float(box[(i + 1) % 4][0] - box[i][0])
                dy = float(box[(i + 1) % 4][1] - box[i][1])
                ln = dx * dx + dy * dy
                if ln > best_len:
                    best_len, alpha = ln, math.degrees(math.atan2(dy, dx)) % 180.0
            r = alpha % 90.0
            delta = -r if r <= 45.0 else 90.0 - r
            delta = max(-CUBE_YAW_MAX_DEG, min(CUBE_YAW_MAX_DEG, delta))
            ahist = self.angle_history.setdefault(self.color_name, deque(maxlen=5))
            ahist.append(delta)
            cube_yaw = median(ahist)
        except Exception:
            pass
        # Yaw kich clamp (+-40) = do suy bien khi KEM THEO 1 trong 2 dau hieu:
        # (a) nhay loan trong history (spread>5, vd doi dau -40/+40 giua 2
        # frame lien tiep - cube that khong the xoay 80 do trong 30ms), hoac
        # (b) cube o RIA ANH (px<120 hoac >520, cung nguong voi WARN vi tri):
        # phoi canh gop mat tren+ben -> minAreaRect cho goc rac ON DINH
        # (vd green px~531: dinh -40.0 hang tram frame, spread~0) - truong hop
        # nay spread khong bat duoc nen phai gate theo vi tri. Log 2026-09-24:
        # thieu gate (b) nen fallback chi giu duoc ~5 frame roi tin lai -40 ->
        # J5=114.3 -> gap hut. Cube nghieng THAT dat GIUA anh (px 200-440)
        # ke ca ~+-40 on dinh -> van tin, J5 bam theo. Fallback yaw=0+WARN.
        aspread = (max(ahist) - min(ahist)) if len(ahist) else 0.0
        at_edge = (point_x < 120.0 or point_x > 520.0)
        yaw_reliable = not (abs(cube_yaw) >= (CUBE_YAW_MAX_DEG - 0.5)
                            and (aspread > 5.0 or at_edge))
        if not yaw_reliable:
            warned = getattr(self, "_yaw_warn", {}).get(self.color_name, False)
            if not warned:
                self.get_logger().warning(
                    f"[{self.color_name}] yaw kich clamp "
                    f"(spread={aspread:.1f}deg edge={at_edge}) -> fallback "
                    f"yaw=0 (J5=J1). Muon test goc nghieng that thi dat cube "
                    f"giua anh (px 200-440)."
                )
                if not hasattr(self, "_yaw_warn"):
                    self._yaw_warn = {}
                self._yaw_warn[self.color_name] = True
            cube_yaw = 0.0
        elif hasattr(self, "_yaw_warn"):
            self._yaw_warn[self.color_name] = False

        x, y, w, h = cv.boundingRect(cnt)
        cv.drawContours(self.image, [cnt], -1, (0, 255, 0), 2)
        cv.rectangle(self.image, (x, y), (x + w, y + h), (255, 255, 0), 1)
        cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
        cv.putText(self.image, f"{self.color_name} {cube_yaw:+.0f}d",
                   (int(x - 15), int(y - 15)),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        # Fit tuyen tinh goc Yahboom (xem block calibration o dau file).
        # Chu y: cong thuc nay fit cho anh THO 640x480 tai pose quan sat luc
        # calib. Neu anh da qua Perspective_transform thi point_x/point_y da
        # bi keo di -> a,b sai he thong theo X. Vi vay color_stacking_play.py
        # phai detect tren frame raw (khong warp), warp chi de hien thi.
        # Giai nen xuyen tam truoc khi map tuyen tinh (tam anh f=1 -> giu
        # nguyen; ria anh f>1 -> mo rong ve dung vi tri that). Box neo giu
        # nguyen pixel tho (bu vi sai tu triet tieu).
        rn2 = ((point_x - 320.0) / 320.0) ** 2 + ((point_y - 240.0) / 240.0) ** 2
        _uf = 1.0 + K_DISTORTION * rn2
        ux = 320.0 + (point_x - 320.0) * _uf
        uy = 240.0 + (point_y - 240.0) * _uf
        a_fwd = round((480 - uy) * PIXEL_TO_M_X + X_BASE_M, 5)
        b_lat = round((ux - 320) * PIXEL_TO_M_Y + Y_OFFSET_M, 8)
        self.get_logger().info(
            f"[{self.color_name}] px=({point_x:.1f},{point_y:.1f}) "
            f"undist={(_uf - 1.0) * 100:.1f}% "
            f"-> fwd={a_fwd} lat={b_lat} -> tar=({-a_fwd},{b_lat}) "
            f"yaw={cube_yaw:+.1f}deg"
            f"{'' if yaw_reliable else ' FALLBACK(J5=J1)'}"
        )
        return ((-a_fwd, b_lat), cube_yaw)

    def server_joint(self, posxy):
        '''
        发布位置请求
        :param posxy: 位置点x,y坐标
        :return: 每个关节旋转角度
        '''
        # 检查参数合法性
        if len(posxy) < 2:
            self.get_logger().error("posxy must have at least x and y coordinates!")
            return None
        
        # 创建ROS2服务请求. Thử z tăng dần: target rìa (vd green
        # x=-0.188 y=-0.058) FAIL ở z=0.039 nhưng OK ở z=0.050/0.060 (đo
        # 2026-09-24). Nâng z khi IK fail để không kẹt cả sequence.
        # The pixel mapping already produces the IK target. An extra positive
        # X offset would pull this negative-X target back toward the robot.
        base_x = posxy[0]
        base_y = posxy[1]
        if abs(base_y) > 0.05:
            self.get_logger().warn(
                f"Target y={base_y:.4f} ở rìa ảnh (|y|>0.05, px<120 hoặc >520): "
                "công thức tuyến tính ngoại suy kém + dễ vào lỗ IK. "
                "Nên dời cube vào giữa bàn (px 200-440)."
            )
        z_candidates = [PICK_Z_M, 0.050, 0.060]
        request = Kinemarics.Request()
        request.kin_name = "ik"
        # Goc nghien TCP so voi mat ban. 1.04 = 60 do (nghien ve truoc).
        # PITCH KHONG tu y tang: do 2026-09-24 red
        # (-0.189,0.054,0.039) OK o 1.04/1.20 nhung FAIL o 1.396/1.50.
        request.pitch = PICK_PITCH
        request.yaw = -3.1416

        try:
            for zi, z_try in enumerate(z_candidates):
                request.tar_x = base_x
                request.tar_y = base_y
                request.tar_z = z_try
                self.get_logger().info(
                    f"IK req x={request.tar_x:.4f} y={request.tar_y:.4f} "
                    f"z={request.tar_z:.4f} pitch={request.pitch:.3f} yaw={request.yaw:.3f}"
                    + (f" (retry z #{zi + 1})" if zi else "")
                )
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future)

                if future.result() is None:
                    self.get_logger().error('Service call failed: no response')
                    return None
                response = future.result()
                # 解析关节角度
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5

                # The service currently returns a successful response with all
                # zeroes when IK fails. Never send that pose to the servos.
                # Het z candidate thi thu z cao hon truoc khi bo tay.
                if all(v == 0.0 for v in joints) or not all(0.0 <= v <= 180.0 for v in joints):
                    self.get_logger().error(
                        f"IK failed at z={z_try:.3f}: {joints}, "
                        + ("retrying higher z..." if zi + 1 < len(z_candidates)
                           else "all z tried, giving up.")
                    )
                    continue
                
                # 越界调整
                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                
                return joints
            self.get_logger().error(
                f"IK failed for all z candidates (x={base_x:.4f}, y={base_y:.4f})")
            return None
        except Exception as e:
            self.get_logger().info(f"arg error: {e}")
            return None

def main(args=None):
    # 创建节点实例
    stacking_node = stacking_GetTarget()
    try:
        # 保持节点运行
        rclpy.spin(stacking_node)
    except KeyboardInterrupt:
        stacking_node.get_logger().info("Node interrupted by user!")
    finally:
        # 销毁节点，释放资源
        stacking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
