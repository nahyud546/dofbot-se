#!/usr/bin/env python3
# coding: utf-8
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2 as cv
import numpy as np
from time import sleep
from numpy import random
from ultralytics import YOLO
import torch
from garbage_grap import garbage_grap_move
from dofbot_interface.srv import Kinemarics 
import math
import threading
import sys
import os
from Arm_Lib import Arm_Device

# 初始化ROS
rclpy.init(args=None)

# 设置设备（CPU/GPU）
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# 加载YOLOv11模型
# Bare-metal compat: thử path gốc /home/yahboom trước, fallback sang best.pt local
# để không cần sudo tạo /home/yahboom và không move source gốc.
try:
    _root = os.environ.get("ROBOT_ARM_ROOT", "/home/jloy/Desktop/robot-arm")
    _candidate_paths = [
        '/home/yahboom/ultralytics/ultralytics/data/yahboom_data/best.pt',
        os.path.join(os.path.dirname(__file__), 'best.pt'),
        os.path.join(_root, 'workspaces/dofbot_ws/src/dofbot_garbage_yolov11/best.pt'),
        os.path.join(_root, 'workspaces/legacy/colcon_ws/src/dofbot_garbage_yolov11/best.pt'),
        '/home/jloy/Desktop/robot-arm/dofbot_ws/src/dofbot_garbage_yolov11/best.pt',  # pre-restructure
        '/home/jloy/Desktop/robot-arm/colcon_ws/src/dofbot_garbage_yolov11/best.pt',  # pre-restructure
    ]
    model_path = next((p for p in _candidate_paths if os.path.exists(p)), _candidate_paths[0])
    #下面两个模型选一个，如果用pt模型打开下面的注释
    #model = YOLO(model_path).to(device)
    model = YOLO(model_path, task='detect') 
    # 获取类别名称和颜色
    names = model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
except Exception as e:
    print(f"model load failed: {e}")
    sys.exit(1)

class garbage_identify(Node):
    def __init__(self):
        super().__init__('dofbot_garbage')
        self.frame = None
        self.arm = Arm_Device(com="/dev/myserial")  
        
        # 机械臂识别位置调节
        self.xy = [90, 130]
        self.garbage_index = 0
        # 初始化抓取类
        self.grap_move = garbage_grap_move()
        self.grap_move.move_status = True  # 强制初始化状态
        
        # ROS服务配置
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(
            Kinemarics, 
            "dofbot_kinemarics",
            callback_group=self.callback_group
        )
        # 服务等待添加超时
        self.service_available = False
        for _ in range(5):  # 最多等待5秒
            if self.client.wait_for_service(timeout_sec=1.0):
                self.service_available = True
                break
            self.get_logger().info('Service not available, waiting again...')
        if not self.service_available:
            # Bare-metal: không sys.exit để Flask+TCP vẫn chạy ổn khi thiếu /dofbot_kinemarics.
            self.get_logger().warning('Service connection failed, degraded mode (server_joint may return None)')

        self.grap_lock = threading.Lock()
        # 服务调用超时时间（秒）
        self.SERVICE_TIMEOUT = 3.0

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=3):
        """绘制检测框"""
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv.rectangle(img, c1, c2, color, thickness=tl, lineType=cv.LINE_AA)
        if label:
            tf = max(tl - 1, 1)
            t_size = cv.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv.rectangle(img, c1, c2, color, -1, cv.LINE_AA)
            cv.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv.LINE_AA)

    def garbage_grap(self, msg, xy=None):
        """执行抓取函数"""
        if xy is not None:
            self.xy = xy
        if len(msg) == 0:
            return
        
        # 蜂鸣器提示
        try:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
            self.arm.Arm_Buzzer_Off()
        except Exception as e:
            self.get_logger().warning(f"buzzer control failed: {e}")
        
        # 加锁防止并发执行动作
        with self.grap_lock:
            for index, name in enumerate(msg):
                # 检查动作状态，异常时重置
                if not self.grap_move.move_status:
                    self.get_logger().warning("action status exception, reset move_status")
                    self.grap_move.move_status = True
                
                try:
                    # 获取关节角度
                    joints = self.server_joint(msg[name])
                    if joints is None:
                        self.get_logger().warning(f"{name} inverse kinematics failed, skip")
                        continue
                    # 执行抓取动作
                    self.get_logger().info(f"start grap {name}, joint angles: {joints}")
                    # 用线程执行动作，避免阻塞
                    grap_thread = threading.Thread(
                        target=self.grap_move.arm_run,
                        args=(str(name), joints),
                        daemon=True
                    )
                    grap_thread.start()
                    grap_thread.join(timeout=20.0)
                    if grap_thread.is_alive():
                        self.get_logger().error(f"{name} grap timeout, force stop")
                        self.grap_move.move_status = True 
                except Exception as e:
                    self.get_logger().error(f"{name} grap failed: {e}")
                    self.grap_move.move_status = True
                    continue
        
        try:
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
            self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
            sleep(1)
        except Exception as e:
            self.get_logger().error(f"return to initial position failed: {e}")

    def garbage_run(self, image):
        """垃圾识别主函数"""
        try:
            self.frame = cv.resize(image, (640, 480))
            txt0 = 'Model-Loading...'
            msg={}
            # 模型加载等待
            if self.garbage_index<3:
                cv.putText(self.frame, txt0, (190, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                self.garbage_index+=1
                return self.frame,msg 
            if self.garbage_index>=3:
                # 获取识别消息
                try: msg = self.get_pos()
                except Exception: 
                    self.get_logger().error("get_pos NoneType")
                return self.frame, msg
        except Exception as e:
            self.get_logger().error(f"garbage identification process exception: {e}")
            return image, {}

    def get_pos(self):
        """
        获取识别信息，坐标变换公式与 yolo11_sortation 保持一致
        :return: 名称,位置 name, location
        """
        img = self.frame.copy()
        msg = {}
        try:
            results = model(img, verbose=False, device=device, conf=0.65)
            if results and len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    xyxy = box.xyxy.view(-1).tolist()
                    conf = box.conf.item()
                    cls = int(box.cls.item())
                    name = names[cls]

                    # 置信度过滤（双重保障）
                    if conf < 0.4:
                        continue

                    cx = int((xyxy[0] + xyxy[2]) / 2)
                    cy = int((xyxy[1] + xyxy[3]) / 2)
                    cv.circle(self.frame, (cx, cy), 5, (0, 0, 255), -1)
                    label = '%s %.2f' % (name, conf)
                    self.plot_one_box(xyxy, self.frame, label=label,
                                      color=colors[cls], line_thickness=2)

                    a = round(((320 - cx) / 5000), 5)
                    b = round(((480 - cy) / 3000) * 0.8 + 0.16, 5)
                    msg[name] = (-b, -a)

        except Exception as e:
            self.get_logger().error(f"get position failed: {e}")
        return msg

    def server_joint(self, posxy):
        """ROS服务调用"""
        if not self.service_available:
            self.get_logger().error("kinemarics service not available, cannot get joint angles")
            return None
        
        try:
            request = Kinemarics.Request()
            request.tar_x = posxy[0]
            request.tar_y = posxy[1] - 0.0069
            request.tar_z = 0.069-0.03
            request.pitch = 1.04
            request.yaw = 0.0
            request.kin_name = "ik"
            self.get_logger().info(f"call kinemarics service: x={posxy[0]}, y={posxy[1]}")
            
            # 异步调用+超时控制
            future = self.client.call_async(request)
            # 超时等待
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > self.SERVICE_TIMEOUT:
                    future.cancel()
                    self.get_logger().error("kinemarics service call timeout")
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)  
            
            if future.done() and future.result() is not None:
                response = future.result()
                joints = [0, 0, 0, 0, 0]
                joints[0] = response.joint1
                joints[1] = response.joint2
                joints[2] = response.joint3
                joints[3] = response.joint4
                joints[4] = response.joint5 
                

                if joints[2] < 0:
                    joints[1] += joints[2] / 2
                    joints[3] += joints[2] * 3 / 4
                    joints[2] = 0
                
                # 角度合法性检查
                for i in range(5):
                    if not (0 <= joints[i] <= 180) and i != 4:
                        self.get_logger().warning(f"joint {i+1} angle exception: {joints[i]}")
                        joints[i] = max(0, min(joints[i], 180))
                joints[4] = max(0, min(joints[4], 270))
                return joints
            else:
                self.get_logger().error("kinemarics service call no response")
                return None
        except Exception as e:
            self.get_logger().error(f"kinemarics service call exception: {e}")
            return None

# 测试主函数
def main():
    try:
        # 创建节点
        node = garbage_identify()
        # 初始化摄像头（示例）
        cap = cv.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    node.get_logger().error("camera read failed")
                    sleep(1)
                    continue
                # 执行识别
                result_frame, msg = node.garbage_run(frame)
                # 执行抓取（识别到垃圾时）
                if len(msg) > 0:
                    node.garbage_grap(msg)
                # 显示画面
                cv.imshow("Garbage Detection", result_frame)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv.destroyAllWindows()
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        node.get_logger().error(f"main program exception: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
