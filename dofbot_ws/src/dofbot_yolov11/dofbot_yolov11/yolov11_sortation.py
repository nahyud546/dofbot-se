#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, String
import cv2 as cv
import threading


from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
from std_msgs.msg import Float32,Bool,Int16
import time
import queue
import math
from ultralytics import YOLO
import warnings
from pathlib import Path
warnings.filterwarnings("ignore")
# Load YOLO model with verbose=False to suppress output
model_path = Path(__file__).resolve().parent / "best.onnx"
if not model_path.is_file():
    raise FileNotFoundError(f"YOLO model not found: {model_path}")
model = YOLO(str(model_path), task='detect')


class YoloAllGarbageDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_all_garbage_detect')
        self.joint5 = Int16()
        self.detect_flag = False
        self.compute_height = True
        self.index = None
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.pubPos_flag = True  # 修改：初始为True，允许发布
        self.pr_time = time.time()
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []
        self.dist = 0.13
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.recyclable_waste=['Newspaper','Zip_top_can','Book','Old_school_bag']
        self.toxic_waste=['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        self.wet_waste=['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        self.dry_waste=['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks']
        self.all_waste = self.recyclable_waste + self.toxic_waste + self.wet_waste + self.dry_waste
        print("All waste types initialized.")
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.processed_items = set()
        self.last_publish_time = 0
        self.get_logger().info("yolov11_sortation.py initialized, pubPos_flag=True")

    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = np.frombuffer(color_frame.data, dtype=np.uint8).reshape(
            color_frame.height, color_frame.step)[:, :color_frame.width * 3].reshape(
            color_frame.height, color_frame.width, 3)
        result_image = np.copy(rgb_image)
        frame = np.copy(rgb_image)
        # Suppress verbose output during inference
        results = model(frame, verbose=False, conf=0.65, device='cpu')
        #print("-------results-----------: ",results[0])
        key = cv2.waitKey(10)
        if key == 32:
            self.pubPos_flag = True
            self.get_logger().info("Space pressed, pubPos_flag set to True")
        boxes = results[0].boxes
        if boxes is not None:
            for box in sorted(boxes, key=lambda b: float(b.conf), reverse=True):
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_id = int(box.cls)
                confidence = float(box.conf)
                label = f"{model.names[class_id]} {confidence:.2f}"
                # 计算重心位置
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                (a, b) = (round(((320 - center_x) / 4000), 5), round(((480 - center_y) / 3000) * 0.8+0.13, 5))
                res_name = model.names[class_id]
                
                # 检查是否是垃圾类型
                if res_name in self.all_waste:
                    # 检查是否已处理过此物品
                    item_key = f"{res_name}_{center_x}_{center_y}"
                    if item_key not in self.processed_items:
                        # 按照简单逻辑发布
                        if self.pubPos_flag == True:
                            self.get_logger().info(f"Publishing position for {res_name} (confidence={confidence:.2f}, pixel=({center_x},{center_y}))")
                            
                            self.pubPos_flag = False
                            center = AprilTagInfo()
                            center.x = b
                            center.y = a
                            center.z = 0.03
                            if str(model.names[class_id]) in self.recyclable_waste:
                                center.id = 1
                            elif str(model.names[class_id]) in self.toxic_waste:
                                center.id = 3
                            elif str(model.names[class_id]) in self.wet_waste:
                                center.id = 2
                            elif str(model.names[class_id]) in self.dry_waste:
                                center.id = 4
                            self.get_logger().info(f"Grasp target: {center}")
                            self.pos_info_pub.publish(center)
                            self.processed_items.add(item_key)
                            self.last_publish_time = time.time()
                            
                            # 添加短暂延时，确保机械臂接收消息
                            time.sleep(0.5)
            
        # 显示检测结果
        annotated_frame = results[0].plot()
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("YOLO Inference - All Garbage Detection", cv2.resize(annotated_frame, (640, 480)))
        key = cv2.waitKey(1)

    def GraspStatusCallback(self,msg):
        self.get_logger().info(f"GraspStatusCallback received: {msg.data}")
        if msg.data == True:
            self.pubPos_flag = True
            self.detect_flag = False
            self.compute_height = True
            self.get_logger().info("Grasp done, pubPos_flag reset to True")


def main(args=None):
    rclpy.init(args=args)
    tag_detect = YoloAllGarbageDetectNode()
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
