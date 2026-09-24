#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32, Bool,String
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool,Int16
import time
import queue
import math
from Arm_Lib import Arm_Device
from ultralytics import YOLO
# Load the YOLO model
model = YOLO("/home/yahboom/colcon_ws/src/dofbot_garbage_yolov11/best.onnx")


class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect')
        self.Arm = Arm_Device()
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 0.0]
        self.joint5 = Int16()
        self.detect_flag = False
        self.compute_height = True
        self.index = None
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = True
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        self.dist = 0.13
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.start_flag = True
        self.recyclable_waste=['Newspaper','Zip_top_can','Book','Old_school_bag']
        self.toxic_waste=['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        self.wet_waste=['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        self.dry_waste=['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks']
        self.declare_parameter('target_type', 1.0)
        self.TargetType = int(self.get_parameter('target_type').get_parameter_value().double_value)
        print("Get self.TargetType is ",self.TargetType)
        
        if self.TargetType == 1:
            self.waste = ['Newspaper','Zip_top_can','Book','Old_school_bag']
        elif self.TargetType == 2:
            self.waste = ['Syringe','Expired_cosmetics','Used_batteries','Expired_tablets']
        elif self.TargetType == 3:
            self.waste = ['Fish_bone','Egg_shell','Apple_core','Watermelon_rind']
        elif self.TargetType == 4:
            self.waste = ['Toilet_paper','Peach_pit','Cigarette_butts','Disposable_chopsticks']
        print("self.waste: ",self.waste)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.count = False
        self.cnt = 0

    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        frame = np.copy(rgb_image)
        results = model(frame)
        #print("-------results-----------: ",results[0])
        key = cv2.waitKey(10)
        annotated_frame = results[0].plot()
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("YOLO Inference", cv2.resize(annotated_frame, (640, 480)))
        if key == 32:
            self.pubPos_flag = True 
        if self.count==True:
            if (time.time() - self.start_time)>20:
                self.start_flag = True
                self.count = False
                self.pubPos_flag = True
        boxes = results[0].boxes
        if boxes != [None] and self.start_flag == True:
            for box in boxes:  # detections per image
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                class_id = int(box.cls)
                confidence = float(box.conf)
                label = f"{model.names[class_id]} {confidence:.2f}"
                # 计算重心位置
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                (a, b) = (round(((320 - center_x) / 4000), 5), round(((480 - center_y) / 3000) * 0.8+0.14, 5))
                res_name = model.names[class_id]
                print("res_name: ",res_name)
                if res_name in self.waste:
                    print("++++++++++++++++++++++++++++++++++++++++++")
                    self.cnt = self.cnt + 1
                    self.detect_flag = True
                #center.result = str(self.yolo_model.names[class_id])
                if self.pubPos_flag == True and res_name in self.waste:
                    self.detect_flag = True
                    self.pubPos_flag = False
                    center = AprilTagInfo()
                    center.x = b
                    center.y = a
                    center.z = 0.03
                    if str(model.names[class_id]) in self.recyclable_waste:
                        center.id = 1
                    elif str(model.names[class_id]) in self.toxic_waste:
                        center.id = 2
                    elif str(model.names[class_id]) in self.wet_waste:
                        center.id = 3
                    elif str(model.names[class_id]) in self.dry_waste:
                        center.id = 4
                    print("center: ",center)
                    self.pos_info_pub.publish(center)
                    

                
            if 	self.detect_flag == False and self.cnt != 0 and self.start_flag == True:
                self.largemodel_arm_done_pub.publish(String(data='garbage_sort_done'))
                self.start_flag = False
                
            if 	self.detect_flag == False and self.cnt == 0 and self.start_flag == True :
                self.start_flag = False
                print("Did not find the target.")
                self.largemodel_arm_done_pub.publish(String(data='garbage_sort_no_found'))
                 

    def GraspStatusCallback(self,msg):
        print("**")
        if msg.data == True:
            self.pubPos_flag = True
            self.detect_flag = False
            self.compute_height = True


def main(args=None):
    rclpy.init(args=args)
    tag_detect = AprilTagDetectNode()
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

  












