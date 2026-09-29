#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool,Int16
from cv_bridge import CvBridge
import cv2 as cv

import getpass
import threading
encoding = ['16UC1', '32FC1']
import time
import math
import os
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Int32, Bool,UInt16
from dofbot_interface.msg import *
from Arm_Lib import Arm_Device
class mono_Tracker(Node):
    def __init__(self):
        super().__init__('monoIdentify') 
        self.Arm = Arm_Device()
        self.select_flags = False
        self.windows_name = 'frame'
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 0.0]
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.cx = 0
        self.cy = 0
        self.rgb_bridge = CvBridge()
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.joint6_pub = self.create_publisher(Int16,'joint6',1)
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        print("OpenCV Version: ",cv.__version__)
        self.Track_state = 'init'
        self.pr_time = time.time()
        self.corner_x = self.corner_y = 0.0
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)

    def Reset(self):
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'


    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.cx = (self.cols[0] + self.rows[0])/2
            self.cy = (self.cols[1] + self.rows[1])/2
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            print("self.Roi_init: ",self.Roi_init)

    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        #depth_image
        action = cv.waitKey(10) & 0xFF
        result_image = cv.resize(result_image, (640, 480))
        result_frame = self.process(result_image,action)  
        if self.cx!=0 and self.cy!=0:
            (a, b) = (round(((320 - self.cx) / 4000), 5), round(((480 - self.cy) / 3000) * 0.8+0.15, 5))
            print("a: ",a)
            print("b: ",b)

            (la, lb) = (round(((320 - self.cols[0]) / 4000), 5), round(((480 - self.cols[1]) / 3000) * 0.8+0.15, 5))
            print("la: ",la)
            print("lb: ",lb)
            (ra, rb) = (round(((320 - self.rows[0]) / 4000), 5), round(((480 - self.rows[1]) / 3000) * 0.8+0.15, 5))
            print("ra: ",ra)
            print("rb: ",rb)
            res_joint6 = abs(lb) - abs(rb)
            print("res_joint6: ",res_joint6)
            val_res_joint6 = 180 - res_joint6*1000*1.2
            print("val_res_joint6: ",val_res_joint6)
            if action == 32:
                joint6 = Int16()
                joint6.data = int(val_res_joint6)
                self.joint6_pub.publish(joint6)
                pos = AprilTagInfo()
                pos.x = b
                pos.y = a
                pos.z = 0.04
                self.pos_info_pub.publish(pos)
                self.cols = self.rows = 0   
                self.cx = self.cy = 0

        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        if self.cols!=0 and self.rows!=0:
            cv.rectangle(result_frame, self.cols, self.rows, (0, 255, 0), 2)
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        cv.imshow(self.windows_name, result_frame)

    def process(self, rgb_img, action):        
        rgb_img = cv.resize(rgb_img, (640, 480))

        if action == ord('r') or action == 114:
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
        return rgb_img

    def pub_arm(self, joints, id=6, angle=180.0, runtime=1500):
        arm_joint = ArmJoint()
        arm_joint.id = id
        arm_joint.angle = angle
        arm_joint.run_time = runtime
        arm_joint.joints = joints
        self.pubPoint.publish(arm_joint)

def main(args=None):
    rclpy.init(args=args)
    kcf_tracker = mono_Tracker()
    kcf_tracker.pub_arm(kcf_tracker.init_joints)
    try:
        rclpy.spin(kcf_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


