#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool,String,Int16,Int16MultiArray,Float32MultiArray
import time
import queue
import math

encoding = ['16UC1', '32FC1']

class RecordPoseNode(Node):
    def __init__(self):
        super().__init__('record_pose')
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.joint5 = Int16()
        self.cur_joints = self.init_joints
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", qos_profile=1)
        
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.xy_subscription = self.create_subscription(Int16MultiArray,'corner_xy',self.GetXYCallback,100) 
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)
    
        self.current_pose_position_pub = self.create_publisher(Float32MultiArray, "current_pose", 1)
        
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = False
        self.pr_time = time.time()
        self.start_time = time.time()
        self.count = True
        self.cx = 0
        self.cy = 0
        self.pub_pose = True
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting service...")
        
        print("Init Done")
        


    def GetXYCallback(self,msg):
        print("msg: ",msg.data)
        print(msg.data[0])
        print(msg.data[1])        
        print(msg.data[2])
        print(msg.data[3])       
        self.cx = (msg.data[0] + msg.data[2])/2
        self.cy = (msg.data[1] + msg.data[3])/2
        self.cols = [msg.data[0],msg.data[1]]
        self.raws = [msg.data[2],msg.data[3]]


    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        if self.cx!=0 and self.cy!=0:
            (a, b) = (round(((320 - self.cx) / 4000), 5), round(((480 - self.cy) / 3000) * 0.8+0.15, 5))
            print("a: ",a)
            print("b: ",b)
            if self.pub_pose == True:
                self.current_pose_position_pub.publish(Float32MultiArray(data=[b, a, 0.03]))
                self.largemodel_arm_done_pub.publish(String(data="compute_pose_done"))
                self.pub_pose = False
                self.cx = 0
                self.cy = 0
            
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        key = cv2.waitKey(1)

    def GraspStatusCallback(self,msg):
        print("**")
        if msg.data == True:
            self.pubPos_flag = True
            self.compute_height = True
            self.detect_flag = False

def main(args=None):
    rclpy.init(args=args)
    record_pose = RecordPoseNode()
    try:    
        rclpy.spin(record_pose)
    except KeyboardInterrupt:
        pass
    finally:
        record_pose.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()













