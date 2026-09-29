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
from dofbot_driver.vutils import draw_tags
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool,Int16
import time
import queue
import math
from Arm_Lib import Arm_Device
from dofbot_driver.compute_joint5 import *

encoding = ['16UC1', '32FC1']

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
        self.sub_grasp_done = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)

        self.sub_gestureID = self.create_subscription(Int16,"GesturetId",self.GestureIDCallback,1)
        self.pub_reset = self.create_publisher(Bool, "/reset_gesture", qos_profile=10)
        
        self.rgb_bridge = CvBridge()
        self.pubPos_flag = False
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.Center_x_list = []
        self.Center_y_list = []
        self.dist = 0.13
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.target_id = 0
        self.detect = False

    def GestureIDCallback(self,msg):
        self.target_id = msg.data
        time.sleep(5.0)

        
    def get_current_end_pos(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'dofbot_kinematics' not available!")
            return
        request = Kinemarics.Request()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
        response = future.result()
        if isinstance(response, Kinemarics.Response):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw   
            print("self.CurEndPos: ",self.CurEndPos)

    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        self.Center_x_list = list(range(len(tags)))
        self.Center_y_list = list(range(len(tags)))
        key = cv2.waitKey(10)
        if key == 32:
            self.pubPos_flag = True
        if len(tags) > 0 :
            for i in range(len(tags)):
                if tags[i].tag_id == self.target_id:
                    self.detect_flag = True
                    self.pubPos_flag = True
                    center_x, center_y = tags[i].center
                    self.Center_x_list[i] = center_x
                    self.Center_y_list[i] = center_y
                    cx = center_x
                    cy = center_y
                    cz = self.dist
                    (a, b) = (round(((cx - 320) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.12, 5))
                    print("a: ",a)
                    print("b: ",b)    
                    if self.pubPos_flag == True:
                        self.pubPos_flag = False
                        pos = AprilTagInfo()
                        pos.x = b
                        pos.y = -a + 0.01
                        pos.z = 0.069-0.03
                        pos.id = tags[i].tag_id
                        self.pos_info_pub.publish(pos)
            if self.detect_flag == False and self.target_id !=0:
                reset = Bool()
                reset.data = True
                self.pub_reset.publish(reset)
                self.pubPos_flag = False
                
                    
                           
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
            self.detect_flag = False


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













