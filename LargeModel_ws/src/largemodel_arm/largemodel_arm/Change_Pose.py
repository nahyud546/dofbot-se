#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool ,Int16MultiArray,String
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
from dofbot_interface.srv import Kinemarics
from Arm_Lib import Arm_Device
from arm_interface.msg import CurJoints

class mono_Tracker(Node):
    def __init__(self):
        super().__init__('monoIdentify') 
        self.Arm = Arm_Device()
        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.circle_r = 0
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 140.0]
        self.src_cols, self.src_raws = 0, 0
        self.tar_cols, self.tar_raws = 0, 0
        self.Mouse_XY = (0, 0)
        self.end = 0
        self.cx = 0
        self.cy = 0

        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        self.pubPoint = self.create_publisher(ArmJoint, "TargetAngle", 10)
        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.joint6_pub = self.create_publisher(Float32,'joint6',1)
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.down_joints_pub = self.create_publisher(CurJoints,'/down_joints',1)

        self.tracker_type = 'KCF'
        self.VideoSwitch = True
        self.img_flip = False

        print("OpenCV Version: ",cv.__version__)
        self.Track_state = 'init'

        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)

        self.xy_subscription = self.create_subscription(Int16MultiArray,'corner_xy',self.GetXYCallback,qos_profile=1)
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)
        self.grasp_c = self.grasp_cx = self.tar_cx = self.tar_cx = 0
        self.get_src_tar = False
        self.t_a = self.t_b = 0
        self.side = 0
        self.src_size = 0.0
        self.tar_size = 0.0
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting service...")
        print("Init Done")        
            
    def GetXYCallback(self,msg):
        print("msg: ",msg.data)    
        self.tar_cols = (msg.data[0],msg.data[1])
        self.tar_rows = (msg.data[2],msg.data[3])
       
        self.grasp_cx = (msg.data[0] + msg.data[2])/2
        self.grasp_cy = (msg.data[1] + msg.data[3])/2
        
        self.side = msg.data[4]
        
        self.tar_size = msg.data[5] * 0.01      
        
        self.get_src_tar = True
        

        

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
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
        if self.get_src_tar == True:
            (g_a, g_b) = (round(((320 - self.grasp_cx) / 4000), 5), round(((480 - self.grasp_cy) / 3000) * 0.8+0.15, 5))
            print("g_a: ",g_a)
            print("g_b: ",g_b)  

            self.move(g_b,g_a)

            self.get_src_tar = False

        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        if self.tar_cols!=0:
            cv.rectangle(result_frame, (self.tar_cols[0],self.tar_cols[1]), (self.tar_rows[0],self.tar_rows[1]), (0, 255, 255), 2)
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        cv.imshow(self.windows_name, result_frame)


    def grasp(self,x,y):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')	
            
        request = Kinemarics.Request()    
        request.tar_x = x -0.02
        request.tar_y = y - 0.02
        request.tar_z = self.src_size + 0.03
        request.kin_name = "ik"
        request.pitch= 1.04
        future = self.client.call_async(request)
        future.add_done_callback(self.grasp_get_ik_respone_callback)


    def grasp_get_ik_respone_callback(self, future):    
        try: 
            response = future.result()
            print("calcutelate_response: ",response)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = response.joint2 
            joints[2] = response.joint3
            joints[3] = response.joint4 
            joints[4] = 90
            joints[5] = 30
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
            time.sleep(3.5)
            self.Arm.Arm_serial_servo_write(6, 135, 2000)
            time.sleep(2.5)
            self.init_joints[5] = 135
            self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)    
            time.sleep(2.5)
            self.move()
        except Exception:
           pass

    def move(self,g_b,g_a):
        request = Kinemarics.Request()
        px = g_b  
        py = g_a 
        pz = self.tar_size
        #前
        if self.side == 1:
            px = px - 0.05
        #后
        elif self.side == 2:
            px = px + 0.05
        #左
        elif self.side == 3:
            py = py + 0.07
        #右
        elif self.side == 4:
            py = py - 0.07
        #上
        elif self.side == 5:
            pz = pz + 0.03
            
        request.tar_x = px - 0.03
        request.tar_y = py +0.01
        request.tar_z = pz + 0.03
        request.kin_name = "ik"
        request.pitch = 1.04
        future = self.client.call_async(request)
        future.add_done_callback(self.move_get_ik_respone_callback)


    def move_get_ik_respone_callback(self, future):    
        try: 
            response = future.result()
            print("calcutelate_response: ",response)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = 180 - response.joint2 
            joints[2] = 180 - response.joint3 
            joints[3] = 180 - response.joint4
            joints[4] = 90
            joints[5] = 140
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
            time.sleep(2.5)
            self.Arm.Arm_serial_servo_write(6, 0, 2000)
            time.sleep(2.5)
            cur_joints = CurJoints()
            cur_joints.joints = [int(x) for x in joints]
            self.down_joints_pub.publish(cur_joints)
            self.init_joints[5] = 30
            self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
            time.sleep(2.5)
            self.largemodel_arm_done_pub.publish(String(data='change_pose_done'))
        except Exception:
           pass
        
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


