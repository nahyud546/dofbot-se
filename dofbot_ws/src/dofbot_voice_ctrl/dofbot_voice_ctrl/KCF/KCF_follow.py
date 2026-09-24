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
from std_msgs.msg import Int32, Bool,UInt16,Int8
from Arm_Lib import Arm_Device
import dofbot_follow.PID as PID
from  dofbot_follow.Track_Lib import *
import os
exit_code = os.system('sudo v4l2-ctl -d /dev/video0 -c brightness=10')

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
        self.init_joints = [90.0, 150.0, 0.0, 20.0, 90.0, 0.0]
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.end = 0
        self.cx = 0
        self.cy = 0
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)
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

        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.get_xy = False
        self.Roi_init = [] 
        self.px = 0.0
        self.py = 0.0
        self.target_servox=90
        self.target_servoy=45
        self.xservo_pid = PID.PositionalPID(0.5, 0.01, 0.1)
        self.yservo_pid = PID.PositionalPID(0.5, 0.01, 0.1)
        self.y_out_range = False
        self.x_out_range = False
        self.a = 0
        self.b = 0
        self.start_follow = False
        

    def getVoiceResultCallBack(self,msg):
        if msg.data == 107:
            self.start_follow = True
            print("Start tracking .")
        elif msg.data == 76:
            self.Reset()
        

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.cx = 0
        self.cy = 0
        self.Roi_init = []
        self.cols, self.rows = 0, 0
        self.start_follow = False
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)


    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x,y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'identify'
            self.gTracker_state = True
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
        if action == 32:
            self.start_follow = True
            
            
        result_image = cv.resize(result_image, (640, 480))
        result_frame = self.process(result_image,action)  
        if self.cx!=0 and self.cy!=0 and self.start_follow == True:
            if (abs(self.cx-320) >10 or abs(self.cy-240)>10) :
                self.XY_track(self.cx,self.cy)
                print("Tracking")
                print("-------------------------------------")
            
            cv2.circle(result_frame, (int(self.cx),int(self.cy)), 10, (255,255,0), thickness=None, lineType=None, shift=None)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_frame, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)                              
        cv.imshow(self.windows_name, result_frame)

    def process(self, rgb_img, action):        
        rgb_img = cv.resize(rgb_img, (640, 480))

        if action == ord('r') or action == 114: self.Reset()

        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                
        if self.Track_state != 'init':
            if self.gTracker_state == True:
                Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                self.gTracker = Tracker(tracker_type=self.tracker_type)
                self.gTracker.initWorking(rgb_img, Roi)
                self.gTracker_state = False
            rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
            center_x = targEnd_x / 2 + targBegin_x / 2
            center_y = targEnd_y / 2 + targBegin_y / 2
            self.cx = center_x
            self.cy = center_y
            print(self.cx)
            print(self.cy)
        return rgb_img


    def XY_track(self,center_x,center_y):
        self.px = center_x
        self.py = center_y
        if not (self.target_servox>=180 and center_x<=320 and self.a == 1 or self.target_servox<=0 and center_x>=320 and self.a == 1):
            if(self.a == 0):
                self.xservo_pid.SystemOutput = center_x
                if self.x_out_range == True:
                    if self.target_servox<0:
                        self.target_servox = 0
                        self.xservo_pid.SetStepSignal(630)
                    if self.target_servox>0:
                        self.target_servox = 180
                        self.xservo_pid.SetStepSignal(10)
                    self.x_out_range = False
                else:
                    self.xservo_pid.SetStepSignal(320)
                    self.x_out_range = False
               
                self.xservo_pid.SetInertiaTime(0.01, 0.1)
                
                target_valuex = int(1500 + self.xservo_pid.SystemOutput)
                
                self.target_servox = int((target_valuex - 500) / 10) -10
        
                if self.target_servox > 180:
                    self.x_out_range = True
                    
                if self.target_servox < 0:
                    self.x_out_range = True
                 
        #180 240 0 240            
        if not (self.target_servoy>=180 and center_y<=240 and self.b == 1 or self.target_servoy<=0 and center_y>=240 and self.b == 1):
            if(self.b == 0):
                self.yservo_pid.SystemOutput = center_y

                if self.y_out_range == True:
                    self.yservo_pid.SetStepSignal(450)
                    self.y_out_range = False
                else:
                    self.yservo_pid.SetStepSignal(240)

                self.yservo_pid.SetInertiaTime(0.01, 0.1)
               
                target_valuey = int(1500 + self.yservo_pid.SystemOutput)
                
                if target_valuey<=1000:
                    target_valuey = 1000
                    self.y_out_range = True
                self.target_servoy = int((target_valuey - 500) / 10) - 55#int((target_valuey - 500) / 10) - 55
                if self.target_servoy > 180: self.target_servoy = 180 #if self.target_servoy > 390: self.target_servoy = 390
                if self.target_servoy < 0: self.target_servoy = 0 

                joint2 = 120 + self.target_servoy
                joint3 =  self.target_servoy / 4.5
                joint4 =  self.target_servoy / 3
                

        
        joints_0 = [float(self.target_servox/1), float(joint2), float(joint3), float(joint4), 90.0, 30.0]

        self.Arm.Arm_serial_servo_write6_array(joints_0,2500)

def main(args=None):
    rclpy.init(args=args)
    kcf_tracker = mono_Tracker()
    try:
        rclpy.spin(kcf_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


