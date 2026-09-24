#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import Bool,Int16,UInt16
from geometry_msgs.msg import Twist
import time
from dofbot_sorting_3d.media_library import *
from rclpy.node import Node
import rclpy
import threading
from Arm_Lib import Arm_Device
from dt_apriltags import Detector
from dofbot_driver.vutils import draw_tags
from dofbot_interface.msg import *

print('init done')
class MediapipeDetectNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.detect_joints = [90, 150, 12, 20, 90, 0]
		self.grasp_joints = [90, 120, 0, 0, 90, 0]
		self.Arm.Arm_serial_servo_write6_array(self.detect_joints,2000)
		self.rgb_bridge = CvBridge()
		self.hand_detector = HandDetector()
		self.pub_gesture = True
		self.cnt = 0
		self.last_sum = 0
		self.pTime = self.cTime = 0
		self.pub_GesturetId = self.create_publisher(Int16,"GesturetId",1)
		self.subscription = self.create_subscription(Bool,'/reset_gesture',self.get_resetCallBack,1)
		self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
		self.get_gesture = 0
		self.pubPos_flag = False

	
	def get_resetCallBack(self,msg):
		if msg.data == True:
			self.pub_gesture = True
			self.last_sum = 0
			self.cnt = 0
			self.Arm.Arm_serial_servo_write6_array(self.detect_joints,2000)
        


	def ImageCallback(self,color_msg):
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_msg, "bgr8")
		self.process(rgb_image)


	def process(self, frame):
		frame, lmList, bbox = self.hand_detector.findHands(frame)
		if len(lmList) != 0 and self.pub_gesture == True:
			gesture = threading.Thread(target=self.Gesture_Detect_threading, args=(lmList,bbox))
			gesture.start()
			gesture.join()
		self.cTime = time.time()
		fps = 1 / (self.cTime - self.pTime)
		self.pTime = self.cTime
		text = "FPS : " + str(int(fps))
		cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
		if cv.waitKey(1) & 0xFF == ord('q'):
			cv.destroyAllWindows()
		cv.imshow('frame', frame)

	def Gesture_Detect_threading(self, lmList,bbox):
		fingers = self.hand_detector.fingersUp(lmList)
		print("sum of fingers: ",sum(fingers))
		
		print(self.pub_gesture)
		if sum(fingers) == self.last_sum:
			print("---------------------------")
			self.cnt = self.cnt + 1
			print("cnt: ",self.cnt)
			if self.cnt==30 and self.pub_gesture == True:
				print("sum of fingers: ",self.last_sum)
				self.pub_gesture = False
				sum_gesture = Int16()
				sum_gesture.data = self.last_sum   
				self.pub_GesturetId.publish(sum_gesture)
				self.Arm.Arm_serial_servo_write6_array(self.grasp_joints,2000)				
		else:
			self.cnt = 0
		self.last_sum = sum(fingers)
			

def main():
	print('----------------------')
	rclpy.init()
	mediapipe_detect = MediapipeDetectNode('MediapipeDetect_node')
	rclpy.spin(mediapipe_detect)