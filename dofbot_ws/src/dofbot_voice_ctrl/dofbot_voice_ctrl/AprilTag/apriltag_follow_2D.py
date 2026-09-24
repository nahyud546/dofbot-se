import cv2
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from dofbot_sorting_3d.vutils import draw_tags
from dt_apriltags import Detector
from cv_bridge import CvBridge
import cv2 as cv

from std_msgs.msg import Float32,Bool,Int16,UInt16,String,Int8
encoding = ['16UC1', '32FC1']
import time
import math
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
import threading
import yaml
import dofbot_follow.PID as PID
from Arm_Lib import Arm_Device
import os
exit_code = os.system('sudo v4l2-ctl -d /dev/video0 -c brightness=10')
print('init done')


class AprilTagTrackNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.init_joints = [90, 150, 0, 20, 90, 0]
		self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
		self.rgb_bridge = CvBridge()
		self.pr_time = time.time()
		self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

		self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        
		self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)

		self.px = 0.0
		self.py = 0.0
		self.target_servox=90
		self.target_servoy=45
		self.xservo_pid = PID.PositionalPID(0.5, 0.1, 0.1)
		self.yservo_pid = PID.PositionalPID(0.5, 0.1, 0.1)
		self.y_out_range = False
		self.x_out_range = False
		self.a = 0
		self.b = 0
		self.start_flag = False

	def getVoiceResultCallBack(self,msg):
		if msg.data == 105:
			self.start_flag = True
		elif msg.data == 76:
			self.start_flag = False
			self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
            


			         
	def ImageCallback(self,color_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        
		tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
		tags = sorted(tags, key=lambda tag: tag.tag_id) 
		draw_tags(rgb_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
		frame1 = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
		cv2.imshow("result_image", frame1)
		key = cv2.waitKey(1)
		if len(tags) > 0 :
            #print("tag: ",tags)
			cur_id = tags[0].tag_id
			center_x, center_y = tags[0].center
			if (abs(center_x-320) >10 or abs(center_y-240)>10) and self.start_flag == True:
				self.XY_track(center_x,center_y)
				print("Tracking")
				print("-------------------------------------")
		
                        
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
				print("self.target_servox: ",self.target_servox)
                 
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

		self.Arm.Arm_serial_servo_write6_array(joints_0,2000)
 

		   
def main():
	print('----------------------')
	rclpy.init()
	apriltag_track = AprilTagTrackNode('ApriltagTrack_node')
	rclpy.spin(apriltag_track)

           