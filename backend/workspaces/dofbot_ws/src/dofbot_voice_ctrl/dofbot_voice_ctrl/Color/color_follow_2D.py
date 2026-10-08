import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_sorting_3d.color_common import *
from std_msgs.msg import Bool,Int16,Int8
import time
from rclpy.node import Node
import rclpy
import threading
from Arm_Lib import Arm_Device
import dofbot_follow.PID as PID
print(cv2.__version__)
package_pwd =  "/home/yahboom/dofbot_ws/src/dofbot_sorting_3d/dofbot_sorting_3d/"
print('init done')

class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.init_joints = [90, 150, 0, 20, 90, 0]
		self.rgb_bridge = CvBridge()
		self.pub_pos_flag = False

		self.target_color = 0
		self.red_hsv_text = os.path.join(package_pwd, 'red_colorHSV.text')
		self.green_hsv_text = os.path.join(package_pwd, 'green_colorHSV.text')
		self.blue_hsv_text = os.path.join(package_pwd, 'blue_colorHSV.text')
		self.yellow_hsv_text = os.path.join(package_pwd, 'yellow_colorHSV.text')
		self.hsv_range = ()
		self.select_flags = False
		self.windows_name = 'frame'
		self.Track_state = 'init'
		self.Mouse_XY = (0, 0)
		self.cols, self.rows = 0, 0
		self.Roi_init = ()
		self.color = color_detect()
		self.cur_color = None
		self.text_color = (0,0,0)
		self.cx = 0
		self.cy = 0
		self.circle_r = 0
		self.joint5 = Int16()
		self.corners = np.empty((4, 2), dtype=np.int32)
		self.cur_target_color = 0
		self.updata_flag = False
		self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
		self.dist = 0.13
		self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
		self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallBack,1)

		self.px = 0.0
		self.py = 0.0
		self.target_servox=90
		self.target_servoy=45
		#self.xservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
		#self.yservo_pid = PID.PositionalPID(0.25, 0.1, 0.05)
		self.xservo_pid = PID.PositionalPID(0.5, 0.1, 0.1)
		self.yservo_pid = PID.PositionalPID(0.5, 0.1, 0.1)
		self.y_out_range = False
		self.x_out_range = False
		self.cur_joints = self.init_joints
		self.a = 0
		self.b = 0
		self.start_flag = False
    
		print("Init done.")

	def getVoiceResultCallBack(self,msg):
		if msg.data == 106:
			self.start_flag = True
			print("Start tracking.")
		elif msg.data == 76:
			self.start_flag = False
			self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
			self.Reset()


	def Reset(self):
		self.hsv_range = ()
		self.circle = (0, 0, 0)
		self.Mouse_XY = (0, 0)
		self.Track_state = 'init'  
		print("Change the state.")
		self.cx = 0
		self.cy = 0

	def img_out(self,result_frame,binary):
		if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(0.8, ([result_frame, binary])))
		else:
			cv.imshow(self.windows_name, result_frame)

	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x, y)
		if event == 4:
			self.select_flags = False
			self.Track_state = 'select'
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            
	def ImageCallback(self,color_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
		result_image = np.copy(rgb_image)  
		key = cv2.waitKey(10)& 0xFF
		result_frame, binary = self.process(rgb_image,key)
		show_frame = threading.Thread(target=self.img_out, args=(result_frame,binary,))
		show_frame.start()
		show_frame.join()
		if self.cx!=0 and self.cy!=0 and self.circle_r>30:
			center_x = self.cx
			center_y = self.cy
			if (abs(center_x-320) >10 or abs(center_y-240)>10) and self.start_flag == True:
				self.XY_track(center_x,center_y)
				print("Tracking")
				print("-------------------------------------" )
            

	def process(self,rgb_img,key):
		rgb_img = cv.resize(rgb_img, (640, 480))
		binary = []
		
		if key == ord('c') or key == ord('C'):
			self.target_color = 0
			self.Reset()  
			self.updata_flag = True
		elif key == ord('r') or key == ord('R'): 
			self.target_color = 1
			self.cur_target_color = self.target_color
		elif key == ord('g') or key == ord('G'): 
			self.target_color = 2
			self.cur_target_color = self.target_color
		elif key == ord('b') or key == ord('B'): 
			self.target_color = 3
			self.cur_target_color = self.target_color
		elif key == ord('y') or key == ord('Y'): 
			self.target_color = 4
			self.cur_target_color = self.target_color
		elif key == ord('i') or key == ord('I') or self.target_color!=0: self.Track_state = "identify"
		#print("self.Track_state: ",self.Track_state)
		if self.Track_state == 'init':
			cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
			cv.setMouseCallback(self.windows_name, self.onMouse, 0)
			if self.select_flags == True:
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0),  2)
				cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
				if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
					rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
					self.dyn_update = True
				else: self.Track_state = 'init'
                    
		elif self.Track_state == "identify":
			if self.target_color == 1:
				self.hsv_range = read_HSV(self.red_hsv_text)
				self.cur_color = "red"
				self.text_color = (0, 0, 255)

                
			elif self.target_color == 2:
				self.hsv_range = read_HSV(self.green_hsv_text)
				self.cur_color = "green"
				self.text_color = (0, 255, 0)

                
			elif self.target_color == 3:
				self.hsv_range = read_HSV(self.blue_hsv_text)
				self.cur_color = "blue"
				self.text_color = (255, 0, 0)
                
			elif self.target_color == 4:
				self.hsv_range = read_HSV(self.yellow_hsv_text)
				self.cur_color = "yellow"
				self.text_color = (255, 255, 0)
                
			else: 
				self.Track_state = 'init'

		if self.Track_state != 'init':
			if len(self.hsv_range) != 0:
				rgb_img, binary, self.circle,_,self.corners= self.color.object_follow(rgb_img, self.hsv_range)
				self.cx = self.circle[0]
				self.cy = self.circle[1]
				self.circle_r = self.circle[2]
				if self.cur_target_color == 1 and self.updata_flag == True:
					write_HSV(self.red_hsv_text, self.hsv_range)
				elif self.cur_target_color  == 2 and self.updata_flag == True:
					write_HSV(self.green_hsv_text, self.hsv_range)
				elif self.cur_target_color == 3 and self.updata_flag == True:
					write_HSV(self.blue_hsv_text, self.hsv_range)
				elif self.cur_target_color == 4 and self.updata_flag == True:
					write_HSV(self.yellow_hsv_text, self.hsv_range)
				self.updata_flag = False
                    
		rgb_img = cv2.putText(rgb_img, self.cur_color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, self.text_color, 2)             
		return rgb_img, binary

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

		self.Arm.Arm_serial_servo_write6_array(joints_0,2000)
		self.cur_joints = joints_0



		   
def main():
	print('----------------------')
	rclpy.init()
	color_recognize = ColorRecognizeNode('ColorRecognize_node')
	rclpy.spin(color_recognize)

           