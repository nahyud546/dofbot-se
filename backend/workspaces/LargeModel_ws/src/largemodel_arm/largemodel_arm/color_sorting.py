import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_sorting_3d.color_common import *
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
from std_msgs.msg import Bool,Int16
import time
from rclpy.node import Node
import rclpy
import threading
from Arm_Lib import Arm_Device
print(cv2.__version__)
package_pwd =  "/home/yahboom/dofbot_ws/src/dofbot_sorting_3d/dofbot_sorting_3d/"
print('init done')

class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.init_joints = [90, 120, 0, 0, 90, 0]
		self.rgb_bridge = CvBridge()
		self.pub_pos_flag = False
        
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

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
    
		self.declare_parameter('target_color', 0.0)
		self.target_color = int(self.get_parameter('target_color').get_parameter_value().double_value)
		print("Get self.target_color is ",self.target_color)
        
		print("Init done.")
		self.statr_flag  = False
		self.count = True
		self.start_time = time.time()
		self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1) 

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			time.sleep(2)
			self.pub_pos_flag = True
			self.adjust_dist = True
			self.valid_dist = True
			

	def Reset(self):
		self.hsv_range = ()
		self.circle = (0, 0, 0)
		self.Mouse_XY = (0, 0)
		self.Track_state = 'init'
		print("Change the state.")
		self.cx = 0
		self.cy = 0
		self.pubPos_flag = False

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
		if self.count==True:
			if (time.time() - self.start_time)>15:
				self.statr_flag = True
				self.count = False
		if key == 32:
			self.pub_pos_flag = True
		if self.cx!=0 and self.cy!=0 and self.circle_r>30:
			cx = int(self.cx)
			cy = int(self.cy)
			(a, b) = (round(((320 - cx) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.15, 5))
			vx = self.corners[0][0][0] - self.corners[1][0][0]
			vy = self.corners[0][0][1] - self.corners[1][0][1]
			#target_joint5 = compute_joint5(vx,vy)
			#self.joint5.data = int(target_joint5)                    
			pos = AprilTagInfo()
			pos.id = self.target_color
			pos.x = float(b)
			pos.y = float(a)
			pos.z = 0.03
			if self.pub_pos_flag == True:
				self.pub_pos_flag = False
				self.pos_info_pub.publish(pos)    
				#self.TargetJoint5_pub.publish(self.joint5)

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
				cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
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

		   
def main():
	print('----------------------')
	rclpy.init()
	color_recognize = ColorRecognizeNode('ColorRecognize_node')
	rclpy.spin(color_recognize)

           