import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from dofbot_sorting_3d.color_common import *
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
from std_msgs.msg import Bool,Int16,Int8
import time
from rclpy.node import Node
import rclpy
import threading
from Arm_Lib import Arm_Device
print(cv2.__version__)
package_pwd =  "/home/yahboom/dofbot_ws/src/dofbot_voice_ctrl/dofbot_voice_ctrl/Color"
import os
from dofbot_sorting_3d.compute_joint5 import *

exit_code = os.system('sudo v4l2-ctl -d /dev/video0 -c brightness=10')

class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.init_joints = [90, 120, 0, 0, 90, 0]
		self.rgb_bridge = CvBridge()
		self.pub_pos_flag = False   
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)		
		self.pub_joint5 = self.create_publisher(Int16, "set_joint5", 10)
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
		self.sub_voice = self.create_subscription(Int8,"voice_result",self.getVoiceResultCallback,1)
		self.pub_playID = self.create_publisher(Int8,"player_id", 1)
		self.pubPos_flag = False
		print("Init done.")


	def getVoiceResultCallback(self,msg): 
		if msg.data == 100:
			self.target_color = "red"
			self.cur_target_color = 3
			self.hsv_range = read_HSV(self.red_hsv_text)
			self.pubPos_flag = True
		elif msg.data == 101:
			self.target_color = "green"
			self.cur_target_color = 2
			self.hsv_range = read_HSV(self.green_hsv_text)
			self.pubPos_flag = True
		elif msg.data == 102:
			self.target_color = "blue"
			self.cur_target_color = 1
			self.hsv_range = read_HSV(self.blue_hsv_text)
			self.pubPos_flag = True
		elif msg.data == 103:
			self.target_color = "yellow"
			self.cur_target_color = 4
			self.hsv_range = read_HSV(self.yellow_hsv_text)
			self.pubPos_flag = True
		print("Get the target color is ",self.target_color)
		self.Track_state = "identify"

       
	def Beep_Loop(self):
		beep = Bool()
		beep.data = True
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = False
		self.pub_beep.publish(beep)

	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			play_id = Int8()
			play_id.data = 81
			self.pub_playID.publish(play_id)
			time.sleep(2)
			self.pubPos_flag = True

			

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
		result_frame, binary = self.process(rgb_image)
		show_frame = threading.Thread(target=self.img_out, args=(result_frame,binary,))
		show_frame.start()
		show_frame.join()
		if self.cx!=0 and self.cy!=0 and self.circle_r>30 :
			cx = int(self.cx)
			cy = int(self.cy)
			(a, b) = (round(((cx - 320) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.13, 5))
			vx = self.corners[0][0][0] - self.corners[1][0][0]
			vy = self.corners[0][0][1] - self.corners[1][0][1]
			target_joint5 = compute_joint5(vx,vy)
			print("target_joint5: ",target_joint5)
			if self.pubPos_flag == True:
				self.pubPos_flag = False
				joint5 = Int16()
				joint5.data = int(target_joint5)
				self.pub_joint5.publish(joint5)
				pos = AprilTagInfo()
				pos.id = self.cur_target_color
				pos.x = float(b)
				pos.y = float(a)
				pos.z = 0.03
				self.pos_info_pub.publish(pos)  
                
		elif self.circle_r<30 and self.pubPos_flag == True:
			self.Reset()
			self.pubPos_flag = False
			play_id = Int8()
			play_id.data = 1
			self.pub_playID.publish(play_id)
        


	def process(self,rgb_img):
		rgb_img = cv.resize(rgb_img, (640, 480))
		binary = []
		print("self.Track_state: ",self.Track_state)
		print("self.target_color: ",self.target_color)
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
			if self.target_color == 3:
				self.hsv_range = read_HSV(self.red_hsv_text)
				self.cur_color = "red"
				self.text_color = (255, 0, 0)
                

                
			elif self.target_color == 2:
				self.hsv_range = read_HSV(self.green_hsv_text)
				self.cur_color = "green"
				self.text_color = (0, 255, 0)

                
			elif self.target_color == 1:
				self.hsv_range = read_HSV(self.blue_hsv_text)
				self.cur_color = "blue"
				self.text_color = (0, 0, 255)
                
			elif self.target_color == 4:
				self.hsv_range = read_HSV(self.yellow_hsv_text)
				self.cur_color = "yellow"
				self.text_color = (255, 255, 0)
                
		print("self.hsv_range: ",self.hsv_range)
		if self.Track_state != 'init':
			if len(self.hsv_range) != 0:
				rgb_img, binary, self.circle,_,self.corners= self.color.object_follow(rgb_img, self.hsv_range)
				self.cx = self.circle[0]
				self.cy = self.circle[1]
				self.circle_r = self.circle[2]
				if self.cur_target_color == 3 and self.updata_flag == True:
					write_HSV(self.red_hsv_text, self.hsv_range)
				elif self.cur_target_color  == 2 and self.updata_flag == True:
					write_HSV(self.green_hsv_text, self.hsv_range)
				elif self.cur_target_color == 1 and self.updata_flag == True:
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


           