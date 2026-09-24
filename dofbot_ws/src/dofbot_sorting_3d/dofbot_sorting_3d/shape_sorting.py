import cv2
import cv2 as cv
import os
import numpy as np
import time
from Arm_Lib import Arm_Device
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dofbot_sorting_3d.color_common import *
from dofbot_interface.msg import *
from std_msgs.msg import Bool,Int16
from rclpy.node import Node
import rclpy
import threading
print(cv2.__version__)
package_pwd =  "/home/yahboom/dofbot_ws/src/dofbot_sorting_3d/dofbot_sorting_3d"
print('init done')
class ColorRecognizeNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.Arm = Arm_Device()
		self.init_joints = [90, 120, 0, 0, 90, 0]
		self.rgb_bridge = CvBridge()
		self.pub_pos_flag = False
        
		self.pos_info_pub = self.create_publisher(AprilTagInfo,"PosInfo",1)
		self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)
  
		self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)

		self.target_color = 0
		self.red_hsv_text = os.path.join(package_pwd, 'red_colorHSV.text')
		self.green_hsv_text = os.path.join(package_pwd, 'green_colorHSV.text')
		self.blue_hsv_text = os.path.join(package_pwd, 'blue_colorHSV.text')
		self.yellow_hsv_text = os.path.join(package_pwd, 'yellow_colorHSV.text')
		self.hsv_range = ()
		self.Roi_init = ()
		self.roi_hsv_range = ()  # 初始化ROI区域的HSV范围
		self.select_flags = False
		self.windows_name = 'frame'
		self.Track_state = 'init'
		self.Mouse_XY = (0, 0)
		self.cols, self.rows = 0, 0
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
		self.target_shape = "Square"
		self.shape_x = self.shape_y = 0
		self.grasp_done = True
		self.dyn_update = False  # 初始化动态更新标志
		
		# 声明HSV参数
		self.declare_parameter("Hmin", 0)
		self.declare_parameter("Smin", 85)
		self.declare_parameter("Vmin", 126)
		self.declare_parameter("Hmax", 9)
		self.declare_parameter("Smax", 253)
		self.declare_parameter("Vmax", 253)
		
		print("Init done.")

    

       
	def Beep_Loop(self):
		beep = Bool()
		beep.data = True
		self.pub_beep.publish(beep)
		time.sleep(1.0)
		beep.data = False
		self.pub_beep.publish(beep)

	def get_current_end_pos(self):
		request = Kinemarics.Request()
		request.cur_joint1 = float(self.init_joints[0])
		request.cur_joint2 = float(self.init_joints[1])
		request.cur_joint3 = float(self.init_joints[2])
		request.cur_joint4 = float(self.init_joints[3])
		request.cur_joint5 = float(self.init_joints[4])
		request.kin_name = "fk"
		future = self.client.call_async(request)
		future.add_done_callback(self.get_fk_respone_callback)

	def get_fk_respone_callback(self, future):
		try:
			response = future.result()
			#self.get_logger().info(f'Response received: {response.x}')
			self.CurEndPos[0] = response.x 
			self.CurEndPos[1] = response.y
			self.CurEndPos[2] = response.z 
			self.CurEndPos[3] = response.roll
			self.CurEndPos[4] = response.pitch
			self.CurEndPos[5] = response.yaw
			print("self.CurEndPose: ",self.CurEndPos)
		except Exception as e:
			self.get_logger().error(f'Service call failed: {e}')


	def get_graspStatusCallBack(self,msg):
		if msg.data == True:
			time.sleep(2)
			self.pub_pos_flag = True
			self.grasp_done = True
			self.adjust_dist = True
			self.valid_dist = True
			


	def Reset(self):
		self.hsv_range = ()
		self.circle = (0, 0, 0)
		self.Mouse_XY = (0, 0)
		self.Track_state = 'init'
		print("Change state.")
		self.cx = 0
		self.cy = 0
		self.pubPos_flag = False

	def img_out(self,result_frame,binary):
		if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(0.8, ([result_frame, binary])))
		else:
			cv.imshow(self.windows_name, result_frame)

	def onMouse(self, event, x, y, flags, param):
		# 鼠标左键点击事件
		if event == 1:
			self.Track_state = 'init'
			self.select_flags = True
			self.Mouse_XY = (x, y)
		# 鼠标左键释放事件
		if event == 4:
			self.select_flags = False
			self.Track_state = 'select'
   
		if self.select_flags == True:
			#起点x,y 终点x,y
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
            
	def ImageCallback(self,color_frame):
        # 将画面转为 opencv 格式
		rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
		rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
		result_image = np.copy(rgb_image)
		#空格键触发发布位置信息
		key = cv2.waitKey(10)
		if key == 32:
			self.pub_pos_flag = True
		result_frame, binary = self.process(rgb_image,key)
		show_frame = threading.Thread(target=self.img_out, args=(result_frame,binary,))
		show_frame.start()
		show_frame.join()
		# 形状识别逻辑
		if self.hsv_range and self.grasp_done == True:
			# 设置当前目标形状
			self.color.target_shape = self.target_shape
			# 进行形状识别
			result_image, binary_processed, circle_info, corner_info = self.color.ShapeRecognition(rgb_image, self.hsv_range)
			
			# 检查是否检测到目标形状
			if self.color.shape_cx != 0 and self.color.shape_cy != 0:
				print("Find the target shape block")
				self.shape_x = self.color.shape_cx
				self.shape_y = self.color.shape_cy
				print("self.shape_x: ", self.shape_x)
				print("self.shape_y: ", self.shape_y)
				# 在结果图像上标注形状名称
				cv2.putText(result_image, self.color.shape_name, (int(self.color.shape_cx) - 20, int(self.color.shape_cy)), 
				            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
							
			if self.shape_x!=0 and self.shape_y!=0 and self.color.shape_name == self.target_shape:  
				print("...................................................")
				cx = int(self.shape_x)
				cy = int(self.shape_y)
				(a, b) = (round(((320 - cx) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.12, 5))
				pos = AprilTagInfo()
				pos.id = self.target_color
				pos.x = float(b)  
				pos.y = float(a)+0.01
				pos.z = 0.069-0.03
				if self.pub_pos_flag == True:
					print("--------------------")
					self.pub_pos_flag = False
					self.pos_info_pub.publish(pos)    
					print(f"Published position info: ID={pos.id}, x={pos.x}, y={pos.y}, z={pos.z}")
					print("*******************")
					self.shape_x = 0
					self.shape_y = 0    
					self.grasp_done = False
		cv.imshow("detect_shape", result_image)

	def process(self,rgb_img,key):
		rgb_img = cv.resize(rgb_img, (640, 480))
		binary = []
		if key == ord('c') or key == ord('C'):
			self.target_color = 0
			self.Reset()  
			self.updata_flag = True
		elif key == ord('r') or key == ord('R'): 
			self.target_color = 3
			self.cur_target_color = self.target_color
		elif key == ord('g') or key == ord('G'): 
			self.target_color = 2
			self.cur_target_color = self.target_color
		elif key == ord('b') or key == ord('B'): 
			self.target_color = 1
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
					self.roi_hsv_range = self.hsv_range  # 保存ROI区域的HSV范围
					self.dyn_update = True
				else: self.Track_state = 'init'
                    
		elif self.Track_state == "identify":
			if self.target_color == 3:
				self.hsv_range = read_HSV(self.red_hsv_text)
				self.cur_color = "red"
				self.text_color = (0, 0, 255)

                
			elif self.target_color == 2:
				self.hsv_range = read_HSV(self.green_hsv_text)
				self.cur_color = "green"
				self.text_color = (0, 255, 0)

                
			elif self.target_color == 1:
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
				if self.dyn_update is True:
					# 根据当前目标颜色写入到相应的HSV文件
					if self.target_color == 3:
						write_HSV(self.red_hsv_text, self.hsv_range)
					elif self.target_color == 2:
						write_HSV(self.green_hsv_text, self.hsv_range)
					elif self.target_color == 1:
						write_HSV(self.blue_hsv_text, self.hsv_range)
					elif self.target_color == 4:
						write_HSV(self.yellow_hsv_text, self.hsv_range)
					
					# 更新ROS参数
					self.Hmin = rclpy.parameter.Parameter('Hmin', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[0][0]))
					self.Smin = rclpy.parameter.Parameter('Smin', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[0][1]))
					self.Vmin = rclpy.parameter.Parameter('Vmin', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[0][2]))
					self.Hmax = rclpy.parameter.Parameter('Hmax', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[1][0]))
					self.Smax = rclpy.parameter.Parameter('Smax', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[1][1]))
					self.Vmax = rclpy.parameter.Parameter('Vmax', rclpy.Parameter.Type.INTEGER, int(self.roi_hsv_range[1][2]))
					all_new_parameters = [self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax]
					self.set_parameters(all_new_parameters)
					self.dyn_update = False
					
				self.cx = self.circle[0]
				self.cy = self.circle[1]
				self.circle_r = self.circle[2]
				if self.cur_target_color == 3 and self.updata_flag == True:
					write_HSV(self.red_hsv_text, self.hsv_range)  # #红色
				elif self.cur_target_color  == 2 and self.updata_flag == True:
					write_HSV(self.green_hsv_text, self.hsv_range)  # #绿色
				elif self.cur_target_color == 1 and self.updata_flag == True:
					write_HSV(self.blue_hsv_text, self.hsv_range)  # #蓝色
				elif self.cur_target_color == 4 and self.updata_flag == True:
					write_HSV(self.yellow_hsv_text, self.hsv_range)  # #黄色
				self.updata_flag = False
                    
		rgb_img = cv2.putText(rgb_img, self.cur_color, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, self.text_color, 2)             
		return rgb_img, binary



		   
def main():
	print('----------------------')
	rclpy.init()
	color_recognize = ColorRecognizeNode('ColorRecognize_node')
	rclpy.spin(color_recognize)