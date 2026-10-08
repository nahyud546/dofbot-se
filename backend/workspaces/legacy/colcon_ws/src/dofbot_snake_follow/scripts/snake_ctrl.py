# !/usr/bin/env python
# coding: utf-8
import math
import threading
import Arm_Lib
from time import sleep
from snake_move import snake_move
from dofbot_interface.srv import Kinemarics
from rclpy.node import Node
import rclpy

class snake_ctrl(Node):
    def __init__(self):
        super().__init__('identify')
        '''
        Initialize some parameters
        '''
        self.sbus = Arm_Lib.Arm_Device()
        self.arm_move = snake_move()
        self.color_name = None
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.move_time = 1000

        self.grap_status = 'Waiting'
        # Angle of servo when clip close
        self.grap_joint = 145
        self.num = 1
        self.move_num = 1


        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')

    def read_joint(self):
        '''
        Read current angle of the servo in loop
        '''
        num = 0
        for i in range(1, 6):
            while 1:
                self.get_logger().info(f"i:{i}")
                joint_bk = self.sbus.Arm_serial_servo_read(i)
                self.get_logger().info(f"joint_bk:{joint_bk}")
                if joint_bk == None:
                    joint_bk = 0.0
                joint = joint_bk*1.0
                self.get_logger().info(f"jointt:{joint}")
                # if num % 10 == 0: print("Please check!Poor contact!")
                if joint_bk == None:
                    joint = 0.0
                if joint != None:
                    self.cur_joint[i - 1] = joint
                    break
                num += 1

        # print("current angle of the servo: {}".format(cur_joint))

    def get_Posture(self):
        '''
        Obtain angle of servo
        '''
        self.read_joint()
        self.client.wait_for_service()
        self.get_logger().info(f"self.cur_joint:{self.cur_joint}")
        request = Kinemarics.Request()
        request.kin_name = "fk"
        request.cur_joint1 = 180 - self.cur_joint[0]
        request.cur_joint2 = 180 - self.cur_joint[1]
        request.cur_joint3 = 180 - self.cur_joint[2]
        request.cur_joint4 = 180 - self.cur_joint[3]
        request.cur_joint5 = 180 - self.cur_joint[4]
        
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()
            self.get_logger().info(f"pos_response:{response}")
            self.Posture[0] = response.x
            self.Posture[1] = response.y
            self.Posture[2] = response.z
            self.Posture[3] = response.Roll
            self.Posture[4] = response.Pitch
            self.Posture[5] = response.Yaw
        except Exception:
            self.get_logger().info("get_Posture error")
    def joints_limit(self, joints):
        request = Kinemarics.Request()
        request.cur_joint1 = 180 - joints[0]
        request.cur_joint2 = 180 - joints[1]
        request.cur_joint3 = 180 - joints[2]
        request.cur_joint4 = 180 - joints[3]
        request.cur_joint5 = 180 - joints[4]
        request.kin_name = "fk"
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()
            if response is not None:
                self.sbus.Arm_serial_servo_write6_array([90,joints[1],joints[2],joints[3],90,135], self.move_time)
        except Exception:
            self.get_logger().info("joints_limit error")

    def snake_run(self, point_y):
        '''
        Obtain angle of servo
        '''
        request = Kinemarics.Request()
        request.tar_x = point_y
        request.tar_y = 0.0
        request.tar_z = 0.225476
        request.pitch = 0.0
        request.kin_name = "ik"
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()
            self.get_logger().info(f"response:{response}")
            joints = [0.0, 0.0, 0.0, 0.0, 0.0]
            joints[0] = 90.0
            joints[1] =  180 - response.joint2
            joints[2] = 180 - response.joint3
            joints[3] = 180 - response.joint4
            joints[4] = 180 - response.joint5
            self.joints_limit(joints)
            
        except Exception:
            self.get_logger().info("snake_run error")

    def snake_main(self, name, msg):
  
        for key, area in msg.items():
            if key == name:
                # Estimate the position of the block according to the camera
                # 估计方块据摄像头的位置
                distance = 27.03 * math.pow(area, -0.51)
                # Estimate the position of the block in the world coordinate system
                # 估计方块在世界坐标系下的位置
                #self.get_Posture()
                target_dist = self.Posture[0]+distance
                #self.get_logger().info("target_dist: %.2f" % target_dist)
                if self.grap_status == 'Waiting':
                    threading.Thread(target=self.snake_run, args=(target_dist,)).start()

                    #太近了
                    if  target_dist <0.18 and distance<0.12 :
                        self.sbus.Arm_serial_servo_write(5,90,300)
                        sleep(0.3)
                        self.sbus.Arm_serial_servo_write(5,120, 300)
                        sleep(0.3)
                        self.num = 1
                    
                    elif 0.23<target_dist<0.26:
                        self.sbus.Arm_serial_servo_write(6,140,300)
                        sleep(0.4)
                        self.sbus.Arm_serial_servo_write(6,90,300)
                        sleep(0.4)
                        # Gripper opening and closing
                        # print("夹爪张合")
                        self.num += 1
                    else:
                        self.num = 1
                    if self.num % 30 == 0: self.grap_status = 'Graping'
                elif self.grap_status == 'Graping':
                    self.sbus.Arm_Buzzer_On(1)
                    self.grap_status = 'Runing'
                    # 执行放下 put down
                    self.arm_move.snake_run(name)
                    # 动作完毕 action completed
                    self.num = 1
                    # 设置移动状态 set mobile state
                    self.grap_status = 'Waiting'