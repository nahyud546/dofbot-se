#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Bool,Int16,Int8
import time
from dofbot_interface.msg import *       # 需确认ROS2消息包名是否一致
from dofbot_interface.srv import Kinemarics
import threading
import os
from Arm_Lib import Arm_Device
from dofbot_interface.msg import *

class GraspNode(Node):
    def __init__(self):
        super().__init__('grap')
        self.Arm = Arm_Device()
        self.sub = self.create_subscription(AprilTagInfo,'PosInfo',self.pos_callback,1)
        self.sub_joint5 = self.create_subscription(Int16,"set_joint5",self.get_joint5_callback,1)
        self.sub_joint6 = self.create_subscription(Int16,"set_joint6",self.get_joint6_callback,1)
        self.pubGraspStatus = self.create_publisher(Bool, 'grasp_done',1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        
        self.grasp_flag = True
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.down_joint = [10.0, 70.0, 34.0, 16.0, 90.0,140.0]
        self.gripper_joint = 90.0
        self.cur_tagId = 0
        self.joint_5 = 90
        self.set_joint5 = 90
        self.joint_6 = 140
        self.set_id = 0
        print("Init Done")


    def get_joint5_callback(self,msg):
        self.joint_5 = msg.data

    def get_joint6_callback(self,msg):
        self.joint_6 = msg.data
        

    def pos_callback(self,msg):
        pos_x = msg.x
        pos_y = msg.y
        pos_z = msg.z
        self.cur_tagId = msg.id
        print("msg: ",msg)
        if self.grasp_flag == True:
            print("Take it now.")
            self.grasp_flag = False
            threading.Thread(target=self.grasp, args=(pos_x,pos_y,pos_z)).start()

    def grasp(self,pos_x,pos_y,pos_z):
        print("------------------------------------------------")
        request = Kinemarics.Request()
        request.tar_x = pos_x 
        request.tar_y = pos_y 
        request.tar_z = pos_z + 0.03
        request.kin_name = "ik"
        request.pitch = 1.04
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()
            print("calcutelate_response: ",response)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = 180-response.joint1 #response.joint1
            joints[1] = 180-response.joint2 
            joints[2] = 180-response.joint3  
            joints[3] = 180-response.joint4     
            print("self.joint_5: ",self.joint_5)
            target_joint5 = self.joint_5
            if target_joint5>0 and target_joint5 < 45:
                joint5 = 90 - target_joint5                 
            elif target_joint5>45 and target_joint5<135:
                joint5 = target_joint5
            elif target_joint5>=135:
                joint5 = target_joint5 - 90                
            elif target_joint5>-45 and target_joint5<0:
                joint5 = 90 + abs(target_joint5)                  
            elif target_joint5<-45 and target_joint5>-135:
                joint5 = abs(target_joint5)               
            elif target_joint5<-135:
                joint5 = abs(target_joint5) - 90
            self.set_joint5 = 180 - joint5
            joints[5] = 30
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],90,joints[5],2000)
            time.sleep(3.5)
            self.move()

        except Exception:
           pass

    def move(self):
        self.Arm.Arm_serial_servo_write(5, 90, 2000)
        time.sleep(2.0)        
        self.Arm.Arm_serial_servo_write(6, 140, 2000)
        time.sleep(2.0)        
        self.Arm.Arm_serial_servo_write(6, self.joint_6, 2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.0)
        print("self.cur_tagId",self.cur_tagId)
        #蓝色
        p_1 = [30, 70, 0, 54, 265, self.joint_6]
        #绿色
        p_2 = [150, 70, 7, 56, 265, self.joint_6]
        #红色
        p_3 = [50, 70, 7, 58, 265, self.joint_6]
        #黄色
        p_4 = [135, 70, 7, 54, 265, self.joint_6]
        if self.cur_tagId == 1:
            self.down_joint = p_1
        elif self.cur_tagId == 2:
            self.down_joint = p_2
        elif self.cur_tagId == 3:
            self.down_joint = p_3
        elif self.cur_tagId == 4:
            self.down_joint = p_4   
        print("self.down_joint",self.down_joint)
        self.down_joint[5] = self.joint_6
        self.Arm.Arm_serial_servo_write6_array(self.down_joint,2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write(6, 90, 2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write(2, 90, 2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        time.sleep(4)
        self.grasp_flag = True
        grasp_done = Bool()
        grasp_done.data = True
        self.pubGraspStatus.publish(grasp_done)
        
     
        

def main(args=None):
    rclpy.init(args=args)
    grasp = GraspNode()
    try:    
        rclpy.spin(grasp)
    except KeyboardInterrupt:
        pass
    finally:
        tag_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()