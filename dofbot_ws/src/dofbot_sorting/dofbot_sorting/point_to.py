#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Bool,Int16
import time
import math
from dofbot_interface.msg import *       # 需确认ROS2消息包名是否一致
from dofbot_interface.srv import Kinemarics
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from ament_index_python import get_package_share_directory
import yaml
import os
from Arm_Lib import Arm_Device


class TagGraspNode(Node):
    def __init__(self):
        super().__init__('color_grap')
        self.Arm = Arm_Device()
        self.sub = self.create_subscription(AprilTagInfo,'PosInfo',self.pos_callback,1)
        self.sub_joint5 = self.create_subscription(Int16,"set_joint5",self.get_joint5_callback,1)
        self.pubGraspStatus = self.create_publisher(Bool, 'grasp_done',1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.grasp_flag = True
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.down_joint = [10.0, 70.0, 34.0, 16.0, 90.0,140.0]
        self.gripper_joint = 90.0
        self.cur_tagId = 0
        self.joint_5 = 90
        print("Init Done")  


    def get_joint5_callback(self,msg):
        self.joint_5 = msg.data

    def pos_callback(self,msg):
        pos_x = msg.x
        pos_y = msg.y
        pos_z = msg.z
        self.cur_tagId = msg.id
        if self.grasp_flag == True :
            print("Take it now.")
            self.grasp_flag = False
            threading.Thread(target=self.grasp, args=(pos_x,pos_y,pos_z)).start()

    def grasp(self,pos_x,pos_y,pos_z):
        print("------------------------------------------------")
        request = Kinemarics.Request() 
        request.tar_x = pos_x + 0.02
        request.tar_y = pos_y - 0.01
        request.tar_z = 0.08
        request.kin_name = "ik"
        request.pitch = 1.04   
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            response = future.result()
            print("calcutelate_response: ",response)
            joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
            joints[0] = response.joint1 #response.joint1
            joints[1] = 180 - response.joint2 
            joints[2] = 180 - response.joint3 
            
            if response.joint4>90:
                joints[3] = 90 
            else:
                joints[3] = response.joint4 
            joints[3] = 180 - response.joint4
            tmp_joints = 180 - joints[0]
            print("self.joint_5: ",self.joint_5)
            if self.joint_5<0:
                joints[4] = abs(self.joint_5)
                if abs(self.joint_5)<90:
                    joints[4] =180-tmp_joints+joints[4] -90 -180
                else:
                    joints[4]= joints[4] - tmp_joints + 90
                if joints[4] >135:
                    joints[4] =  joints[4] -90
                elif joints[4]<45:
                    joints[4] =  joints[4] + 180      
            if self.joint_5>0:
                if self.joint_5<90:
                    joints[4] = joints[4]   - tmp_joints
                else :
                    joints[4] = joints[4] - (tmp_joints)
                if joints[4] >135:
                    joints[4] =  joints[4] -90
                elif joints[4]<45:
                    joints[4] =  joints[4] + 180
            joints[5] = 150
            self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
            time.sleep(3.5)
            #self.move()

        except Exception:
           pass

    def move(self):
        self.Arm.Arm_serial_servo_write(6, 142, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.5)
        print("self.cur_tagId",self.cur_tagId)
        if self.cur_tagId == 1:
            self.down_joint = [155.0, 35, 70, 5, 60.0,142]
        elif self.cur_tagId == 2:
            self.down_joint = [180.0, 35, 60, 0, 90.0,142]
        elif self.cur_tagId == 3:
            self.down_joint = [28.0, 30, 70, 2, 90.0,142]
        elif self.cur_tagId == 4:
            self.down_joint = [0.0, 43, 48, 6, 90.0,142]
        elif self.cur_tagId == 0:
            self.down_joint = [155.0, 35, 70, 5, 60.0,142]
        print("self.down_joint",self.down_joint)
        self.Arm.Arm_serial_servo_write6_array(self.down_joint,2000)
        time.sleep(3.0)
        self.Arm.Arm_serial_servo_write(6, 90, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write(2, 90, 2000)
        time.sleep(2.5)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        time.sleep(5)
        self.grasp_flag = True
        grasp_done = Bool()
        grasp_done.data = True
        self.pubGraspStatus.publish(grasp_done)
        

    def pubTargetArm(self, joints, id=6, angle=180.0, runtime=2000):
        print(joints)
        self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],2000)
     
        

def main(args=None):
    rclpy.init(args=args)
    tag_grasp = TagGraspNode()
    try:    
        rclpy.spin(tag_grasp)
    except KeyboardInterrupt:
        pass
    finally:
        tag_grasp.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
