#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Bool,Int16
import time
from dofbot_interface.msg import *       # 需确认ROS2消息包名是否一致
from dofbot_interface.srv import Kinemarics
import threading
import os
from Arm_Lib import Arm_Device

class TagGraspNode(Node):
    def __init__(self):
        super().__init__('color_grap')
        self.Arm = Arm_Device()
        self.sub = self.create_subscription(AprilTagInfo,'PosInfo',self.pos_callback,1)
        self.sub_joint5 = self.create_subscription(Int16,"set_joint5",self.get_joint5_callback,1)
        self.sub_step = self.create_subscription(Int16,"step",self.get_step_callback,1)
        self.pubGraspStatus = self.create_publisher(Bool, 'grasp_done',1)
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.grasp_flag = True
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 30.0]
        self.down_joint = [10.0, 70.0, 34.0, 16.0, 90.0,140.0]
        self.gripper_joint = 90.0
        self.cur_tagId = 0
        self.joint_5 = 90
        self.set_joint5 = 90
        self.step = 1.0
        
        # 等待服务最多10秒，避免无限等待
        timeout_count = 0
        max_timeout = 10  # 10秒超时
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting service...")
            timeout_count += 1
            if timeout_count >= max_timeout:
                self.get_logger().error("Service 'dofbot_kinemarics' not available after %d seconds" % max_timeout)
                break
        
        print("Init Done")

    def get_step_callback(self,msg):
        self.step = msg.data
        print("self.step: ",self.step)
        


    def get_joint5_callback(self,msg):
        self.joint_5 = msg.data

    def pos_callback(self,msg):
        pos_x = msg.x 
        pos_y = msg.y 
        pos_z = msg.z
        self.cur_tagId = msg.id
        print("msg: ",msg)
        if self.grasp_flag == True :
            print("Take it now.")
            self.grasp_flag = False
            threading.Thread(target=self.grasp, args=(pos_x,pos_y,pos_z)).start()

    def grasp(self,pos_x,pos_y,pos_z):
        print("------------------------------------------------")
        request = Kinemarics.Request()
        request.tar_x = pos_x -0.01
        request.tar_y = pos_y +0.01
        request.tar_z = pos_z * self.step + 0.03
        request.kin_name = "ik"
        request.pitch = 1.04   
        try:
            # 检查客户端是否可用
            if not self.client.service_is_ready():
                self.get_logger().warn("Service not ready, skipping this grasp attempt")
                self.grasp_flag = True  # 重置标志，允许下次尝试
                return
                
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0) 
            
            # 检查future是否完成
            if future.done():
                response = future.result()
                if response is not None:
                    print("calculate_response: ",response)
                    joints = [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
                    joints[0] = response.joint1 #response.joint1
                    joints[1] = 180 - response.joint2 
                    joints[2] = 180 - response.joint3 
                    joints[3] = 180 - response.joint4 
                    print("self.joint_5: ",self.joint_5)
                    target_joint5 = self.joint_5
                    joint5 = 90  # 默认值
                    if target_joint5>0 and target_joint5 < 45:
                        joint5 = 90 - target_joint5                 
                    elif target_joint5>45 and target_joint5<135:
                        joint5 = target_joint5
                    elif target_joint5>135:
                        joint5 = target_joint5 - 90                
                    elif target_joint5>-45 and target_joint5<0:
                        joint5 = 90 + abs(target_joint5)                  
                    elif target_joint5<-45 and target_joint5<-135:
                        joint5 = abs(target_joint5)               
                    elif target_joint5<-135:
                        joint5 = abs(target_joint5) - 90
                    self.set_joint5 = joint5
                    joints[5] = 30
                    self.Arm.Arm_serial_servo_write6(joints[0],joints[1],joints[2],joints[3],90,joints[5],2000)
                    time.sleep(3.5)
                    self.move()
                else:
                    self.get_logger().error("Service call failed, response is None")
                    self.grasp_flag = True  # 重置标志，允许下次尝试
            else:
                self.get_logger().error("Service call timed out")
                self.grasp_flag = True  # 重置标志，允许下次尝试

        except Exception as e:
            self.get_logger().error(f"Exception in grasp: {str(e)}")
            self.grasp_flag = True  # 重置标志，允许下次尝试

    def move(self):
        self.Arm.Arm_serial_servo_write(5, self.set_joint5, 2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write(6, 140, 2000)
        time.sleep(2.0)
        self.Arm.Arm_serial_servo_write(2, 120, 2000)
        time.sleep(2.0)
        print("self.cur_tagId",self.cur_tagId)
        if self.cur_tagId == 1:
            self.down_joint = [30, 70, 0, 54, 265, 142]
        elif self.cur_tagId == 2:
            self.down_joint = [50, 70, 7, 58, 265, 142]
        elif self.cur_tagId == 3:
            self.down_joint = [150, 70, 7, 52, 265, 142]
        elif self.cur_tagId == 4:
            self.down_joint = [135, 70, 7, 54, 265, 142]
        elif self.cur_tagId == 0:
            self.down_joint = [155.0, 45, 70, 5, 60.0,142]
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
        self.step = 1.0

     
        

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