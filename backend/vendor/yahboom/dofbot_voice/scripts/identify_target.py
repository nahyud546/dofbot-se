#!/usr/bin/env python3
# coding: utf-8
import rclpy
from rclpy.node import Node
import Arm_Lib
import cv2 as cv
import numpy as np
from time import sleep
from identify_grap import identify_grap
from dofbot_interface.srv import Kinemarics 
import math
rclpy.init()
class identify_GetTarget(Node):
    def __init__(self):
        super().__init__('dofbot_identify')
        self.grap = identify_grap()

        self.xy = [90, 135]  
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
        self.client = self.create_client(Kinemarics, "dofbot_kinemarics")
        # 等待服务启动
        self.get_logger().info("Waiting for dofbot_kinemarics service...")
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service not available, waiting again...')
        self.get_logger().info("Service is available!")

    def target_run(self, msg, xy=None):
        '''
        抓取函数
        '''
        if xy is not None:
            self.xy = xy
        num = 1  
        move_status = 0
        for i in msg.values():
            if i is not None:
                move_status = 1
        if move_status == 1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)  
        for name, pos in msg.items():
            joints = self.server_joint(pos)
            self.grap.identify_move(str(name), joints)
            num += 1

        if move_status == 1:
            joints_uu = [90, 80, 50, 50, 265, 30]  
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1) 
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30] 
            self.arm.Arm_serial_servo_write6_array(joints_0, 500) 
            sleep(0.5)  

    def select_color(self, image, color_hsv, color_list):
        '''
        选择识别颜色
        '''
        self.image = cv.resize(image, (640, 480))  
        msg = {}
        if len(color_list) == 0:
            return self.image, msg
        if '4' in color_list:
            self.color_name = color_list['4']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos is not None: msg[self.color_name] = pos
        if '3' in color_list:
            self.color_name = color_list['3']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos is not None: msg[self.color_name] = pos
        if '2' in color_list:
            self.color_name = color_list['2']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos is not None: msg[self.color_name] = pos
        if '1' in color_list:
            self.color_name = color_list['1']
            pos = self.get_Sqaure(color_hsv[self.color_name])
            if pos is not None: msg[self.color_name] = pos
        return self.image, msg

    def get_Sqaure(self, color_hsv):
        '''
        颜色识别
        '''
        (lowerb, upperb) = color_hsv
        # 复制原始图像
        mask = self.image.copy()
        # 将图像转换为HSV
        HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        img = cv.inRange(HSV_img, lowerb, upperb)
        mask[img == 0] = [0, 0, 0]
        # 获取结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = find_contours[1] if len(find_contours) == 3 else find_contours[0]
        for cnt in contours:
            x, y, w, h = cv.boundingRect(cnt)
            area = cv.contourArea(cnt)
            if area > 1000:
                point_x = float(x + w / 2)
                point_y = float(y + h / 2)
                cv.rectangle(self.image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(self.image, (int(point_x), int(point_y)), 5, (0, 0, 255), -1)
                cv.putText(self.image, self.color_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                # ========== 坐标计算逻辑 ==========
                a = round(((point_x - 320) / 4000), 5) 
                b = round(((480 - point_y) / 3000) * 0.8 + 0.19, 5)
                return (a, b)
        return None  # 保留容错返回

    def server_joint(self, posxy):
        '''
        服务调用
        '''
        if len(posxy) < 2:
            self.get_logger().error("posxy must have at least x and y coordinates!")
            return None
        request = Kinemarics.Request()
        request.tar_x = posxy[0]
        request.tar_y = posxy[1]
        request.kin_name = "ik"
        self.get_logger().info(f"Calling service with pos: {posxy[0]}, {posxy[1]}")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            joints = [0.0, 0.0, 0.0, 0.0, 0.0]
            joints[0] = response.joint1
            joints[1] = response.joint2
            joints[2] = response.joint3
            joints[3] = response.joint4
            joints[4] = response.joint5 
            if joints[2] < 0:
                joints[1] += joints[2] * 3 / 5
                joints[3] += joints[2] * 3 / 5
                joints[2] = 0
            self.get_logger().info(f"Received joints: {joints}")
            return joints
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    identify_node = identify_GetTarget()
    try:
        rclpy.spin(identify_node)
    except KeyboardInterrupt:
        identify_node.get_logger().info("Node interrupted by user!")
    finally:
        identify_node.destroy_node()
        rclpy.shutdown()

# 程序入口
if __name__ == '__main__':
    main()