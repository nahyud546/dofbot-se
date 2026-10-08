import re
import rclpy
from rclpy.node import Node
from rclpy.client import Client
import Arm_Lib
import cv2 as cv
import os
import numpy as np
from time import sleep
from identify_grap import identify_grap
from dofbot_interface.srv import Kinemarics 

rclpy.init()
class identify_GetTarget(Node):
    def __init__(self):
        # ROS2节点初始化
        super().__init__('dofbot_identify')
        self.grap = identify_grap()
        # 机械臂识别位置调节
        self.xy = [90, 130]
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
        # 创建ROS2服务客户端
        self.client: Client = self.create_client(Kinemarics, "dofbot_kinemarics")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service dofbot_kinemarics not available, waiting again...')

    def select_color(self, image, color_hsv=None, color_list=None):
        '''
        :param image: 输入图像
        :param color_hsv: HSV范围阈值
        :param color_list: 要识别的颜色列表
        :return: 处理后图像, {颜色:位置}, 各颜色识别结果
        '''
        self.image = cv.resize(image, (640, 480))
        msg = {}
        coo1, coo2, coo3, coo4 = '', '', '', ''
        
        # 默认参数处理
        if color_list is None or len(color_list) == 0:
            return self.image, msg, coo1, coo2, coo3, coo4
        
       
        target_colors = []
       
        if isinstance(color_list, dict):
            target_colors = list(color_list.values())
        else:
            target_colors = color_list
        
        target_colors = list(set(target_colors))
        
        color_idx = 0  
        for color in target_colors:
            if color not in color_hsv:
                continue
            self.color_name = color
            # 获取该颜色的所有有效位置
            positions = self.get_Sqaure(color_hsv[color])
            if positions:
                # 取第一个有效位置
                msg[color] = positions[0]
                
                if color_idx == 0:
                    coo1 = color
                elif color_idx == 1:
                    coo2 = color
                elif color_idx == 2:
                    coo3 = color
                elif color_idx == 3:
                    coo4 = color
                color_idx += 1
        
        return self.image, msg, coo1, coo2, coo3, coo4

    def get_Sqaure(self, color_hsv):
        positions = []
        if isinstance(color_hsv, list) and len(color_hsv) == 2:
            mask1 = cv.inRange(cv.cvtColor(self.image, cv.COLOR_BGR2HSV), 
                            np.array(color_hsv[0][0]), np.array(color_hsv[0][1]))
            mask2 = cv.inRange(cv.cvtColor(self.image, cv.COLOR_BGR2HSV), 
                            np.array(color_hsv[1][0]), np.array(color_hsv[1][1]))
            img = cv.bitwise_or(mask1, mask2)  
        else:
            lowerb = np.array(color_hsv[0])
            upperb = np.array(color_hsv[1])
            img = cv.inRange(cv.cvtColor(self.image, cv.COLOR_BGR2HSV), lowerb, upperb)
        
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        img = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)  # 开运算去噪
        
        # 查找轮廓
        contours, _ = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv.contourArea(cnt)
            if 5000 < area < 20000:  
                x, y, w, h = cv.boundingRect(cnt)
                point_x = float(x + w / 2)
                point_y = float(y + h / 2)
                cv.rectangle(self.image, (x, y), (x+w, y+h), (0,255,0), 2)
                cv.putText(self.image, self.color_name, (x-10, y-10), 
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
                #这里根据位置自行调节，和机械臂初始位姿有关
                #this is the key part for the position adjustment, you can adjust the parameters according to your needs and the initial pose of the robotic arm
                (a, b) = (round(((point_x - 320) / 4500), 5), round(((480 - point_y) / 3000) * 0.8+0.15, 5))
                positions.append((b, a))
        return positions

    def _get_color_mask(self, lowerb, upperb):
        '''
        辅助函数：获取颜色掩码
        '''
        HSV_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        img = cv.inRange(HSV_img, lowerb, upperb)
        # 降噪：开运算+闭运算
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        img = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)
        return img

    def target_run(self, msg, xy=None):
        '''
        '''
        if xy is not None:
            self.xy = xy
        move_status = 0
        for i in msg.values():
            if i is not None:
                move_status = 1
        if move_status == 1:
            self.arm.Arm_Buzzer_On(1)
            sleep(0.5)
        for name, pos in msg.items():
            try:
                # 调用反解服务
                joints = self.server_joint(pos)
                if joints:
                    self.grap.identify_move(str(name), joints)
                else:
                    self.get_logger().info(f"Failed to get joints for {name}")
            except Exception as e:
                self.get_logger().info(f"square_pos empty: {e}")  
        if move_status == 1:
            # 架起位置
            joints_uu = [90, 80, 50, 50, 265, 30]
            self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
            sleep(1)
            # 初始位置
            joints_0 = [self.xy[0], self.xy[1], 0, 0, 90, 30]
            self.arm.Arm_serial_servo_write6_array(joints_0, 500)
            sleep(0.5)

    def server_joint(self, posxy):
        '''
        '''
        request = Kinemarics.Request()
        request.tar_x = posxy[0] +0.01
        request.tar_y = posxy[1] -0.02
        request.tar_z = 0.06
        request.pitch = 1.04
        request.kin_name = "ik"  

        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is not None:
                # 关节角度计算
                joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                joints[0] = 180-response.joint1
                joints[1] = 180-response.joint2
                joints[2] = 180-response.joint3
                joints[3] = 180-response.joint4
                joints[4] = 180-response.joint5
                # 越界调整
                if joints[2] < 0:
                    joints[1] += joints[2] * 3 / 5
                    joints[3] += joints[2] * 3 / 5
                    joints[2] = 0
                return joints
        except Exception as e:
            self.get_logger().info(f"arg error: {e}")  
        return None

def main(args=None):
    rclpy.init(args=args)
    # 创建识别节点实例
    identify_node = identify_GetTarget()
    try:
        # 保持节点运行
        rclpy.spin(identify_node)
    except KeyboardInterrupt:
        identify_node.get_logger().info("Node interrupted by user")
    finally:
        # 销毁节点
        identify_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()