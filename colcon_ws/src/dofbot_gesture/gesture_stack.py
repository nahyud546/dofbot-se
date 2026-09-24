# !/usr/bin/env python
# coding: utf-8
import cv2
import numpy as np
import random
import time
from Arm_Lib import Arm_Device
import threading

import mediapipe as mp
from dofbot_sorting_3d.media_library import HandDetector  # 添加mediapipe相关导入

class gesture_stack:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.Arm = Arm_Device()
        self.color_name = None
        self.image = None

        self.look_at = [90, 164, 18, 0, 90, 90]
        self.p_top = [90, 80, 50, 50, 270]

        self.p_Yellow = [62, 22, 64, 56, 270]
        self.p_Red = [118, 19, 66, 56, 270]

        self.p_Green = [136, 66, 20, 29, 270]
        self.p_Blue = [44, 66, 20, 28, 270]

        self.p_gray = [90, 52, 37, 29, 270]

        self.p_layer_4 = [90, 76, 40, 17, 270]
        self.p_layer_3 = [90, 65, 46, 15, 270]
        self.p_layer_2 = [90, 65, 25, 36, 270]
        self.p_layer_1 = [90, 48, 37, 30, 270]

        self.p_push_over_1 = [90, 90, 5, 0, 90, 150]
        self.p_push_over_2 = [90, 90, 0, 50, 90, 150]

        self.g_state_arm = 0
        self.started = 0
        self.g_layer = 0

        self.yellow_grabbed = 0
        self.red_grabbed = 0
        self.green_grabbed = 0
        self.blue_grabbed = 0

        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Fist = 0
        
        # 添加mediapipe手势检测器
        self.hand_detector = HandDetector()
        self.cnt = 0
        self.last_sum = 0

    # 推倒积木块
    def push_over_block(self):
        self.Arm.Arm_serial_servo_write6_array(self.p_push_over_1, 1000)
        time.sleep(1.2)
        self.Arm.Arm_serial_servo_write6_array(self.p_push_over_2, 1000)
        time.sleep(1.1)
        self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
        time.sleep(1)
        self.g_layer = 0

    def put_down_block(self, layer):
        if layer == 1:
            self.arm_move(self.p_layer_1, 1000)
            self.arm_clamp_block(0) 
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
        elif layer == 2:
            self.arm_move(self.p_layer_2, 1000)
            self.arm_clamp_block(0)
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
        elif layer == 3:
            self.arm_move(self.p_layer_3, 1000)
            self.arm_clamp_block(0) 
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
        elif layer == 4:
            self.arm_move(self.p_layer_4, 1000)
            time.sleep(.1)
            self.arm_clamp_block(0) 
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)

    def ctrl_arm_move(self, index):
        self.g_layer = self.g_layer + 1
        if self.g_layer >= 5:
            self.g_layer = 1
        self.arm_clamp_block(0)

        if index == 1:
            self.number_action(1)
            self.put_down_block(self.g_layer)
        elif index == 2:
            self.number_action(2)
            self.put_down_block(self.g_layer)
        elif index == 3:
            self.number_action(3)
            self.put_down_block(self.g_layer)
        elif index == 4:
            self.number_action(4)
            self.put_down_block(self.g_layer)
        elif index == 5:
            self.push_over_block()
        self.g_state_arm = 0

    # 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
    def arm_move(self, p, s_time = 500):
        for i in range(5):
            id = i + 1
            if id == 5:
                time.sleep(.1)
                self.Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
            elif id == 1 :
                self.Arm.Arm_serial_servo_write(id, p[i], int(3*s_time/4))
            else:
                self.Arm.Arm_serial_servo_write(id, p[i], int(s_time))
            time.sleep(.01)
        time.sleep(s_time/1000)
    
    # 定义夹积木块函数，enable=1：夹住，=0：松开
    def arm_clamp_block(self, enable):
        if enable == 0:
            self.Arm.Arm_serial_servo_write(6, 60, 400)
        else:
            self.Arm.Arm_serial_servo_write(6, 135, 400)
        time.sleep(.5)

    #数字功能定义
    def number_action(self, index):
        if index == 1:
            # 抓取黄色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Yellow, 1000)
            self.arm_clamp_block(1)
    #         time.sleep(.5)
            self.arm_move(self.p_top, 1000)
        elif index == 2:
            # 抓取红色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Red, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)
        elif index == 3:
            # 抓取绿色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Green, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)
        elif index == 4:
            # 抓取蓝色的积木块
            self.arm_move(self.p_top, 1000)
            self.arm_move(self.p_Blue, 1000)
            self.arm_clamp_block(1)
            self.arm_move(self.p_top, 1000)

    def start_move_arm(self, index):
        # 开启机械臂控制线程
        if self.g_state_arm == 0:
            closeTid = threading.Thread(target = self.ctrl_arm_move, args = [index])
            closeTid.setDaemon(True)
            closeTid.start()
            
            self.g_state_arm = 1

    def reset_state(self):
        self.started = 0

    def Gesture_Action(self, frame):
        if self.started == 0:
            self.Arm.Arm_serial_servo_write6_array(self.look_at, 1000)
            time.sleep(1.2)

            self.Arm.Arm_Buzzer_On(1)
            s_time = 300
            self.Arm.Arm_serial_servo_write(4, 10, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 10, s_time)
            time.sleep(s_time/1000)
            self.Arm.Arm_serial_servo_write(4, 0, s_time)
            time.sleep(s_time/1000)
            self.started = 1
        
        # 使用mediapipe进行手势识别
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        
        if len(lmList) != 0:
            fingers = self.hand_detector.fingersUp(lmList)
            gesture = sum(fingers)  # 计算张开手指的数量作为手势
             
            # 显示手势数量
            cv2.putText(frame, f"Fingers: {gesture}", (450, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 根据手指数量判断手势
            if gesture == 1:  # One
                self.Count_One = self.Count_One + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Fist = 0
                if self.Count_One >= 10:  # 增加识别稳定性阈值
                    if self.yellow_grabbed == 0:
                        print('Recongnize_num:1')
                        self.start_move_arm(1)
                        self.yellow_grabbed = 1
                        # 重置计数器
                        self.Count_One = 0
            elif gesture == 2:  # Two
                self.Count_Two = self.Count_Two + 1
                self.Count_One = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Fist = 0
                if self.Count_Two >= 10:
                    if self.red_grabbed == 0:
                        print('Recongnize_num:2')
                        self.start_move_arm(2)
                        self.red_grabbed = 1
                        self.Count_Two = 0
            elif gesture == 3:  # Three
                self.Count_Three = self.Count_Three + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Four = 0
                self.Count_Fist = 0
                if self.Count_Three >= 10:
                    if self.green_grabbed == 0:
                        print('Recongnize_num:2')
                        self.start_move_arm(3)
                        self.green_grabbed = 1
                        self.Count_Three = 0
            elif gesture == 4:  # Four
                self.Count_Four = self.Count_Four + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Fist = 0
                if self.Count_Four >= 10:
                    if self.blue_grabbed == 0:
                        print('Recongnize_num:4')
                        self.start_move_arm(4)
                        self.blue_grabbed = 1
                        self.Count_Four = 0
            elif gesture == 0:  # Fist (握拳，没有手指张开)
                self.Count_Fist = self.Count_Fist + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                if self.Count_Fist >= 10:
                    print('Recongnize_num:0')
                    self.Count_One = 0
                    self.Count_Two = 0
                    self.Count_Three = 0
                    self.Count_Four = 0
                    self.Count_Fist = 0
                    self.yellow_grabbed = 0
                    self.red_grabbed = 0
                    self.green_grabbed = 0
                    self.blue_grabbed = 0
                    self.start_move_arm(5)
            else:
                # 如果不是预设的手势，不做操作
                pass
        else:
            # 没有检测到手时，显示提示信息
            cv2.putText(frame, "No Hand Detected", (300, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return frame


    def start_gesture(self, img):
        '''
        颜色跟随控制函数
        :param img: 输入图像
        :return: 输出处理后的图像
        '''
        self.yellow_grabbed = 0
        self.red_grabbed = 0
        self.green_grabbed = 0
        self.blue_grabbed = 0
        self.image = self.Gesture_Action(img)
        
        return self.image