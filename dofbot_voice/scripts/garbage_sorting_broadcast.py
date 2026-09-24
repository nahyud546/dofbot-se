#!/usr/bin/env python
# coding: utf-8
import sys
import Arm_Lib
import os
import cv2 as cv
import threading
from time import sleep
from speech_garbage_identify import speech_garbage_identify

XYT_CONFIG_PATH = "XYT_config.txt"
Arm = Arm_Lib.Arm_Device()  # 机械臂实例

single_garbage = speech_garbage_identify()
# 初始化模式
model = "General"
single_garbage.init_robot_joint()

def camera():
    global single_garbage,Arm
    #机械臂复位位置
    joints_0 = [90, 129, 0, 0, 90, 30]
    Arm.Arm_serial_servo_write6_array(joints_0, 500)
    # 打开摄像头
    capture = cv.VideoCapture(0, cv.CAP_V4L2)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            # 读取相机的每一帧 
            _, img = capture.read()
            # 统一图像大小
            img = cv.resize(img, (640, 480))
            img = single_garbage.single_garbage_run(img)
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            cv.imshow("img", img)
            key = cv.waitKey(1)            
        except KeyboardInterrupt:capture.release()
            
threading.Thread(target=camera, ).start()
