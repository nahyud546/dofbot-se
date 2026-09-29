import sys
import os
import cv2 as cv
import threading
from time import sleep
from speech_garbage import speech_garbage
import Arm_Lib
from dofbot_utils.dofbot_config import *
from dofbot_utils.fps import FPS
fps = FPS()

XYT_CONFIG_PATH = os.environ.get("XYT_CONFIG", os.path.join(os.environ.get("ROBOT_ARM_ROOT", str(__import__("pathlib").Path(__file__).resolve().parent)), "XYT_config.txt"))
xy = []
# 创建获取目标实例
single_garbage = speech_garbage()
Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")  # 机械臂实例
# 初始化模式
model = "General"

def load_config():
    """加载XYT配置文件"""
    # 加载XYT配置
    try:
        global xy
        xy,_= read_XYT(XYT_CONFIG_PATH)
        print(xy)
    except Exception as e:
        print(f"Read XYT Error: {e}")

def camera():
    global single_garbage,Arm
    load_config()
    #机械臂复位位置
    joints_0 = [91, 129, 0, 0, 90, 30]
    Arm.Arm_serial_servo_write6_array(joints_0, 500)
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2', cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            # 读取相机的每一帧
            _, img = capture.read()
            fps.update_fps()
            img = single_garbage.single_garbage_run(img)
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            fps.show_fps(img)
            cv.imshow("img", img)
            key = cv.waitKey(1)            
        except KeyboardInterrupt:
            capture.release()
            del single_garbage
            del Arm
            cv.destroyAllWindows()
threading.Thread(target=camera, ).start()
