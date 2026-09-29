import os
import sys
from pathlib import Path
_ROOT = Path(os.environ.get("ROBOT_ARM_ROOT", Path(__file__).resolve().parents[3]))
for _p in [
    _ROOT / "workspaces/dofbot_ws/src/dofbot_face_follow/scripts",
    _ROOT / "workspaces/dofbot_ws/src/dofbot_color_follow",
    _ROOT / "workspaces/dofbot_ws/src/dofbot_apriltag/scripts",
    # legacy fallbacks (pre-restructure / Yahboom gốc)
    Path("/home/jloy/Desktop/robot-arm/dofbot_ws/src/dofbot_face_follow/scripts"),
    Path("/home/jloy/Desktop/robot-arm/dofbot_ws/src/dofbot_color_follow"),
    Path("/home/jloy/Desktop/robot-arm/dofbot_ws/src/dofbot_apriltag/scripts"),
]:
    if str(_p) not in sys.path:
        sys.path.append(str(_p))
import time
import cv2 as cv
import cv2 
import numpy as np
import threading
import random
from time import sleep
import math
import ipywidgets as widgets
from IPython.display import display
from color_follow import color_follow
from face_follow import Face_Follow
from apriltag_follow import Apriltag_Follow  
from apriltag_identify import ApriltagIdentify 
from dofbot_utils.dofbot_config import *
from Speech_Lib import Speech
import Arm_Lib


mySpeech = Speech()
follow = color_follow()
follow2 = Face_Follow()
apriltag_follow = Apriltag_Follow()  # AprilTag跟踪实例
apriltag_identify = ApriltagIdentify()  # AprilTag识别实例

# 初始化模式
model = 'General'
# 初始化HSV_learning值
HSV_learning = ()
# 初始化HSV值
color_hsv = {"red": ((170, 124, 134), (229, 242, 255)),
             "green": ((54, 113, 64), (75, 255, 255)),
             "blue": ((102, 150, 124), (124, 253, 255)),
             "yellow": ((22, 125, 130), (47, 255, 255))}
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
# HSV参数路径
def _hsv_default():
    for c in [
        _ROOT / "workspaces/legacy/colcon_ws/src/dofbot_color_follow/scripts/HSV_config.txt",
        _ROOT / "workspaces/dofbot_ws/src/dofbot_color_follow/HSV_config.txt",
        Path("/home/jloy/Desktop/robot-arm/colcon_ws/src/dofbot_color_follow/scripts/HSV_config.txt"),
    ]:
        if c.exists():
            return str(c)
    return str(_ROOT / "workspaces/legacy/colcon_ws/src/dofbot_color_follow/scripts/HSV_config.txt")
HSV_path = os.environ.get("HSV_CONFIG", _hsv_default())
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error !!!")



# 创建AprilTag跟踪实例
apriltag_follow = Apriltag_Follow()

def apriltag_follow_function(img):
    """
    AprilTag跟随
    """
    global apriltag_follow, apriltag_identify
    
    result_image, msg = apriltag_identify.getApriltagPosition(img)
    
    apriltag_follow.follow_function(msg if len(msg) > 0 else {})
    
    return result_image


def camera():
    global HSV_learning,model
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2', cv.CAP_V4L2)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)  #设置帧率
    
    Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
    joints_0 = [90, 135, 20, 25, 90, 30]
    Arm.Arm_serial_servo_write6_array(joints_0, 1000)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            global mySpeech
            time.sleep(0.1)
            # 读取相机的每一帧
            _, img = capture.read()
            key = cv2.waitKey(10)
            result = mySpeech.speech_read()
            if result == 105:
                model = "apriltag_follow"
                mySpeech.void_write(45)#等待当前语句播报结束
                time.sleep(0.1)
            if result == 73:
                choose_color = 'red'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1) 
            elif result == 72:
                choose_color = 'yellow'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)
            elif result == 74:
                choose_color = 'green'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)  
            elif result == 75:
                choose_color = 'blue'
                model = 'color_follow'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)     
            elif result == 71:
                model = 'follow2'
                mySpeech.void_write(71)#等待当前语句播报结束
                time.sleep(0.1)   
            elif result == 76:
                model = 'General'
                mySpeech.void_write(76)#等待当前语句播报结束
                time.sleep(0.1)
            
            if model == 'color_follow':
                img = follow.follow_function(img, color_hsv[choose_color])
                # 添加文字
                cv.putText(img, choose_color, (int(img.shape[0] / 2), 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
            if model == 'follow2':
                img, pos = follow2.follow_function(img)
            if model == "apriltag_follow":
                img = apriltag_follow_function(img)
            cv2.imshow("res_image", img)
        except KeyboardInterrupt:
            capture.release()
            del mySpeech
            del Arm
            cv2.destroyAllWindows()
            break
        except Exception as e:
            print("error:", e)
            break
threading.Thread(target=camera, ).start()