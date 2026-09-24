import time
import smbus
import Arm_Lib
from Speech_Lib import Speech
import os
from dofbot_utils.robot_controller import Robot_Controller
from dofbot_utils.fps import FPS
import cv2
import numpy as np
import threading
import inspect
import ctypes

# 初始化硬件模块
robot = Robot_Controller()
robot.move_init_pose()
os.system("mpg123 /home/yahboom/speech_music/0.mp3")

fps = FPS()
mySpeech = Speech()

# 摄像头初始化
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3, 640)       # 宽度
cap.set(4, 480)       # 高度
cap.set(5, 30)        # 帧率
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

# 线程控制标志
is_running = True

# 线程相关函数
def _async_raise(tid, exctype):
    """触发线程异常，用于停止线程"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("无效的线程ID")
    elif res != 1:
        # 重置异常状态
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        
def stop_thread(thread):
    """停止指定线程"""
    if thread.is_alive():
        _async_raise(thread.ident, SystemExit)

def get_color(img):
    """
    识别指定区域的颜色
    :param img: 输入图像
    :return: 处理后的图像，颜色名称字典
    """
    color_name = {'name': 'unknown'}
    img = cv2.resize(img, (640, 480))
    
    # 转HSV色彩空间
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 绘制识别区域矩形
    cv2.rectangle(img, (280, 180), (360, 260), (0, 255, 0), 2)
    
    H = []

    for j in range(180, 260):         
        for i in range(280, 360):    
            H.append(HSV[j, i][0])
    
    if not H:  
        return img, color_name
    
    # 计算H值的最大最小值
    H_min = min(H)
    H_max = max(H)
    
    # 颜色判断（HSV色相范围）
    if (H_min >= 0 and H_max <= 18) or (H_min >= 156 and H_max <= 180):
        color_name['name'] = 'red'      # 红色
    elif H_min >= 21 and H_max <= 28:
        color_name['name'] = 'yellow'   # 黄色
    elif H_min >= 35 and H_max <= 78:
        color_name['name'] = 'green'    # 绿色
    elif H_min >= 100 and H_max <= 124:
        color_name['name'] = 'blue'     # 蓝色
    
    return img, color_name

def Color_Recongnize():
    """颜色识别"""
    global is_running
    while is_running:
        # 读取摄像头帧
        ret, frame = cap.read()
        if not ret:  # 摄像头读取失败时跳过
            time.sleep(0.01)
            continue
        
        # 颜色识别
        frame, color_name = get_color(frame)
        # 语音识别交互
        try:
            if color_name['name'] != 'unknown':
                print(f"color: {color_name['name']}")
                result = mySpeech.speech_read()
                time.sleep(0.01)
                if result == 60:
                    color_speech_map = {
                        'yellow': 64,
                        'green': 63,
                        'blue': 62,
                        'red': 61,
                    }
                    speech_code = color_speech_map.get(color_name['name'])
                    if speech_code:
                        mySpeech.void_write(speech_code)
                        time.sleep(0.01)
        except Exception as e:
            print(f"语音交互异常: {e}")
            time.sleep(0.01)
        
        # 显示图像
        cv2.imshow("res_image", frame)
        
        # 按键检测：按q退出
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            is_running = False
            break

# 启动线程
thread1 = threading.Thread(target=Color_Recongnize)
thread1.setDaemon(True)
thread1.start()

# 主程序循环
try:
    while is_running:
        time.sleep(0.5)
except KeyboardInterrupt:
    # 捕获Ctrl+C中断
    print("程序被用户终止！")
finally:
    is_running = False
    del Arm
    del mySpeech
    # 停止线程
    stop_thread(thread1)
    # 释放摄像头
    cap.release()
    # 销毁所有OpenCV窗口
    cv2.destroyAllWindows()
