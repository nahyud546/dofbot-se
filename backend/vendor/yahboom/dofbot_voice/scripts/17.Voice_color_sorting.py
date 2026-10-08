#!/usr/bin/env python
# coding: utf-8

# # 语音识别颜色分拣玩法

# In[1]:


import os
import sys
_VROOT = os.environ.get("ROBOT_ARM_ROOT", "/home/jloy/Desktop/robot-arm")
for _p in [
    os.path.join(_VROOT, "vendor/yahboom/dofbot_voice/scripts"),
    os.path.join(_VROOT, "workspaces/dofbot_ws/src/dofbot_color_stacking/scripts"),
    '/home/jloy/Desktop/robot-arm/dofbot_pro/dofbot_color_sorting/scripts',  # legacy, không còn tồn tại
]:
    if _p not in sys.path:
        sys.path.append(_p)


# ### 导入头文件

# In[2]:


#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
import threading
from time import sleep

import ipywidgets as widgets
from IPython.display import display
from speech_identify_target import identify_GetTarget
from dofbot_utils.fps import FPS
from dofbot_utils.dofbot_config import *
from Speech_Lib import Speech
mySpeech = Speech()


# ### 创建实例,初始化参数

# In[3]:


## 创建获取目标实例
target      = identify_GetTarget()
# 创建相机标定实例
calibration = Arm_Calibration()
# 初始化一些参数
num=0
dp    = []
xy=[90,106]
msg   = {}
threshold = 116
model = "General"
color_list = {}
# 初始化HSV值
color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
              "green" : ((35, 43, 46), (77, 255, 255)),
              "blue"  : ((100, 43, 46), (124, 255, 255)),
              "yellow": ((21, 43, 46), (28, 255, 255))}
HSV_path="./HSV_config.txt"
# XYT参数路径
XYT_path="./XYT_config.txt"
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error !!!")
try: xy, threshold = read_XYT(XYT_path)
except Exception: print("Read XYT_config Error !!!")


# In[4]:


import Arm_Lib
# 创建机械臂驱动实例
import time
Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
joints_0 = [xy[0], xy[1], 0, 0, 90, 30]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)
fps = FPS()


# ### 创建控件

# In[5]:


button_layout      = widgets.Layout(width='320px', height='60px', align_self='center')
output = widgets.Output()
# 调整滑杆
joint1_slider      = widgets.IntSlider(description='joint1 :'   ,    value=xy[0]     , min=70 , max=110, step=1, orientation='horizontal')
joint2_slider      = widgets.IntSlider(description='joint2 :'   ,    value=xy[1]     , min=90, max=150, step=1, orientation='horizontal')
threshold_slider   = widgets.IntSlider(description='threshold :',    value=threshold , min=0  , max=255, step=1, orientation='horizontal')

# 进入标定模式
calibration_model  = widgets.Button(description='calibration_model',  button_style='primary', layout=button_layout)
calibration_ok     = widgets.Button(description='calibration_ok',     button_style='success', layout=button_layout)
calibration_cancel = widgets.Button(description='calibration_cancel', button_style='danger', layout=button_layout)
# 选择抓取颜色
color_list_one     = widgets.Dropdown(options=['red', 'green', 'blue', 'yellow', 'none'], value='none', disabled=False)
color_list_two     = widgets.Dropdown(options=['red', 'green', 'blue', 'yellow', 'none'], value='none', disabled=False)
color_list_three   = widgets.Dropdown(options=['red', 'green', 'blue', 'yellow', 'none'], value='none', disabled=False)
color_list_four    = widgets.Dropdown(options=['red', 'green', 'blue', 'yellow', 'none'], value='none', disabled=False)
# 目标检测抓取
target_detection   = widgets.Button(description='target_detection', button_style='info', layout=button_layout)
reset_color_list   = widgets.Button(description='reset_color_list', button_style='info', layout=button_layout)
grap = widgets.Button(description='grap', button_style='success', layout=button_layout)
# 退出
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
color_down = widgets.HBox([exit_button], layout=widgets.Layout(align_self='center'));
color_img = widgets.VBox([imgbox, color_down], layout=widgets.Layout(align_self='center'));
color_identify = widgets.VBox(
    [joint1_slider, joint2_slider, threshold_slider, calibration_model, calibration_ok, calibration_cancel,
     color_list_one, color_list_two, color_list_three, color_list_four, target_detection, grap],
    layout=widgets.Layout(align_self='center'));
controls_box = widgets.HBox([color_img], layout=widgets.Layout(align_self='center'))


# ### 标定回调

# In[6]:


def calibration_model_Callback(value):
    global model
    model = 'Calibration'
    with output: print(model)
def calibration_OK_Callback(value):
    global model
    model = 'calibration_OK'
    with output: print(model)
def calibration_cancel_Callback(value):
    global model
    model = 'calibration_Cancel'
    with output: print(model)
calibration_model.on_click(calibration_model_Callback)
calibration_ok.on_click(calibration_OK_Callback)
calibration_cancel.on_click(calibration_cancel_Callback)


# ### 颜色选择序列

# In[7]:


# 选择颜色
def color_list_one_Callback(value):
    global model,color_list
    model="General"
    if not isinstance(value['new'],str):return
    if value['new'] == "none":
        if '1' in color_list:del color_list['1']
    elif value['new'] == "red":
        color_list['1'] = "red"
    elif value['new']== "green":
        color_list['1'] = "green"
    elif value['new'] == "blue":
        color_list['1'] = "blue"
    elif value['new'] == "yellow":
        color_list['1'] = "yellow"
    with output:
        print("color_list_three_Callback clicked.",color_list)
def color_list_two_Callback(value):
    global model,color_list
    model="General"
    if not isinstance(value['new'],str):return
    if value['new'] == "none":
        if '2' in color_list:del color_list['2']
    elif value['new'] == "red":
        color_list['2'] = "red"
    elif value['new'] == "green":
        color_list['2'] = "green"
    elif value['new'] == "blue":
        color_list['2'] = "blue"
    elif value['new'] == "yellow":
        color_list['2'] = "yellow"
    with output:
        print("color_list_three_Callback clicked.",color_list)
def color_list_three_Callback(value):
    global model,color_list
    model="General"
    if not isinstance(value['new'],str):return
    if value['new'] == "none":
        if '3' in color_list:del color_list['3']
    elif value['new'] == "red":
        color_list['3'] = "red"
    elif value['new'] == "green":
        color_list['3'] = "green"
    elif value['new'] == "blue":
        color_list['3'] = "blue"
    elif value['new'] == "yellow":
        color_list['3'] = "yellow"
    with output:
        print("color_list_three_Callback clicked.",color_list)
def color_list_four_Callback(value):
    global model,color_list
    model="General"
    if not isinstance(value['new'],str):return
    if value['new'] == "none":
        if '4' in color_list:del color_list['4']
    elif value['new'] == "red":
        color_list['4'] = "red"
    elif value['new'] == "green":
        color_list['4'] = "green"
    elif value['new'] == "blue":
        color_list['4'] = "blue"
    elif value['new'] == "yellow":
        color_list['4'] = "yellow"
    with output:
        print("color_list_four_Callback clicked.",color_list)
color_list_one.observe(color_list_one_Callback)
color_list_two.observe(color_list_two_Callback)
color_list_three.observe(color_list_three_Callback)
color_list_four.observe(color_list_four_Callback)


# ### 抓取控制

# In[8]:


# 抓取控制
def target_detection_Callback(value):
    global model
    model = 'Detection'
    with output: print(model)
def reset_color_list_Callback(value):
    global model
    model = 'Reset_list'
    with output: print(model)
def grap_Callback(value):
    global model
    model = 'Grap'
    with output: print(model)
def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)
target_detection.on_click(target_detection_Callback)
reset_color_list.on_click(reset_color_list_Callback)
grap.on_click(grap_Callback)
exit_button.on_click(exit_button_Callback)


# ### 主程序

# In[9]:


def camera():
    global color_hsv,model,dp,msg,color_list,color_read
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2', cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # 当摄像头正常打开的情况下循环执行
    dp = []
    while capture.isOpened():
        try:
            result =  mySpeech.speech_read()
            time.sleep(0.01)
            # 读取相机的每一帧
            _, img = capture.read()
            fps.update_fps()
            xy=[joint1_slider.value,joint2_slider.value]
            if model == 'Calibration':
                time.sleep(0.5)
                dp, img = calibration.calibration_map(img, xy, threshold)
                if len(dp) != 0:
                    model="Detection2"
                    continue
            if len(dp) != 0: img = calibration.Perspective_transform(dp, img)
            if model == 'calibration_Cancel':  
                dp = []
                msg= {}
                color_read = 0
                model="General"
            if model == 'Detection2':
                color_read = 0
                color_list['1'] = "red"
                color_list['2'] = "green"
                color_list['3'] = "blue"
                color_list['4'] = "yellow"
                img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv, color_list)
                time.sleep(0.1)
                mySpeech.void_write(88) 
                print("-+-+-+-+-+")
                if coo1 == 'red':
                    color_read += 1
                    mySpeech.void_write(46)  
                if coo2 == 'green':
                    color_read += 1
                    mySpeech.void_write(49)   
                if coo3 == 'blue':
                    color_read += 1
                    mySpeech.void_write(48)  
                if coo4 == 'yellow':
                    color_read += 1
                    mySpeech.void_write(47)   
                imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
                mySpeech.void_write(89)  
                model="list1"
                    
            if model == 'list1' and color_read > 0:  
                if result == 87:
                    color_list['1'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 88:
                    color_list['1'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 89:
                    color_list['1'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 90:
                    color_list['1'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist1"
            if model == 'tolist1':
                if color_read > 0:
                    mySpeech.void_write(84) 
                    model="list2"
                elif color_read == 0:
                    mySpeech.void_write(92)
                    model="toGrap"
                   
            if model == 'list2' and color_read > 0:  
                if result == 87:
                    color_list['2'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 88:
                    color_list['2'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 89:
                    color_list['2'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 90:
                    color_list['2'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist2"
            if model == 'tolist2':
                if color_read > 0:
                    mySpeech.void_write(85)
                    model="list3"
                elif color_read == 0:
                    mySpeech.void_write(92) 
                    model="toGrap"
                         
            if model == 'list3' and color_read > 0:  
                if result == 87:
                    color_list['3'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 88:
                    color_list['3'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 89:
                    color_list['3'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 90:
                    color_list['3'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist3"
            if model == 'tolist3':
                if color_read > 0:
                    mySpeech.void_write(85)
                    model="list4"
                elif color_read == 0:
                    mySpeech.void_write(92)
                    model="toGrap"
                             
            if model == 'list4' and color_read > 0:  
                if result == 87:
                    color_list['4'] = "red" 
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 88:
                    color_list['4'] = "green"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 89:
                    color_list['4'] = "blue"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 90:
                    color_list['4'] = "yellow"
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
                elif result == 91:
                    img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                    result = 0
                    color_read -= 1
                    model="tolist4"
            if model == 'tolist4':
                mySpeech.void_write(92)
                model="toGrap"
                    
            if model == 'toGrap':  
                img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                #coo = {}
                #img, msg, coo1, coo2, coo3, coo4 = target.select_color(img, color_hsv,color_list)
                if result == 92:
                    result = 0
                    model="Grap"

            if result == 7:
                result = 0
                msg={}
                color_list = {}
                model="Detection2"
                Speech_text("好的",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1) 
            if model=="Reset_list":
                #print(coo1)
                #print(coo2)
                #print(coo3)
                #print(coo4)
                msg={}
                color_list = {}
                model="General"
            if len(msg)!= 0 and model == 'Grap':
                threading.Thread(target=target.target_run, args=(msg,xy)).start()
                msg={}
                model="General"
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            fps.show_fps(img)
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:capture.release()


# ### 启动

# In[10]:


#第一步先自动标定
model = 'Calibration'
display(controls_box,output)
threading.Thread(target=camera, ).start()

