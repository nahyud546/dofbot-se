#!/usr/bin/env python
# coding: utf-8

# In[1]:


import sys
sys.path.append('/home/jetson/dofbot_ws/src/dofbot_color_grab/scripts')


# # 初始化机械臂

# In[2]:


#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

# 获取机械臂的对象
Arm = Arm_Device(com="/dev/ttyUSB0")
time.sleep(.1)

from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller(port="/dev/ttyUSB0")


# # 初始化机摄像头

# In[3]:


#bgr8转jpeg格式
import enum
import cv2

def bgr8_to_jpeg(value, quality=75):
    return bytes(cv2.imencode('.jpg', value)[1])


# # 初始化摄像头组件

# In[4]:


#摄像头组件显示
import traitlets
import ipywidgets.widgets as widgets
import time
# 线程功能操作库
import threading
import inspect
import ctypes

origin_widget = widgets.Image(format='jpeg', width=320, height=240)
mask_widget = widgets.Image(format='jpeg',width=320, height=240)
result_widget = widgets.Image(format='jpeg',width=320, height=240)

# 创建一个水平框容器，将图像小部件相邻放置
image_container = widgets.HBox([origin_widget, mask_widget, result_widget])
# image_container = widgets.Image(format='jpeg', width=600, height=500)


# # 定义线程和颜色识别函数

# In[5]:


#线程相关函数
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # 如果res的值大于1，则需要异常处理
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


# In[6]:


def get_color(img):
    H = []
    color_name={}
    img = cv2.resize(img, (640, 480), )
    # 将彩色图转成HSV
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 画矩形框
    cv2.rectangle(img, (280, 180), (360, 260), (0, 255, 0), 2)
    # 依次取出每行每列的H,S,V值放入容器中
    for i in range(280, 360):
        for j in range(180, 260): H.append(HSV[j, i][0])
    # 分别计算出H,S,V的最大最小
    H_min = min(H);H_max = max(H)
#     print(H_min,H_max)
    # 判断颜色
    if H_min >= 0 and H_max <= 20 or H_min >= 156 and H_max <= 180: color_name['name'] = 'red'
    elif H_min >= 21 and H_max <= 28: color_name['name'] = 'yellow'
    elif H_min >= 35 and H_max <= 78: color_name['name'] = 'green'
    elif H_min >= 100 and H_max <= 124: color_name['name'] = 'blue'
    return img, color_name


# # 定义不同位置的变量参数

# In[7]:


# 定义不同位置的变量参数
look_at = robot.P_LOOK_AT
p_top = robot.P_TOP

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED

p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE

p_gray = robot.P_CENTER


# # 定义移动机械臂函数

# In[8]:


# 定义移动机械臂函数,同时控制1-6号舵机运动，p=[S1,S2,S3,S4,S5,S6]
def arm_move_6(p, s_time = 500):
    for i in range(6):
        id = i + 1
        Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)
    
# 定义移动机械臂函数,同时控制1-5号舵机运动，p=[S1,S2,S3,S4,S5]
def arm_move(p, s_time = 500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time*1.2))
        elif id == 1 :
            Arm.Arm_serial_servo_write(id, p[i], int(3*s_time/4))
        else:
            Arm.Arm_serial_servo_write(id, p[i], int(s_time))
        time.sleep(.01)
    time.sleep(s_time/1000)
    
# 定义夹积木块函数，enable=1：夹住，=0：松开
def arm_clamp_block(enable):
    if enable == 0:
        Arm.Arm_serial_servo_write(6, 60, 400)
    else:
        Arm.Arm_serial_servo_write(6, 135, 400)
    time.sleep(.5)


# In[9]:


arm_move_6(look_at, 1000)
time.sleep(1)


# # 定义颜色控制机械臂函数

# In[10]:


global g_state_arm
g_state_arm = 0

def ctrl_arm_move(index):
    arm_clamp_block(0)
    if index == 1:
        print("黄色")
        Speech_text("识别到黄色",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        Speech_text("放置完成",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  
    elif index == 2:
        print("红色")
        Speech_text("识别到红色",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1) 
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        Speech_text("放置完成",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  
    elif index == 3:
        print("绿色")
        Speech_text("识别到绿色",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1) 
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        Speech_text("放置完成",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  
    elif index == 4:
        print("蓝色")
        Speech_text("识别到蓝色",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1) 
        Arm.Arm_Buzzer_On(1)
        time.sleep(.5)
        number_action(index)
        put_down_block()
        Speech_text("放置完成",EncodingFormat_Type["GB2312"])
        while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
            time.sleep(0.1)  
    
    global g_state_arm
    g_state_arm = 0


# In[11]:


#数字功能定义
def number_action(index):
    if index == 1:
        # 抓取黄色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Yellow, 1500)
    elif index == 2:
        # 抓取红色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Red, 1500)
    elif index == 3:
        # 抓取绿色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Green, 1500)
    elif index == 4:
        # 抓取蓝色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Blue, 1500)
    arm_clamp_block(1)
    arm_move(p_top, 1500)

    
def put_down_block():
    arm_move(p_gray, 1000)
    arm_clamp_block(0) 
    time.sleep(.5)
    arm_move_6(look_at, 1500)
  


# In[12]:


def start_move_arm(index):
    # 开启机械臂控制线程
    global g_state_arm
    if g_state_arm == 0:
        closeTid = threading.Thread(target = ctrl_arm_move, args = [index])
        closeTid.setDaemon(True)
        closeTid.start()
        
        g_state_arm = 1


# # 加载语音播报库文件

# In[13]:


import smbus
# bus = smbus.SMBus(1)
try:
    bus = smbus.SMBus(Arm.get_i2c_bus_num())
except AttributeError:
    bus = smbus.SMBus(1)  # laptop: I2C module speech khong noi truc tiep, dung shim/MOCK

i2c_speech_addr = 0x30   #语音播报模块地址
speech_date_head = 0xfd


# # 定义播报参数控制函数

# In[14]:


def I2C_WriteBytes(str_):
    global i2c_speech_addr
    for ch in str_:
        try:
            bus.write_byte(i2c_speech_addr,ch)
            time.sleep(0.01)
        except:
            print("write I2C error")



EncodingFormat_Type = {
                        'GB2312':0x00,
                        'GBK':0X01,
                        'BIG5':0x02,
                        'UNICODE':0x03
                        }
def Speech_text(str_,encoding_format):
    str_ = str_.encode('gb2312')   
    size = len(str_)+2
    DataHead = speech_date_head
    Length_HH = size>>8
    Length_LL = size & 0x00ff
    Commond = 0x01
    EncodingFormat = encoding_format

    Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

    I2C_WriteBytes(Date_Pack)

    I2C_WriteBytes(str_)

def SetBase(str_):
    str_ = str_.encode('gb2312')   
    size = len(str_)+2

    DataHead = speech_date_head
    Length_HH = size>>8
    Length_LL = size & 0x00ff
    Commond = 0x01
    EncodingFormat = 0x00

    Date_Pack = [DataHead,Length_HH,Length_LL,Commond,EncodingFormat]

    I2C_WriteBytes(Date_Pack)

    I2C_WriteBytes(str_)

def TextCtrl(ch,num):
    if num != -1:
        str_T = '[' + ch + str(num) + ']'
        SetBase(str_T)
    else:
        str_T = '[' + ch + ']'
        SetBase(str_T)


ChipStatus_Type = {
                    'ChipStatus_InitSuccessful':0x4A,#初始化成功回传
                    'ChipStatus_CorrectCommand':0x41,#收到正确的命令帧回传
                    'ChipStatus_ErrorCommand':0x45,#收到不能识别命令帧回传
                    'ChipStatus_Busy':0x4E,#芯片忙碌状态回传
                    'ChipStatus_Idle':0x4F #芯片空闲状态回传                  
                }

def GetChipStatus():
    global i2c_speech_addr
    AskState = [0xfd,0x00,0x01,0x21]
    try:
        I2C_WriteBytes(AskState)
        time.sleep(0.05)
    except:
        print("I2CRead_Write error")


    try:
        Read_result = bus.read_byte(i2c_speech_addr)
        return Read_result
    except:
        print("I2CRead error")

Style_Type = {
                'Style_Single':0,#为 0，一字一顿的风格
                'Style_Continue':1#为 1，正常合成
                }#合成风格设置 [f?]

def SetStyle(num):
    TextCtrl('f',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)   


Language_Type = {
                'Language_Auto':0,#为 0，自动判断语种
                'Language_Chinese':1,#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                'Language_English':2#为 1，阿拉伯数字、度量单位、特殊符号等合成为中文
                }#合成语种设置 [g?]

def SetLanguage(num):
    TextCtrl('g',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

Articulation_Type = {
                'Articulation_Auto':0,#为 0，自动判断单词发音方式
                'Articulation_Letter':1,#为 1，字母发音方式
                'Articulation_Word':2#为 2，单词发音方式
                }#设置单词的发音方式 [h?]

def SetArticulation(num):
    TextCtrl('h',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


Spell_Type = {
                'Spell_Disable':0,#为 0，不识别汉语拼音
                'Spell_Enable':1#为 1，将“拼音＋1 位数字（声调）”识别为汉语拼音，例如： hao3
                }#设置对汉语拼音的识别 [i?]

def SetSpell(num):
    TextCtrl('i',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


Reader_Type = {
                'Reader_XiaoYan':3,#为 3，设置发音人为小燕(女声, 推荐发音人)
                'Reader_XuJiu':51,#为 51，设置发音人为许久(男声, 推荐发音人)
                'Reader_XuDuo':52,#为 52，设置发音人为许多(男声)
                'Reader_XiaoPing':53,#为 53，设置发音人为小萍(女声
                'Reader_DonaldDuck':54,#为 54，设置发音人为唐老鸭(效果器)
                'Reader_XuXiaoBao':55#为 55，设置发音人为许小宝(女童声)                
                }#选择发音人 [m?]

def SetReader(num):
    TextCtrl('m',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


NumberHandle_Type = {
                'NumberHandle_Auto':0,#为 0，自动判断
                'NumberHandle_Number':1,#为 1，数字作号码处理
                'NumberHandle_Value':2#为 2，数字作数值处理
                }#设置数字处理策略 [n?]

def SetNumberHandle(num):
    TextCtrl('n',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)



ZeroPronunciation_Type = {
                'ZeroPronunciation_Zero':0,#为 0，读成“zero
                'ZeroPronunciation_O':1#为 1，读成“欧”音
                }#数字“0”在读 作英文、号码时 的读法 [o?]

def SetZeroPronunciation(num):
    TextCtrl('o',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)



NamePronunciation_Type = {
                'NamePronunciation_Auto':0,#为 0，自动判断姓氏读音
                'NamePronunciation_Constraint':1#为 1，强制使用姓氏读音规则
                }#设置姓名读音 策略 [r?]


def SetNamePronunciation(num):
    TextCtrl('r',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

#设置语速 [s?] ? 为语速值，取值：0～10
def SetSpeed(speed):
    TextCtrl('s',speed)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


#设置语调 [t?] ? 为语调值，取值：0～10
def SetIntonation(intonation):
    TextCtrl('t',intonation)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

#设置音量 [v?] ? 为音量值，取值：0～10
def SetVolume(volume):
    TextCtrl('v',volume)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


OnePronunciation_Type = {
                'OnePronunciation_Yao':0,#为 0，合成号码“1”时读成幺
                'OnePronunciation_Yi':1#为 1，合成号码“1”时读成一
                }#设置号码中“1”的读法 [y?]

def SetOnePronunciation(num):
    TextCtrl('y',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


Rhythm_Type = {
                'Rhythm_Diasble':0,#为 0，“ *”和“#”读出符号
                'Rhythm_Enable':1#为 1，处理成韵律，“*”用于断词，“#”用于停顿
                }#是否使用韵律 标记“*”和“#” [z?]

def SetRhythm(num):
    TextCtrl('z',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

#恢复默认的合成参数 [d] 所有设置（除发音人设置、语种设置外）恢复为默认值
def SetRestoreDefault():
    TextCtrl('d',-1)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


# # 颜色识别主进程

# # 启动进程

# In[15]:


import cv2
import numpy as np
import ipywidgets.widgets as widgets

cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)  #设置帧率
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
# image.set(cv2.CAP_PROP_BRIGHTNESS, 40) #设置亮度 -64 - 64  0.0
# image.set(cv2.CAP_PROP_CONTRAST, 50)   #设置对比度 -64 - 64  2.0
# image.set(cv2.CAP_PROP_EXPOSURE, 156)  #设置曝光值 1.0 - 5000  156.0


# 默认选择红色的,程序会自动根据方框中检测到的颜色切换颜色
# 红色区间
color_lower = np.array([0, 43, 46])
color_upper = np.array([10, 255, 255])

# #绿色区间
# color_lower = np.array([35, 43, 46])
# color_upper = np.array([77, 255, 255])

# #蓝色区间
# color_lower=np.array([100, 43, 46])
# color_upper = np.array([124, 255, 255])

# #黄色区间
# color_lower = np.array([26, 43, 46])
# color_upper = np.array([34, 255, 255])

# #橙色区间
# color_lower = np.array([11, 43, 46])
# color_upper = np.array([25, 255, 255])


def Color_Recongnize():
    Arm.Arm_Buzzer_On(1)
    s_time = 300
    Arm.Arm_serial_servo_write(4, 10, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 0, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 10, s_time)
    time.sleep(s_time/1000)
    Arm.Arm_serial_servo_write(4, 0, s_time)
    time.sleep(s_time/1000)
    
    SetReader(Reader_Type["Reader_XiaoPing"])#选择播音人晓萍
    SetVolume(8)
    Speech_text("初始化完成",EncodingFormat_Type["GB2312"])
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
        time.sleep(0.1)  
    while(1):
        # get a frame and show 获取视频帧并转成HSV格式, 利用cvtColor()将BGR格式转成HSV格式，参数为cv2.COLOR_BGR2HSV。
        ret, frame = cap.read()
        frame, color_name = get_color(frame)
        if len(color_name)==1:
            global color_lower
            global color_upper
            # print ("color_name :", color_name)
            # print ("name :", color_name['name'])
            if color_name['name'] == 'yellow':
                color_lower = np.array([21, 43, 46])
                color_upper = np.array([28, 255, 255])
                start_move_arm(1)
            elif color_name['name'] == 'red':
                color_lower = np.array([0, 43, 46])
                color_upper = np.array([10, 255, 255])
                start_move_arm(2)
            elif  color_name['name'] == 'green':
                color_lower = np.array([35, 43, 46])
                color_upper = np.array([77, 255, 255])
                start_move_arm(3) 
            elif color_name['name'] == 'blue':
                color_lower=np.array([100, 43, 46])
                color_upper = np.array([124, 255, 255]) 
                start_move_arm(4)
        
        origin_widget.value = bgr8_to_jpeg(frame)
        #cv2.imshow('Capture', frame)

        # change to hsv model
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # get mask 利用inRange()函数和HSV模型中蓝色范围的上下界获取mask，mask中原视频中的蓝色部分会被弄成白色，其他部分黑色。
        mask = cv2.inRange(hsv, color_lower, color_upper)
        #cv2.imshow('Mask', mask)
        mask_widget.value = bgr8_to_jpeg(mask)
        # detect blue 将mask于原视频帧进行按位与操作，则会把mask中的白色用真实的图像替换：
        res = cv2.bitwise_and(frame, frame, mask=mask)
        #cv2.imshow('Result', res)
        result_widget.value = bgr8_to_jpeg(res)
        time.sleep(0.01)


# In[16]:


#启动进程
thread1 = threading.Thread(target=Color_Recongnize)
thread1.setDaemon(True)
thread1.start()


# # 等待结束进程

# In[17]:


display(image_container)
#等待结束进程
try:
    while True:
        time.sleep(.000001)
except KeyboardInterrupt:
    print(" Program closed! ")
    pass


# # 释放进程

# In[18]:


#使用完成对象记住释放掉对象，不然下一个程序使用这个对象模块会被占用，导致无法使用
stop_thread(thread1)
cap.release()  


# In[ ]:




