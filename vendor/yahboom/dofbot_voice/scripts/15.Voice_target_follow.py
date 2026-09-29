#!/usr/bin/env python
# coding: utf-8

# # 语音识别目标追踪

# ### 导入头文件

# In[1]:


import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_color_follow/scripts')
sys.path.append('/home/jetson/dofbot_pro/dofbot_face_follow/scripts')


# In[2]:


import cv2 as cv
import threading
import random
from time import sleep
import ipywidgets as widgets
from IPython.display import display
from color_follow import color_follow
from face_follow import Face_Follow
from dofbot_utils.dofbot_config import *


# ### 初始化机械臂位置

# In[ ]:


import Arm_Lib
Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
joints_0 = [90, 135, 20, 25, 90, 30]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)


# # 加载语音识别和语音播报库文件

# In[ ]:


import smbus
import time
# bus = smbus.SMBus(1)
try:
    bus = smbus.SMBus(Arm.get_i2c_bus_num())
except AttributeError:
    bus = smbus.SMBus(1)  # laptop: I2C module speech khong noi truc tiep, dung shim/MOCK

i2c_addr = 0x0f   #Speech recognition module address
asr_add_word_addr  = 0x01   #Entry add address
asr_mode_addr  = 0x02   #Recognition mode setting address, the value is 0-2, 0: cyclic recognition mode 1: password mode, 2: button mode, the default is cyclic detection
asr_rgb_addr = 0x03   #RGB lamp setting address, need to send two bits, the first one is directly the lamp number 1: blue 2: red 3: green
                      #The second byte is brightness 0-255, the larger the value, the higher the brightness
asr_rec_gain_addr  = 0x04    #Identification sensitivity setting address, sensitivity can be set to 0x00-0x7f, the higher the value, the easier it is to detect but the easier it is to misjudge
                             #It is recommended to set the value to 0x40-0x55, the default value is 0x40
asr_clear_addr = 0x05   #Clear the operation address of the power-off cache, clear the cache area information before entering the information
asr_key_flag = 0x06  #Used in key mode, set the startup recognition mode
asr_voice_flag = 0x07   #Used to set whether to turn on the recognition result prompt sound
asr_result = 0x08  #Recognition result storage address
asr_buzzer = 0x09  #Buzzer control register, 1 bit is on, 0 bit is off
asr_num_cleck = 0x0a #Check the number of entries
asr_vession = 0x0b #firmware version number
asr_busy = 0x0c #Busy and busy flag

i2c_speech_addr = 0x30   #语音播报模块地址
speech_date_head = 0xfd


# # 定义播报参数控制函数

# In[ ]:


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


# # 定义语音控制函数

# In[ ]:


#Write entry
def AsrAddWords(idnum,str):
    global i2c_addr
    global asr_add_word_addr
    words = []
    words.append(asr_add_word_addr)
    words.append(len(str) + 2)
    words.append(idnum)
    for  alond_word in str:
        words.append(ord(alond_word))
    words.append(0)
    print(words)
    for date in words:
        bus.write_byte (i2c_addr, date)
        time.sleep(0.03)

#Set RGB
def RGBSet(R,G,B):
    global i2c_addr
    global asr_rgb_addr
    date = []
    date.append(R)
    date.append(G)
    date.append(B)
    print(date)
    bus.write_i2c_block_data (i2c_addr,asr_rgb_addr,date)

#Read result
def I2CReadByte(reg):
    global i2c_addr
    bus.write_byte (i2c_addr, reg)
    time.sleep(0.05)
    Read_result = bus.read_byte (i2c_addr)
    return Read_result

#Wait busy
def Busy_Wait():
    busy = 255
    while busy != 0:
        busy = I2CReadByte(asr_busy)
        print(asr_busy)


# ### 创建实例,初始化参数

# In[ ]:


follow = color_follow()
follow2 = Face_Follow()
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
HSV_path="/home/jetson/dofbot_pro/src/dofbot_color_follow/scripts/HSV_config.txt"
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error !!!")


# ### 创建控件

# In[ ]:


button_layout = widgets.Layout(width='200px', height='100px', align_self='center')
# 输出控件
output = widgets.Output()
# 颜色追踪
color_follow = widgets.Button(description='color_follow', button_style='success', layout=button_layout)
# 选择颜色
choose_color = widgets.ToggleButtons(options=['red', 'green', 'blue', 'yellow'], button_style='success',
             tooltips=['Description of slow', 'Description of regular', 'Description of fast'])
# 取消追踪
follow_cancel = widgets.Button(description='follow_cancel', button_style='danger', layout=button_layout)
# 学习颜色
learning_color = widgets.Button(description='learning_color', button_style='primary', layout=button_layout)
# 学习颜色追踪
learning_follow = widgets.Button(description='learning_follow', button_style='success', layout=button_layout)
# 退出
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
# 图像控件
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='auto'))
# 垂直布局
img_box = widgets.VBox([imgbox, choose_color], layout=widgets.Layout(align_self='auto'))
# 垂直布局
Slider_box = widgets.VBox([color_follow, learning_color, learning_follow,follow_cancel,exit_button],
                          layout=widgets.Layout(align_self='auto'))
# 水平布局
controls_box = widgets.HBox([imgbox, exit_button], layout=widgets.Layout(align_self='auto'))
# ['auto', 'flex-start', 'flex-end', 'center', 'baseline', 'stretch', 'inherit', 'initial', 'unset']


# ### 模式切换

# In[ ]:


def color_follow_Callback(value):
    global model
    model = 'color_follow'
def learning_color_Callback(value):
    global model
    model = 'learning_color'
def learning_follow_Callback(value):
    global model
    model = 'learning_follow'
def follow_cancel_Callback(value):
    global model
    model = 'General'
def exit_button_Callback(value):
    global model
    model = 'Exit'
color_follow.on_click(color_follow_Callback)
learning_color.on_click(learning_color_Callback)
learning_follow.on_click(learning_follow_Callback)
follow_cancel.on_click(follow_cancel_Callback)
exit_button.on_click(exit_button_Callback)


# # 清除掉电缓存区中的词条和模块模式数据，这部分第一次使用写入即可，后续如果不需要在更改设置可以把1 设置为0，或者跳过，之后设置模块的灵敏度和识别提示声的开关，之后亮起模块的RGB 灯为白色1s 和鸣笛1s，并播报"初始化完成，请发布指令"。

# In[ ]:


'''
The mode and phrase have the function of power-down save, if there is no modification after the first entry, you can change 1 to 0
'''
cleck = 1

if 1:
    bus.write_byte_data(i2c_addr, asr_clear_addr, 0x40)#Clear the power-down buffer area
    Busy_Wait()#Wait for the module to be free
    print("Cache cleared")
    bus.write_byte_data(i2c_addr, asr_mode_addr, 1)
    Busy_Wait()
    print("The mode is set")
    AsrAddWords(0,"xiao ya")
    Busy_Wait()
    AsrAddWords(1,"kai shi zhui zong hong se")
    Busy_Wait()
    AsrAddWords(2,"kai shi zhui zong huang se")
    Busy_Wait()
    AsrAddWords(3,"kai shi zhui zong lv se")
    Busy_Wait()
    AsrAddWords(4,"kai shi zhui zong lan se")
    Busy_Wait()
    AsrAddWords(5,"kai shi zhui zong ren lian")
    Busy_Wait()
    AsrAddWords(6,"qu xiao zhui zong")
    Busy_Wait()
    while cleck != 7:
        cleck = I2CReadByte(asr_num_cleck)
        print(cleck)

bus.write_byte_data(i2c_addr, asr_rec_gain_addr, 0x40)#Set the sensitivity, the recommended value is 0x40-0x55
bus.write_byte_data(i2c_addr, asr_voice_flag, 1)#Set switch sound
bus.write_byte_data(i2c_addr, asr_buzzer, 1)#buzzer
RGBSet(255,255,255)
time.sleep(1)
RGBSet(0,0,0)
bus.write_byte_data(i2c_addr, asr_buzzer, 0)#buzzer

SetReader(Reader_Type["Reader_XiaoPing"])#选择播音人晓萍
SetVolume(8)
Speech_text("初始化完成，请发布指令",EncodingFormat_Type["GB2312"])
while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
    time.sleep(0.1)  


# ### 主程序

# In[ ]:


def camera():
    global HSV_learning,model
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2', cv.CAP_V4L2)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)  #设置帧率
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            result = I2CReadByte(asr_result)
            time.sleep(0.01)
            # 读取相机的每一帧
            _, img = capture.read()
            # 统一图像大小
            # img = cv.resize(img, (640, 480))
            
            if result == 1:
                choose_color.value = 'red'
                model = 'color_follow'
                result = 255
                Speech_text("好的，开始追踪红色",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1)  
            elif result == 2:
                choose_color.value = 'yellow'
                model = 'color_follow'
                result = 255
                Speech_text("好的，开始追踪黄色",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1)   
            elif result == 3:
                choose_color.value = 'green'
                model = 'color_follow'
                result = 255
                Speech_text("好的，开始追踪绿色",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1)  
            elif result == 4:
                choose_color.value = 'blue'
                model = 'color_follow'
                result = 255
                Speech_text("好的，开始追踪蓝色",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1)   
            elif result == 5:
                model = 'follow2'
                result = 255
                Speech_text("好的，开始追踪人脸",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1)  
            elif result == 6:
                model = 'General'
                result = 255
                Speech_text("好的，取消追踪",EncodingFormat_Type["GB2312"])
                while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                    time.sleep(0.1) 
            
            if model == 'color_follow':
                img = follow.follow_function(img, color_hsv[choose_color.value])
                # 添加文字
                cv.putText(img, choose_color.value, (int(img.shape[0] / 2), 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
            if model == 'follow2':
                img, pos = follow2.follow_function(img)
            if model == 'learning_color':
                img,HSV_learning = follow.get_hsv(img)
            if model == 'learning_follow' :
                if len(HSV_learning)!=0:
                    print(HSV_learning)
                    img = follow.learning_follow(img, HSV_learning)
                    # 添加文字
                    cv.putText(img,'LeColor', (240, 50), cv.FONT_HERSHEY_SIMPLEX, 1, color[random.randint(0, 254)], 1)
            if model == 'Exit':
                capture.release()
                break
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:
            capture.release()
            break
        except Exception as e:
            print("error:", e)
            break


# ### 启动

# In[ ]:


display(controls_box,output)
threading.Thread(target=camera, ).start()


# ##### 

# In[ ]:





# In[ ]:




