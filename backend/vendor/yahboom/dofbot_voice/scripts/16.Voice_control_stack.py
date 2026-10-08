#!/usr/bin/env python
# coding: utf-8

# # 语音控制堆叠

# In[1]:


#导入相关的模块
import threading
from Arm_Lib import Arm_Device
import time
from dofbot_utils.robot_controller import Robot_Controller

Arm = Arm_Device(com="/dev/ttyUSB0")
robot = Robot_Controller(port="/dev/ttyUSB0")


# # 定义机械臂控制函数

# In[2]:


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
    robot.arm_clamp_block(enable, 400)
    time.sleep(.5)


# In[3]:


# 定义不同位置的变量参数
look_at = robot.P_LOOK_AT
p_top = robot.P_TOP

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED

p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE

p_layer_4 = robot.P_CENTER_HEAP_L4
p_layer_3 = robot.P_CENTER_HEAP_L3
p_layer_2 = robot.P_CENTER_HEAP_L2
p_layer_1 = robot.P_CENTER_HEAP_L1


p_push_over_1 = robot.P_OVER_1
p_push_over_2 = robot.P_OVER_2


#定义抓取方块的状态
yellow_grabbed = 0
red_grabbed = 0
green_grabbed = 0
blue_grabbed = 0


# In[4]:


arm_move_6(look_at, 1000)
time.sleep(1)


# In[5]:


#数字功能定义
def number_action(index):
    if index == 1:
        # 抓取黄色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Yellow, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 2:
        # 抓取红色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Red, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 3:
        # 抓取绿色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Green, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)
    elif index == 4:
        # 抓取蓝色的积木块
        arm_move(p_top, 1000)
        arm_move(p_Blue, 1000)
        arm_clamp_block(1)
        Arm.Arm_serial_servo_write(2, 90, 1000)
        time.sleep(1)
        arm_move(p_top, 1000)

    
def put_down_block(layer):
    
    if layer == 1:
        arm_move(p_layer_1, 1000)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    elif layer == 2:
        arm_move(p_layer_2, 1000)
        arm_clamp_block(0)
        arm_move_6(look_at, 1000)
    elif layer == 3:
        arm_move(p_layer_3, 1000)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    elif layer == 4:
        arm_move(p_layer_4, 1000)
        time.sleep(.1)
        arm_clamp_block(0) 
        arm_move_6(look_at, 1000)
    Speech_text("放置完成",EncodingFormat_Type["GB2312"])
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
        time.sleep(0.1)  
    
# 推倒积木块
def push_over_block():
    arm_move_6(p_push_over_1, 1000)
    time.sleep(.2)
    arm_move_6(p_push_over_2, 1000)
    time.sleep(.1)
    arm_move_6(look_at, 1000)
    time.sleep(1)
    global g_layer
    g_layer = 0


# In[6]:


global g_state_arm
g_state_arm = 0

global g_layer
g_layer = 0

def ctrl_arm_move(index):
    global g_layer
    g_layer = g_layer + 1
    if g_layer >= 5:
        g_layer = 1
    arm_clamp_block(0)
    if index == 1:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 2:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 3:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 4:
        number_action(index)
        time.sleep(1)
        put_down_block(g_layer)
    elif index == 5:
        time.sleep(1)
        push_over_block()
    
        
    global g_state_arm
    g_state_arm = 0


# In[7]:


def start_move_arm(index):
    # 开启机械臂控制线程
    global g_state_arm
    if g_state_arm == 0:
        closeTid = threading.Thread(target = ctrl_arm_move, args = [index])
        closeTid.setDaemon(True)
        closeTid.start()
        
        g_state_arm = 1


# # 加载语音识别和语音播报库文件

# In[8]:


import smbus
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

# In[9]:


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

# In[10]:


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


# # 清除掉电缓存区中的词条和模块模式数据，这部分第一次使用写入即可，后续如果不需要在更改设置可以把1 设置为0，或者跳过，之后设置模块的灵敏度和识别提示声的开关，之后亮起模块的RGB 灯为白色1s 和鸣笛1s，并播报"初始化完成，请发布指令"。

# In[11]:


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
    AsrAddWords(1,"jia qu hong se")
    Busy_Wait()
    AsrAddWords(2,"jia qu lan se")
    Busy_Wait()
    AsrAddWords(3,"jia qu lv se")
    Busy_Wait()
    AsrAddWords(4,"jia qu huang se")
    Busy_Wait()
    AsrAddWords(5,"tui dao")
    Busy_Wait()
    while cleck != 6:
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


# # 主进程

# In[12]:


try:
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
   while True:
       res = I2CReadByte(asr_result)
       time.sleep(0.01)
       if res != 255:
           print ("Asr Words :", res)
       if res == 4:
           if yellow_grabbed == 0:
               Speech_text("好的，开始夹取",EncodingFormat_Type["GB2312"])
               while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                   time.sleep(0.1)  
               Arm.Arm_Buzzer_On(1)
               time.sleep(.1)
               start_move_arm(1)
               time.sleep(1)
               # global yellow_grabbed
               yellow_grabbed = 1
       elif res == 1:
           if red_grabbed == 0:
               Speech_text("好的，开始夹取",EncodingFormat_Type["GB2312"])
               while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                   time.sleep(0.1)  
               Arm.Arm_Buzzer_On(1)
               time.sleep(.1)
               start_move_arm(2)
               # global red_grabbed
               red_grabbed = 1
       elif res == 3:
           if green_grabbed == 0:
               Speech_text("好的，开始夹取",EncodingFormat_Type["GB2312"])
               while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                   time.sleep(0.1)  
               Arm.Arm_Buzzer_On(1)
               time.sleep(.1)
               start_move_arm(3)
               # global green_grabbed
               green_grabbed = 1
       elif res == 2:
           if blue_grabbed == 0:
               Speech_text("好的，开始夹取",EncodingFormat_Type["GB2312"])
               while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
                   time.sleep(0.1)  
               Arm.Arm_Buzzer_On(1)
               time.sleep(.1)
               start_move_arm(4)
               # global blue_grabbed
               blue_grabbed = 1
       elif res == 5: #拳头
           Arm.Arm_Buzzer_On(1)
           time.sleep(.1)
           start_move_arm(5)
           yellow_grabbed = 0
           red_grabbed = 0
           green_grabbed = 0
           blue_grabbed = 0

except KeyboardInterrupt:
   print(" Program closed! ")
   pass


# In[ ]:




