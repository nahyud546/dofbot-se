#!/usr/bin/env python
# coding: utf-8

# # 定义模块的设备地址和寄存器地址
# # Define the device address and registration address of the module

# In[1]:


#!/usr/bin/env python3
#coding=utf-8
import smbus
import time

# bus = smbus.SMBus(1)
import Arm_Lib
Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
try:
    bus = smbus.SMBus(Arm.get_i2c_bus_num())
except AttributeError:
    bus = smbus.SMBus(1)  # laptop: I2C module speech khong noi truc tiep, dung shim/MOCK


# In[2]:


i2c_addr = 0x30 
date_head = 0xfd


# # 定义播报参数控制函数
# # Define broadcast parameter control function

# In[3]:


def I2C_WriteBytes(str_):
    global i2c_addr
    for ch in str_:
        try:
            bus.write_byte(i2c_addr,ch)
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
    DataHead = date_head
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

    DataHead = date_head
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
                    'ChipStatus_InitSuccessful':0x4A,#初始化成功回传#Initialization successful return
                    'ChipStatus_CorrectCommand':0x41,#收到正确的命令帧回传#Receive the correct command frame and return it
                    'ChipStatus_ErrorCommand':0x45,#收到不能识别命令帧回传#Receipt of unrecognized command frame response
                    'ChipStatus_Busy':0x4E,#芯片忙碌状态回传#Chip busy status return
                    'ChipStatus_Idle':0x4F #芯片空闲状态回传  #Chip idle status return                
                }

def GetChipStatus():
    global i2c_addr
    AskState = [0xfd,0x00,0x01,0x21]
    try:
        I2C_WriteBytes(AskState)
        time.sleep(0.05)
    except:
        print("I2CRead_Write error")


    try:
        Read_result = bus.read_byte(i2c_addr)
        return Read_result
    except:
        print("I2CRead error")

Style_Type = {
                'Style_Single':0,
                'Style_Continue':1
                }#合成风格设置 [f?]#Synthesis style settings [f?]

def SetStyle(num):
    TextCtrl('f',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)   


Language_Type = {
                'Language_Auto':0,
                'Language_Chinese':1,
                'Language_English':2
                }#合成语种设置 [g?]#Synthetic language setting [g?]

def SetLanguage(num):
    TextCtrl('g',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

Articulation_Type = {
                'Articulation_Auto':0,
                'Articulation_Letter':1,
                'Articulation_Word':2
                }#设置单词的发音方式 [h?]#Set the pronunciation method of the word [h?]

def SetArticulation(num):
    TextCtrl('h',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


Spell_Type = {
                'Spell_Disable':0,
                'Spell_Enable':1
                }#设置对汉语拼音的识别 [i?]#Set up recognition of Chinese Pinyin [i?]

def SetSpell(num):
    TextCtrl('i',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


Reader_Type = {
                'Reader_XiaoYan':3,
                'Reader_XuJiu':51,
                'Reader_XuDuo':52,
                'Reader_XiaoPing':53,
                'Reader_DonaldDuck':54,
                'Reader_XuXiaoBao':55            
                }#选择发音人 [m?]#Select the speaker [m?]

def SetReader(num):
    TextCtrl('m',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)


NumberHandle_Type = {
                'NumberHandle_Auto':0,
                'NumberHandle_Number':1,
                'NumberHandle_Value':2
                }#设置数字处理策略 [n?]#Set number processing strategy [n?]

def SetNumberHandle(num):
    TextCtrl('n',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)



ZeroPronunciation_Type = {
                'ZeroPronunciation_Zero':0,
                'ZeroPronunciation_O':1
                }#数字“0”在读 作英文、号码时 的读法 [o?]#How to pronounce the number "0" in English and as a number [o?]

def SetZeroPronunciation(num):
    TextCtrl('o',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)



NamePronunciation_Type = {
                'NamePronunciation_Auto':0,
                'NamePronunciation_Constraint':1
                }#设置姓名读音 策略 [r?]#Set name pronunciation strategy [r?]


def SetNamePronunciation(num):
    TextCtrl('r',num)
    while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:
        time.sleep(0.002)

#设置语速 [s?] ? 为语速值，取值：0～10
#Set speaking speed [s?] ? is the speaking speed value, value: 0~10
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


# # 配置参数并播放：你好亚博智能科技

# In[4]:


SetReader(Reader_Type["Reader_XiaoPing"])#选择播音人晓萍
SetVolume(10)
Speech_text("你好亚博智能科技",EncodingFormat_Type["GB2312"])

while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
    time.sleep(0.1)  


# # 配置参数并播放：欢迎使用亚博智能语音播报模块

# In[5]:


SetReader(Reader_Type["Reader_XuDuo"])#选择播音人许多
Speech_text("欢迎使用亚博智能语音播报模块",EncodingFormat_Type["GB2312"])

while GetChipStatus() != ChipStatus_Type['ChipStatus_Idle']:#等待当前语句播报结束
    time.sleep(0.1)   


# In[ ]:




