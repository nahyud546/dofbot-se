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


# # 定义语音控制函数
# # Define voice control function

# In[3]:


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


# # 清除掉电缓存区中的词条和模块模式数据，这部分第一次使用写入即可，后续如果不需要在更改设置可以把1 设置为0，或者跳过，之后设置模块的灵敏度和识别提示声的开关，之后亮起模块的RGB 灯为白色1s 和鸣笛1s。
# # Clear the entries and module mode data in the power-off cache area. This part can be written for the first time. If you do not need to change the settings later, you can set 1 to 0 or skip it, and then set the sensitivity and sensitivity of the module. Identify the beep switch, then turn on the module's RGB light in white for 1s and whistle for 1s.

# In[4]:


'''
The mode and phrase have the function of power-down save, if there is no modification after the first entry, you can change 1 to 0
'''
cleck = 1

if 1:
    bus.write_byte_data(i2c_addr, asr_clear_addr, 0x40)#Clear the power-down buffer area
    Busy_Wait()#Wait for the module to be free
    print("Cache cleared")
    bus.write_byte_data(i2c_addr, asr_mode_addr, 0x00)#Set to loop mode
    Busy_Wait()
    print("The mode is set")
    AsrAddWords(1,"hong deng")
    Busy_Wait()
    AsrAddWords(2,"lv deng")
    Busy_Wait()
    AsrAddWords(3,"lan deng")
    Busy_Wait()
    AsrAddWords(4,"guan deng")
    Busy_Wait()
    while cleck != 4:
        cleck = I2CReadByte(asr_num_cleck)
        print(cleck)

bus.write_byte_data(i2c_addr, asr_rec_gain_addr, 0x40)#Set the sensitivity, the recommended value is 0x40-0x55
bus.write_byte_data(i2c_addr, asr_voice_flag, 1)#Set switch sound
bus.write_byte_data(i2c_addr, asr_buzzer, 1)#buzzer
RGBSet(255,255,255)
time.sleep(1)
RGBSet(0,0,0)
bus.write_byte_data(i2c_addr, asr_buzzer, 0)#buzzer


# # 循环检测声音
# # Loop to detect sounds

# In[5]:


while True:
    result = I2CReadByte(asr_result)
    print(result)
    time.sleep(0.5)


# In[ ]:




