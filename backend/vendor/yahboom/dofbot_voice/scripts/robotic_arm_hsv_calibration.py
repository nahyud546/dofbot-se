#!/usr/bin/env python
# coding: utf-8

# # HSV校准

# ### 导入头文件

# In[1]:


#!/usr/bin/env python
# coding: utf-8
import threading
import cv2 as cv
from time import sleep
from dofbot_config import *
import ipywidgets as widgets
from IPython.display import display


# ### 创建实例,初始化参数

# In[2]:


# 创建更新HSV实例
update_hsv = update_hsv()
# 初始化num参数
num=0
# 初始化模式
model = "General"
# 初始化HSV名称
HSV_name=None
# 初始化HSV值
color_hsv  = {"red"   : ((0, 43, 46), (10, 255, 255)),
              "green" : ((35, 43, 46), (77, 255, 255)),
              "blue"  : ((100, 43, 46), (124, 255, 255)),
              "yellow": ((26, 43, 46), (34, 255, 255))}
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
# HSV参数路径
HSV_path="./HSV_config.txt"
# 读取HSV配置文件,更新HSV值
try: read_HSV(HSV_path,color_hsv)
except Exception: print("Read HSV_config Error!!!")


# ### 初始化机械臂位置

# In[3]:


import Arm_Lib
Arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
joints_0 = [90, 135, 0, 0, 90, 30]
Arm.Arm_serial_servo_write6_array(joints_0, 1000)


# ### 创建控件

# In[4]:


# 创建布局
button_layout      = widgets.Layout(width='260px', height='40px', align_self='center')
# 输出部件
output = widgets.Output()
# 进入颜色更新模式
HSV_update_red     = widgets.Button(description='HSV_update_red',    button_style='success', layout=button_layout)
HSV_update_green   = widgets.Button(description='HSV_update_green',  button_style='success', layout=button_layout)
HSV_update_blue    = widgets.Button(description='HSV_update_blue',   button_style='success', layout=button_layout)
HSV_update_yellow  = widgets.Button(description='HSV_update_yellow', button_style='success', layout=button_layout)
HSV_write_file     = widgets.Button(description='HSV_write_file',    button_style='primary', layout=button_layout)
Color_Binary       = widgets.Button(description='Color/Binary',      button_style='info',    layout=button_layout)
# 调整滑杆
H_min_slider       = widgets.IntSlider(description='H_min :', value=0, min=0, max=255, step=1, orientation='horizontal')
S_min_slider       = widgets.IntSlider(description='S_min :', value=43, min=0, max=255, step=1, orientation='horizontal')
V_min_slider       = widgets.IntSlider(description='V_min :', value=46, min=0, max=255, step=1, orientation='horizontal')
H_max_slider       = widgets.IntSlider(description='H_max :', value=10, min=0, max=255, step=1, orientation='horizontal')
S_max_slider       = widgets.IntSlider(description='S_max :', value=255, min=0, max=255, step=1, orientation='horizontal')
V_max_slider       = widgets.IntSlider(description='V_max :', value=255, min=0, max=255, step=1, orientation='horizontal')
# 退出按钮
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
# 图像控件
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
# 调试按钮布局
HSV_slider = widgets.VBox(
    [H_min_slider, S_min_slider, V_min_slider, H_max_slider, S_max_slider, V_max_slider, HSV_update_red,
     HSV_update_green, HSV_update_blue, HSV_update_yellow, Color_Binary,HSV_write_file, exit_button],
    layout=widgets.Layout(align_self='center'))
# 整体布局
controls_box = widgets.HBox([imgbox,HSV_slider], layout=widgets.Layout(align_self='center'))


# ### 颜色更新回调

# In[5]:


def update_red_Callback(value):
    # 点击红色按钮,回调函数
    global HSV_name
    HSV_name = "red"
    H_min_slider.value=color_hsv[HSV_name][0][0]
    S_min_slider.value=color_hsv[HSV_name][0][1]
    V_min_slider.value=color_hsv[HSV_name][0][2]
    H_max_slider.value=color_hsv[HSV_name][1][0]
    S_max_slider.value=color_hsv[HSV_name][1][1]
    V_max_slider.value=color_hsv[HSV_name][1][2]
def update_green_Callback(value):
    # 点击绿色按钮,回调函数
    global HSV_name
    HSV_name = "green"
    H_min_slider.value=color_hsv[HSV_name][0][0]
    S_min_slider.value=color_hsv[HSV_name][0][1]
    V_min_slider.value=color_hsv[HSV_name][0][2]
    H_max_slider.value=color_hsv[HSV_name][1][0]
    S_max_slider.value=color_hsv[HSV_name][1][1]
    V_max_slider.value=color_hsv[HSV_name][1][2]
def update_blue_Callback(value):
    # 点击蓝色按钮,回调函数
    global HSV_name
    HSV_name = "blue"
    H_min_slider.value=color_hsv[HSV_name][0][0]
    S_min_slider.value=color_hsv[HSV_name][0][1]
    V_min_slider.value=color_hsv[HSV_name][0][2]
    H_max_slider.value=color_hsv[HSV_name][1][0]
    S_max_slider.value=color_hsv[HSV_name][1][1]
    V_max_slider.value=color_hsv[HSV_name][1][2]
def update_yellow_Callback(value):
    # 点击黄色按钮,回调函数
    global HSV_name
    HSV_name = "yellow"
    H_min_slider.value=color_hsv[HSV_name][0][0]
    S_min_slider.value=color_hsv[HSV_name][0][1]
    V_min_slider.value=color_hsv[HSV_name][0][2]
    H_max_slider.value=color_hsv[HSV_name][1][0]
    S_max_slider.value=color_hsv[HSV_name][1][1]
    V_max_slider.value=color_hsv[HSV_name][1][2]
# 点击按钮
HSV_update_red.on_click(update_red_Callback)
HSV_update_green.on_click(update_green_Callback)
HSV_update_blue.on_click(update_blue_Callback)
HSV_update_yellow.on_click(update_yellow_Callback)


# ### 模式切换控件

# In[6]:


# 模式设置
def write_file_Callback(value):
    global model
    model = 'Write_file'
def Color_Binary_Callback(value):
    global model,num
    if num%2==0: model="Binary"
    if num%2==1: model="General"
    num+=1
def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)
# 点击按钮
HSV_write_file.on_click(write_file_Callback)
Color_Binary.on_click(Color_Binary_Callback)
exit_button.on_click(exit_button_Callback)


# ### 摄像头函数

# In[7]:


def camera():
    global HSV_name,model,color_hsv,num
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2')
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            # 读取相机的每一帧
            _, img = capture.read()
            # 统一图像大小
            img = cv.resize(img, (640, 480))
            if HSV_name!=None:
                # 添加文字
                cv.putText(img, HSV_name, (int(img.shape[0] / 2)+40, 50), cv.FONT_HERSHEY_SIMPLEX, 2, color[random.randint(0, 254)], 2)
                # 实时修改颜色的HSV值
                color_hsv[HSV_name] = ((H_min_slider.value, S_min_slider.value, V_min_slider.value), (H_max_slider.value, S_max_slider.value, V_max_slider.value))
                # 轮廓检测
                img, binary = update_hsv.get_contours(img, HSV_name, color_hsv[HSV_name],color_hsv)
                if model == "Binary":
                    # 二值图显示
                    _, img = update_hsv.get_contours(img, HSV_name, color_hsv[HSV_name],color_hsv)
                if model == 'Write_file':
                    # 写入文件
                    try: write_HSV(HSV_path,color_hsv)
                    except Exception: print("File Path Error!!!")
                    # 所有颜色取反
                    cv.bitwise_not(img, img)
                    num=0
                    model="General"
            if model == 'Exit':
                # 退出程序
                cv.destroyAllWindows()
                capture.release()
                break
            # 图像格式转换
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:
            capture.release()


# ### 显示

# In[8]:


# 显示布局
display(controls_box,output)
# 开启图像线程
threading.Thread(target=camera, ).start()


# In[ ]:




