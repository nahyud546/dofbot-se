#!/usr/bin/env python
# coding: utf-8

# # 垃圾识别语音识别播报玩法

# In[1]:


import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_garbage_yolov11')


# ### 导入头文件

# In[2]:


#!/usr/bin/env python
# coding: utf-8

import os
import cv2 as cv
import threading
from time import sleep
import ipywidgets as widgets
from IPython.display import display
from speech_garbage import speech_garbage


# ### 创建实例,初始化参数

# In[3]:


# 创建获取目标实例
single_garbage = speech_garbage()
# 初始化模式
model = "General"


# ### 初始化机械臂位置

# In[4]:


from dofbot_utils.robot_controller import Robot_Controller
robot = Robot_Controller(port="/dev/ttyUSB0")
robot.move_look_map()


# In[5]:


from dofbot_utils.fps import FPS
fps = FPS()


# ### 创建控件

# In[6]:


button_layout      = widgets.Layout(width='320px', height='60px', align_self='center')
output = widgets.Output()
# 退出
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
controls_box = widgets.VBox([imgbox, exit_button], layout=widgets.Layout(align_self='center'))


# ### 退出模式控件

# In[7]:


def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)
exit_button.on_click(exit_button_Callback)


# ### 主程序

# In[8]:


def camera():
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
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:capture.release()


# ### 启动

# In[9]:


display(controls_box,output)
threading.Thread(target=camera, ).start()


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




