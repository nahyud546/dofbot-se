#!/usr/bin/env python
# coding: utf-8

# # 垃圾分拣播报玩法

# In[1]:


import sys
sys.path.append('/home/jetson/dofbot_pro/dofbot_garbage_yolov11')


# ### 导入头文件

# In[2]:


#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
import os
import cv2 as cv
import threading
from time import sleep
import ipywidgets as widgets
from IPython.display import display
from speech_garbage_identify import speech_garbage_identify


# ### 创建实例,初始化参数

# In[3]:


# 创建获取目标实例
single_garbage = speech_garbage_identify()
# 初始化模式
model = "General"


# ### 初始化机械臂位置

# In[4]:


single_garbage.init_robot_joint()


# ### 创建控件

# In[5]:


button_layout      = widgets.Layout(width='320px', height='60px', align_self='center')
output = widgets.Output()
# 退出
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
controls_box = widgets.VBox([imgbox, exit_button], layout=widgets.Layout(align_self='center'))


# ### 退出模式控件

# In[6]:


def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)
exit_button.on_click(exit_button_Callback)


# ### 主程序

# In[7]:


def camera():
    # 打开摄像头
    capture = cv.VideoCapture('/dev/video2', cv.CAP_V4L2)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            # 读取相机的每一帧
            _, img = capture.read()
            # 统一图像大小
            img = cv.resize(img, (640, 480))
            img = single_garbage.single_garbage_run(img)
            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except KeyboardInterrupt:capture.release()


# ### 启动

# In[8]:


display(controls_box,output)
threading.Thread(target=camera, ).start()


# In[ ]:





# In[ ]:




