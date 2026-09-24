import pygame
import time
import platform  

# 初始化pygame
pygame.init()
pygame.joystick.init()

# 获取手柄数量
joystick_count = pygame.joystick.get_count()
print(f"检测到 {joystick_count} 个游戏手柄")

if joystick_count == 0:
    print("未检测到游戏手柄，请连接手柄后重试")
    exit()

# 初始化手柄
joysticks = []
for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    joysticks.append(joystick)
    
    print(f"\n手柄 {i}: {joystick.get_name()}")
    print(f"  轴数量: {joystick.get_numaxes()}")
    print(f"  方向键数量: {joystick.get_numhats()}")

system = platform.system()
print(f"platform: {system}")
if system == "Windows":
    # Windows下的映射
    button_names = {
        0: "A", 1: "B", 3: "X", 4: "Y",
        6: "LB", 7: "RB", 8: "LT", 9: "RT",
        10: "SELECT", 11: "START", 12: "左摇杆按下", 13: "右摇杆按下"
    }
    axis_names = [
        "左摇杆X", "",  
        "左摇杆Y", "右摇杆X",   
        "", "右摇杆Y"  
    ]
elif system == "Linux":
    # Linux下的映射
    button_names = {
        0: "A", 1: "B", 3: "X", 4: "Y",
        6: "LB", 7: "RB", 8: "LT", 9: "RT",
        10: "SELECT", 11: "START", 12: "左摇杆按下", 13: "右摇杆按下"
    }
    axis_names = [
        "左摇杆X", "左摇杆Y", "右摇杆X", "右摇杆Y", "RT", "LT"
    ]
else:
    # 其他系统默认映射（兼容处理）
    button_names = {}
    axis_names = []

# 仅打印绝对值大于此值的轴数据
AXIS_THRESHOLD = 0.1

try:
    while True:
        # 处理事件
        for event in pygame.event.get():
            # 按钮按下
            if event.type == pygame.JOYBUTTONDOWN:
                btn_name = button_names.get(event.button, f"未知按钮({event.button})")
                print(f"手柄 {event.joy}: 按钮 {btn_name} 按下")
            
            # 按钮释放
            elif event.type == pygame.JOYBUTTONUP:
                btn_name = button_names.get(event.button, f"未知按钮({event.button})")
                print(f"手柄 {event.joy}: 按钮 {btn_name} 释放")
            
            # 摇杆轴移动
            elif event.type == pygame.JOYAXISMOTION:
                if abs(event.value) > AXIS_THRESHOLD:
                    axis_name = axis_names[event.axis] if event.axis < len(axis_names) else f"轴{event.axis}"
                    # 仅Windows下屏蔽1/4轴，Linux下不屏蔽
                    if system == "Windows":
                        if event.axis != 1 and event.axis != 4:
                            print(f"手柄 {event.joy}: {axis_name} = {event.value:.2f}")
                    else:  # Linux系统
                        print(f"手柄 {event.joy}: {axis_name} = {event.value:.2f}")
            
            # 方向键移动
            elif event.type == pygame.JOYHATMOTION:
                if event.value != (0, 0):  # 只显示非零值
                    hat_dir = {
                        (1, 0): "右", (-1, 0): "左",
                        (0, 1): "上", (0, -1): "下",
                    }.get(event.value, event.value)
                    print(f"手柄 {event.joy}: 方向键 = {hat_dir}")
        
        time.sleep(0.01)  # 避免CPU占用过高

except KeyboardInterrupt:
    print("\n\n程序结束")
    pygame.quit()

