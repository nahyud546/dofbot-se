# -*- coding: utf-8 -*-
import pygame
import time
import platform
import threading
import inspect
import ctypes

# Try to import robotic arm library
# 尝试导入机械臂库
try:
    from Arm_Lib import Arm_Device
    Arm = Arm_Device()
    ARM_AVAILABLE = True
    print("Arm_Lib init success!")
except ImportError:
    ARM_AVAILABLE = False
    print("Arm_Lib import failed.")

# Initialize Pygame and Joystick
# 初始化Pygame和手柄
pygame.init()
pygame.joystick.init()

# Get joystick count
# 获取手柄数量
joystick_count = pygame.joystick.get_count()
print(f"Detected {joystick_count} joystick(s).")

if joystick_count == 0:
    print("No joystick detected, program exit.")
    exit()

# Initialize joysticks
# 初始化所有手柄
joysticks = []
for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    joysticks.append(joystick)
    
    print(f"\nJoystick {i}: {joystick.get_name()}")
    print(f"  Axes count: {joystick.get_numaxes()}")
    print(f"  Buttons count: {joystick.get_numbuttons()}")
    print(f"  Hats count: {joystick.get_numhats()}")

# System detection
# 系统检测
system = platform.system()
print(f"Operating System: {system}")

# Button mapping
# 按键映射
button_names = {
    0: "A", 1: "B", 3: "X", 4: "Y",
    6: "LB", 7: "RB", 8: "LT", 9: "RT",
    10: "SELECT", 11: "START"
}

# Axis mapping
# 轴映射
if system == "Windows":
    axis_names = ["Left Stick X", "", "Left Stick Y", "Right Stick X", "", "Right Stick Y"]
elif system == "Linux":
    axis_names = ["Left Stick X", "Left Stick Y", "Right Stick X", "Right Stick Y", "RT", "LT"]
else:
    axis_names = []

# Axis threshold
# 轴阈值
AXIS_THRESHOLD = 0.1

# RGB预设颜色（R/G/B: 0-255）
# RGB Preset Colors (R/G/B: 0-255)
RGB_COLORS = [
    (255, 0, 0),    # Red
    (0, 255, 0),    # Green
    (0, 0, 255),    # Blue
    (255, 255, 0),  # Yellow
    (0, 0, 0)       # Off
]
current_rgb_index = 0  # Current RGB color index / 当前RGB颜色索引

def _async_raise(tid, exctype):
    """
    Asynchronously raise an exception in a thread
    异步在线程中抛出异常
    :param tid: Thread ID / 线程ID
    :param exctype: Exception type / 异常类型
    """
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        
def stop_thread(thread):
    """
    Stop the specified thread
    停止指定线程
    :param thread: Target thread / 目标线程
    """
    _async_raise(thread.ident, SystemExit)

# ---------------------- RGB Switch Function ----------------------
def switch_rgb(direction):
    """
    Switch RGB color (cycle left/right)
    切换RGB颜色（左/右）
    :param direction: Switch direction (left/right) 
    """
    global current_rgb_index
    rgb_count = len(RGB_COLORS)
    
    if direction == "left":
        current_rgb_index = (current_rgb_index - 1) % rgb_count
    elif direction == "right":
        current_rgb_index = (current_rgb_index + 1) % rgb_count
    
    current_color = RGB_COLORS[current_rgb_index]
    if ARM_AVAILABLE:
        try:
            Arm.Arm_RGB_set(*current_color)  # Adapt to actual RGB interface / 适配实际RGB设置接口
        except AttributeError:
            pass
# ---------------------- Servo Step Adjust Function ----------------------
def adjust_servo_step(current_step, direction):
    """
    Adjust global servo movement step
    调节全局舵机移动步长
    :param current_step: Current step / 当前步长
    :param direction: Adjust direction (up=increase/down=decrease) / 调节方向（up=增大/down=减小）
    :return: New step / 新步长
    """
    step_min = 0.5    #  最小步长
    step_max = 2.5    #  最大步长
    
    if direction == "up":
        new_step = min(step_max, current_step + 0.1)
    elif direction == "down":
        new_step = max(step_min, current_step - 0.1)
    else:
        new_step = current_step
    
    new_step = round(new_step, 1)
    print(f"Servo step adjusted: {current_step}-->{new_step} (range: {step_min}-{step_max})")
    return new_step

# ---------------------- Main Handle Control Function ----------------------
def Arm_Handle():
    """
    Main function for joystick control of robotic arm
    手柄控制机械臂函数
    """
    s_time = 500
    s_step = 1.0  # Initial global step / 初始全局步长
    angle_1 = angle_2 = angle_3 = angle_4 = angle_5 = angle_6 = 90.0
    
    # State cache (prevent repeated triggering)
    # 状态缓存（防止重复触发）
    select_pressed = False
    hat_pressed = {"up": False, "down": False, "left": False, "right": False}
    
    if joystick_count == 0:
        return
    
    joystick = joysticks[0]
    try:
        while True:
            # ---------------------- Process Pygame Events ----------------------
            for event in pygame.event.get():
                # Button down events
                # 按键按下事件
                if event.type == pygame.JOYBUTTONDOWN:
                    btn_idx = event.button 

                    # A Button (0) - Servo 6 clamp
                    # A键 (0) - 6号舵机夹紧
                    if btn_idx == 0:
                        angle_6 += s_step
                        angle_6 = max(0, min(180, angle_6))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(6, angle_6, s_time)
                        else:
                            print(f"Servo 6 angle is : {angle_6:.1f}°")
                    
                    # B Button (1) - Servo 6 release
                    # B键 (1) - 6号舵机放松
                    elif btn_idx == 1:
                        angle_6 -= s_step
                        angle_6 = max(0, min(180, angle_6))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(6, angle_6, s_time)
                        else:
                            print(f"Servo 6 angle is: {angle_6:.1f}°")
                    
                    # X Button (3) - Servo 5 move
                    # X键 (3) - 5号舵机移动
                    elif btn_idx == 3:
                        angle_5 += s_step
                        angle_5 = max(0, min(180, angle_5))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(5, angle_5, s_time)
                        else:
                            print(f"Servo 5 angle is: {angle_5:.1f}°")
                    # Y Button (4) - Servo 5 move
                    # Y键 (4) - 5号舵机移动
                    elif btn_idx == 4:
                        angle_5 -= s_step
                        angle_5 = max(0, min(180, angle_5))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(5, angle_5, s_time)
                        else:
                            print(f"Servo 5 angle is: {angle_5:.1f}°")
                    # LB Button (6) - Servo 3 move forward
                    # LB键 (6) - 3号舵机向前
                    elif btn_idx == 6:
                        angle_3 += s_step
                        angle_3 = max(0, min(180, angle_3))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(3, angle_3, s_time)
                        else:
                            print(f"Servo 3 angle is: {angle_3:.1f}°")
                    
                    # LT Button (8) - Servo 3 move backward
                    # LT键 (8) - 3号舵机向后
                    elif btn_idx == 8:
                        angle_3 -= s_step
                        angle_3 = max(0, min(180, angle_3))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(3, angle_3, s_time)
                        else:
                            print(f"Servo 3 angle is: {angle_3:.1f}°")
                    
                    # RT Button (9) - Servo 4 move forward
                    # RT键 (9) - 4号舵机向前
                    elif btn_idx == 9:
                        angle_4 += s_step
                        angle_4 = max(0, min(180, angle_4))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(4, angle_4, s_time)
                        else:
                            print(f"Servo 4 angle is: {angle_4:.1f}°")
                    
                    # RB Button (7) - Servo 4 move backward
                    # RB键 (7) - 4号舵机向后
                    elif btn_idx == 7:
                        angle_4 -= s_step
                        angle_4 = max(0, min(180, angle_4))
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write(4, angle_4, s_time)
                        else:
                            print(f"Servo 4 angle is: {angle_4:.1f}°")
                    
                    # START Button (11) - Buzzer
                    # START键 (11) - 蜂鸣器
                    elif btn_idx == 11:
                        if ARM_AVAILABLE:
                            Arm.Arm_Buzzer_On(2)
                            time.sleep(0.5)
                            Arm.Arm_Buzzer_On(0)
                 
                    # SELECT Button (10) - Reset all servos
                    # SELECT键 (10) - 重置所有舵机
                    elif btn_idx == 10 and not select_pressed:
                        angle_1 = angle_2 = angle_3 = angle_4 = angle_5 = angle_6 = 90.0
                        if ARM_AVAILABLE:
                            Arm.Arm_serial_servo_write6(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6, s_time)
                        print("All servos reset to 90.0°.")
                        select_pressed = True
                
                # D-Pad events (step adjustment + RGB switch)
                # 方向键事件（步长调节+RGB切换）
                elif event.type == pygame.JOYHATMOTION:
                    hat_value = event.value
                    if hat_value != (0, 0):
                        hat_dir = {
                            (1, 0): "Right", (-1, 0): "Left",
                            (0, 1): "Up", (0, -1): "Down",
                        }.get(hat_value, hat_value)
                        print(f"Joystick {event.joy}: D-Pad = {hat_dir}")
                        
                        # Up/Down: Adjust global servo step
                        # 上下方向：调节全局舵机步长
                        if hat_value == (0, 1) and not hat_pressed["up"]:
                            s_step = adjust_servo_step(s_step, "up")
                            hat_pressed["up"] = True
                        elif hat_value == (0, -1) and not hat_pressed["down"]:
                            s_step = adjust_servo_step(s_step, "down")
                            hat_pressed["down"] = True
                        
                        # Left/Right: Switch RGB color
                        # 左右方向：切换RGB颜色
                        elif hat_value == (1, 0) and not hat_pressed["right"]:
                            switch_rgb("right")
                            hat_pressed["right"] = True
                        elif hat_value == (-1, 0) and not hat_pressed["left"]:
                            switch_rgb("left")
                            hat_pressed["left"] = True
                    else:
                        # Reset D-Pad state cache
                        # 方向键复位：清空状态缓存
                        hat_pressed = {"up": False, "down": False, "left": False, "right": False}
                

            # Servo 1 - Left Stick X (axes[0])
            # 1号舵机 - 左摇杆X (axes[0])
            axis_0 = joystick.get_axis(0)
            if abs(axis_0) > AXIS_THRESHOLD:
                angle_1 += (s_step if axis_0 > 0 else -s_step)
                angle_1 = max(0, min(180, angle_1))
                if ARM_AVAILABLE:
                    Arm.Arm_serial_servo_write(1, angle_1, s_time)
                else:
                    print(f"Servo 1: {angle_1:.1f}° (Left Stick X: {axis_0:.2f})")
                time.sleep(0.01)
            
            # Servo 2 - Left Stick Y (axes[1])
            # 2号舵机 - 左摇杆Y (axes[1])
            axis_1 = joystick.get_axis(1)
            if abs(axis_1) > AXIS_THRESHOLD:
                angle_2 += (-s_step if axis_1 > 0 else +s_step)
                angle_2 = max(0, min(180, angle_2))
                if ARM_AVAILABLE:
                    Arm.Arm_serial_servo_write(2, angle_2, s_time)
                else:
                    print(f"Servo 2: {angle_2:.1f}° (Left Stick Y: {axis_1:.2f})")
                time.sleep(0.01)
            
            # Servo 6 - Right Stick X (axes[2])
            # 6号舵机 - 右摇杆X (axes[2])
            axis_2 = joystick.get_axis(2)
            if abs(axis_2) > 0.5:
                angle_6 += (-s_step if axis_2 > 0.5 else s_step)
                angle_6 = max(0, min(180, angle_6))
                if ARM_AVAILABLE:
                    Arm.Arm_serial_servo_write(6, angle_6, s_time)
                else:
                    print(f"Servo 6: {angle_6:.1f}° (Right Stick X: {axis_2:.2f})")
                time.sleep(0.01)
            
            # Servo 5 - Right Stick Y (axes[3])
            # 5号舵机 - 右摇杆Y (axes[3])
            axis_3 = joystick.get_axis(3)
            if abs(axis_3) > 0.5:
                angle_5 += (s_step if axis_3 > 0.5 else -s_step)
                angle_5 = max(0, min(180, angle_5))
                if ARM_AVAILABLE:
                    Arm.Arm_serial_servo_write(5, angle_5, s_time)
                else:
                    print(f"Servo 5: {angle_5:.1f}° (Right Stick Y: {axis_3:.2f})")
                time.sleep(0.01)
            
            # Reduce CPU usage
            # 降低CPU占用
            time.sleep(0.005)
    
    except KeyboardInterrupt:
        print("\n\nProgram terminated.")
        # Turn off RGB when exiting
        # 退出时关闭RGB灯
        if ARM_AVAILABLE:
            try:
                Arm.Arm_RGB_set(0, 0, 0)
            except:
                pass
        pygame.quit()

# ---------------------- Start Thread ----------------------
if __name__ == "__main__":
    # Create and start control thread
    # 创建并启动控制线程
    thread = threading.Thread(target=Arm_Handle)
    thread.daemon = True
    thread.start()
    
    try:
        while thread.is_alive():
            thread.join(0.1)
    except KeyboardInterrupt:
        print("\nProgram exiting.")
        # Turn off RGB when exiting
        # 退出时关闭RGB灯
        if ARM_AVAILABLE:
            try:
                Arm.Arm_RGB_set(0, 0, 0)
            except:
                pass
        if thread.is_alive():
            stop_thread(thread)
        pygame.quit()    
