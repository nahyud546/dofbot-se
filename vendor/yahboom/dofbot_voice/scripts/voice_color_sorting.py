from Speech_Lib import Speech
import sys
import cv2 as cv
import threading
import time
import traceback
import Arm_Lib
import os
import numpy as np
from speech_identify_target import identify_GetTarget
from dofbot_utils.fps import FPS
from dofbot_utils.dofbot_config import *
from dofbot_sorting_3d.color_common import write_HSV, read_HSV

# 摄像头参数
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
# 配置文件路径
_VOICE_DIR = os.path.join(os.environ.get("ROBOT_ARM_ROOT", str(__import__("pathlib").Path(__file__).resolve().parents[3])), "vendor/yahboom/dofbot_voice/scripts")
HSV_CONFIG_PATH = os.environ.get("HSV_CONFIG", os.path.join(_VOICE_DIR, "HSV_config.txt"))
XYT_CONFIG_PATH = os.environ.get("XYT_CONFIG", os.path.join(_VOICE_DIR, "XYT_config.txt"))
# 颜色列表 
COLOR_LIST = ['red', 'green', 'blue', 'yellow']  

# 语音命令码映射
VOICE_CMD = {
    "start_detect": 88,                # 开始检测  Start detection
    "color_red": 46,                   # 红色播报  Red announcement
    "color_green": 49,                 # 绿色播报  Green announcement
    "color_blue": 48,                  # 蓝色播报  Blue announcement
    "color_yellow": 47,                # 黄色播报  Yellow announcement
    "prompt_select_color": 89,         # 提示选择颜色  Prompt to select color
    "sort_complete_confirm": 92,       # 分拣确认  Confirm sorting
    "re_detect": 45,                   # 重新检测  Re-detect
    "queue_cleared": 84,               # 队列已清空  Queue cleared
    "prompt_queue_info": 92,           # 播报队列信息  Announce queue info
    
    "recognize_red": 87,               # 识别红色指令  Recognize red command
    "recognize_green": 88,             # 识别绿色指令  Recognize green command
    "recognize_blue": 89,              # 识别蓝色指令  Recognize blue command
    "recognize_yellow": 90,            # 识别黄色指令  Recognize yellow command
    "recognize_cancel": 91,            # 取消选择指令  Cancel selection command
    "recognize_confirm_grap": 92,      # 确认抓取指令  Confirm grasp command
    "recognize_clear_queue": 85,       # 清空队列指令  Clear queue command
    "recognize_stop_select": 86,       # 停止选择指令  Stop selection command
    "recognize_begin_sort": 92         # 开始分拣指令  Begin sorting command
}

# 初始化硬件参数
mySpeech = Speech()
target = identify_GetTarget()
calibration = Arm_Calibration()

# 初始HSV颜色范围
color_hsv = {
    "red": [[(0, 68, 184), (10, 204, 255)], [(170, 129, 0), (180, 204, 255)]],  
    "green": [(34, 76, 0), (77, 252, 229)],
    "blue": [(85, 149, 87), (171, 255, 179)],
    "yellow": [(19, 107, 229), (125, 253, 255)]
}

dp = []                      # 校准参数  Calibration parameters
xy = [90, 120]               # 初始坐标  Initial coordinates
threshold = 130              # 阈值  Threshold
model = "General"            # 运行模式  Operation mode
is_running = True            # 程序运行标志  Program running flag
fps = FPS()                  # FPS计数器  FPS counter
msg = {}                     # 检测到的颜色和位置  Detected colors and positions
detected_colors = []         # 已检测颜色列表  List of detected colors
sort_queue = []              # 分拣队列  Sorting queue
result = 0                   # 语音识别结果  Speech recognition result
is_sorting = False           # 分拣中标志  Sorting in progress flag
current_sorting_thread = None# 当前分拣线程  Current sorting thread

# HSV学习相关参数  HSV Learning Related Parameters
hsv_learning_mode = False    # HSV学习模式标志  HSV learning mode flag
select_flags = False         # 鼠标选择标志  Mouse selection flag
roi_hsv_range = ()           # 选中区域HSV范围  HSV range of selected ROI
Roi_init = ()                # 选中区域坐标  Selected ROI coordinates
Mouse_XY = (0, 0)            # 鼠标位置  Mouse position
cols, rows = 0, 0            # 选中区域行列  ROI rows/columns
current_learning_color = ""  # 当前学习颜色  Current learning color

def load_config():
    # 加载HSV配置  Load HSV config
    try:
        hsv_files = {
            "red": "red_colorHSV.text",
            "green": "green_colorHSV.text",
            "blue": "blue_colorHSV.text",
            "yellow": "yellow_colorHSV.text"
        }
        for color, filepath in hsv_files.items():
            if os.path.exists(filepath):
                color_hsv[color] = read_HSV(filepath)
    except Exception as e:
        print(f" Failed to read HSV_config: {e}")
    
    # 加载XYT配置  Load XYT config
    try:
        global xy, threshold
        xy, threshold = read_XYT(XYT_CONFIG_PATH)
    except Exception as e:
        print(f"xFailed to read XYT_config: {e}")

def play_voice(cmd_code, desc=""):
    """
    播放语音提示  Play voice prompt
    :param cmd_code: 语音命令码  Voice command code
    :param desc: 命令描述  Command description
    """
    try:
        mySpeech.void_write(cmd_code)
        time.sleep(0.5)
        if desc:
            print(f"{desc}")
    except Exception as e:
        print(f"Voice play failed: {e}")

def parse_speech_result(result_code):
    """
    解析语音识别结果码  Parse speech recognition result code
    :param result_code: 语音识别返回码  Speech recognition return code
    :return: 转换后的命令字符串  Converted command string
    """
    code2cmd = {
        VOICE_CMD["recognize_red"]: "red",
        VOICE_CMD["recognize_green"]: "green",
        VOICE_CMD["recognize_blue"]: "blue",
        VOICE_CMD["recognize_yellow"]: "yellow",
        VOICE_CMD["recognize_cancel"]: "cancel",
        VOICE_CMD["recognize_confirm_grap"]: "confirm",
        VOICE_CMD["recognize_clear_queue"]: "clear_queue",
        VOICE_CMD["recognize_stop_select"]: "stop_select",
        VOICE_CMD["recognize_begin_sort"]: "begin_sort"
    }
    return code2cmd.get(result_code)

def update_sort_queue(color):
    """
    更新分拣队列 Update sorting queue
    :param color: 要添加颜色  Color to add
    """
    global sort_queue
    if color in detected_colors and color not in sort_queue:
        sort_queue.append(color)

def clear_sort_queue():
    """
    清空分拣队列  Clear sorting queue
    """
    global sort_queue
    sort_queue = []
    play_voice(VOICE_CMD["queue_cleared"], "Sorting queue cleared")

def begin_sorting():
    """
    开始分拣任务  Begin sorting task
    """
    global model, is_sorting, sort_queue, msg, current_sorting_thread
    if not sort_queue:
        return
    is_sorting = True
    model = "Grap"
    play_voice(VOICE_CMD["sort_complete_confirm"], f"Begin sorting {len(sort_queue)} colors")
    print(f"Starting task, queue length: {len(sort_queue)}")

def execute_grap():
    """
    执行抓取任务
    """
    global sort_queue, msg, model, is_sorting
    try:
        while is_sorting and sort_queue:
            current_color = sort_queue[0]
            if current_color in msg:
                current_pos = msg[current_color]
                play_voice(VOICE_CMD[f"color_{current_color}"], f"Begin grasping {current_color}")
                try:
                    target.target_run({current_color: current_pos}, xy)
                    sort_queue.pop(0)
                except Exception as e:
                    print(f"{current_color} grasp failed: {e}")
                    time.sleep(1)
            else:
                print(f" {current_color} not in detection results")
                sort_queue.pop(0)
    except Exception as e:
        print(f"Sorting thread exception: {e}")
    finally:
        is_sorting = False
        if not sort_queue:
            model = "General"
        print(f"Task completed")

def hsv_learning_Callback():
    """
    进入HSV学习模式  Enter HSV learning mode
    """
    global model, hsv_learning_mode
    model = 'HSV_Learning'
    hsv_learning_mode = True
    print(f"Switched to: {model}")

def onMouse(event, x, y, flags, param):
    """
    鼠标事件回调  Mouse event callback function
    """
    global select_flags, Mouse_XY, cols, rows, Roi_init, roi_hsv_range, hsv_learning_mode, current_learning_color
    if not hsv_learning_mode:
        return
    if event == 1:  # 左键按下  Left button pressed
        select_flags = True
        Mouse_XY = (x, y)
    if event == 4:  # 左键释放  Left button released
        select_flags = False
        calculate_roi_hsv()
    if select_flags == True:
        cols = min(Mouse_XY[0], x), min(Mouse_XY[1], y)
        rows = max(Mouse_XY[0], x), max(Mouse_XY[1], y)
        Roi_init = (cols[0], cols[1], rows[0], rows[1])

def calculate_roi_hsv():
    """
    计算选中区域的HSV范围  Calculate HSV range of selected area
    """
    global roi_hsv_range, image, cols, rows
    try:
        roi = image[cols[1]:rows[1], cols[0]:rows[0]]
        hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        
        # 收集H、S、V值
        H = hsv_roi[:, :, 0].flatten()
        S = hsv_roi[:, :, 1].flatten()
        V = hsv_roi[:, :, 2].flatten()
        
        # 计算H、S、V的最小最大值
        h_min = int(np.min(H))
        h_max = int(np.max(H))
        s_min = int(np.min(S))
        v_min = int(np.min(V))
        
        if h_max + 10 > 179:
            h_max = 179
        else:
            h_max += 10
        if h_min - 10 < 0:
            h_min = 0
        else:
            h_min -= 10
        
        # S和V的最大值固定为255
        s_max = 255
        v_max = 255
        

        
        roi_hsv_range = ((h_min, s_min, v_min), (h_max, s_max, v_max))
        print(f"HSV Range: H[{h_min}-{h_max}], S[{s_min}-{s_max}], V[{v_min}-{v_max}]")
    except Exception as e:
        print(f"Error calculating HSV range: {e}")

def save_hsv_to_file(color_name):
    """
    保存HSV范围到文件  Save HSV range to file
    :param color_name: 颜色名称  Color name
    """
    global roi_hsv_range, color_hsv, current_learning_color
    try:
        write_HSV(f"{color_name}_colorHSV.text", roi_hsv_range)
        color_hsv[color_name] = roi_hsv_range
        print(f"HSV range for {color_name} saved successfully")
        current_learning_color = ""
    except Exception as e:
        print(f"Error saving HSV range: {e}")

def camera():
    global color_hsv, model, dp, msg, detected_colors, sort_queue, result, is_running, is_sorting, current_sorting_thread
    global hsv_learning_mode, select_flags, roi_hsv_range, Roi_init, Mouse_XY, cols, rows, image, current_learning_color
    
    # 初始化摄像头  Initialize camera
    capture = cv.VideoCapture('/dev/video2')
    capture.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    if not capture.isOpened():
        print(f"Camera open failed")
        return
        
    # 设置鼠标回调  Set mouse callback
    cv.namedWindow("Color Sorting (Voice Control)")
    cv.setMouseCallback("Color Sorting (Voice Control)", onMouse)

    while is_running and capture.isOpened():
        try:
            # 读取语音指令  Read speech command
            try:
                result = mySpeech.speech_read()
            except Exception as e:
                result = 0
            for i in range(15):
                #  读取摄像头帧  Read camera frame
                ret, img = capture.read()
            if not ret or img is None:
                pass
            fps.update_fps()
            image = img.copy()

            #校准模式  Calibration mode
            if model == 'Calibration':
                dp, img = calibration.calibration_map(img, xy, threshold)
                if len(dp) != 0:
                    model = "Detection2"
                    continue

            # 透视变换  Perspective transform
            #if len(dp) != 0:
                #img = calibration.Perspective_transform(dp, img)

            #  颜色检测模式  Color detection mode
            if model == 'Detection2':
                # 清空历史数据  Clear historical data
                msg = {}
                detected_colors = []
                sort_queue = []
                is_sorting = False
                if current_sorting_thread and current_sorting_thread.is_alive():
                    current_sorting_thread.join(timeout=1)
                
                # 执行颜色检测  Execute color detection
                img, msg, _, _, _, _ = target.select_color(img, color_hsv, COLOR_LIST)
                play_voice(VOICE_CMD["start_detect"], " Begin color detection")

                # 收集检测到的颜色  Collect detected colors
                for color in COLOR_LIST:
                    if color in msg:
                        detected_colors.append(color)
                        play_voice(VOICE_CMD[f"color_{color}"], f" Detected {color}")

                # 切换模式  Switch mode
                if detected_colors:
                    play_voice(VOICE_CMD["prompt_select_color"], f"Detected {len(detected_colors)} colors: {','.join(detected_colors)}")
                    model = "ColorSelection"
                else:
                    model = "General"

            # 颜色选择模式  Color selection mode
            if model == 'ColorSelection' and result != 0:
                speech_cmd = parse_speech_result(result)
                if speech_cmd:
                    # 颜色选择  Color selection
                    if speech_cmd in COLOR_LIST:
                        update_sort_queue(speech_cmd)
                    # 取消选择  Cancel selection
                    elif speech_cmd == "cancel" and sort_queue:
                        removed_color = sort_queue.pop()
                    # 清空队列  Clear queue
                    elif speech_cmd == "clear_queue":
                        clear_sort_queue()
                    # 停止选择  Stop selection
                    elif speech_cmd == "stop_select" and sort_queue:
                        play_voice(VOICE_CMD["sort_complete_confirm"], f"Sorting order confirmed: {', '.join(sort_queue)}")
                        model = "toGrap"
                    # 重新检测  Re-detect
                    elif speech_cmd == "re_detect":
                        model = "Detection2"
                    # 开始分拣  Begin sorting
                    elif speech_cmd == "begin_sort":
                        begin_sorting()
                result = 0

            # 进行颜色检测和绘制
            if model != 'HSV_Learning' and model != 'Calibration':
                img, current_msg, _, _, _, _ = target.select_color(img, color_hsv, COLOR_LIST)
                # 更新msg用于分拣，
                if model in ['ColorSelection', 'toGrap']:
                    msg = current_msg

            #  分拣确认模式  Sort confirmation mode
            if model == 'toGrap' and result != 0:
                speech_cmd = parse_speech_result(result)
                if speech_cmd in ["confirm", "begin_sort"]:
                    result = 0
                    begin_sorting()
                elif speech_cmd == "re_detect":
                    result = 0
                    model = "Detection2"

            # 执行分拣  Execute sorting
            if model == 'Grap' and is_sorting and sort_queue:
                if current_sorting_thread and current_sorting_thread.is_alive():
                    if not is_sorting:
                        current_sorting_thread = None
                else:
                    current_sorting_thread = threading.Thread(target=execute_grap, daemon=True)
                    current_sorting_thread.start()

            #检查线程状态  Check thread status
            if current_sorting_thread and not current_sorting_thread.is_alive():
                if is_sorting:
                    is_sorting = False
                    model = "General"
                current_sorting_thread = None

            #HSV学习模式绘制  HSV learning mode drawing
            if model == 'HSV_Learning':
                if select_flags:
                    cv.rectangle(img, (cols[0], cols[1]), (rows[0], rows[1]), (0, 255, 0), 2)
                if len(roi_hsv_range) > 0:
                    hsv_text = f"HSV: {roi_hsv_range[0]} - {roi_hsv_range[1]}"
                    cv.putText(img, hsv_text, (10, 460), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if current_learning_color:
                    color_text = f"Learning: {current_learning_color}"
                    cv.putText(img, color_text, (10, 430), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 绘制状态信息  Draw status info
            queue_text = f"Queue: {sort_queue}" if sort_queue else "Queue:Empty"
            sorting_text = f"Sorting: ACTIVE" if is_sorting else "Sorting:IDLE"
            cv.putText(img, queue_text, (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(img, sorting_text, (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示图像和按键处理  Show image and key processing
            cv.imshow("Color Sorting (Voice Control)", img)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                is_running = False
                is_sorting = False
                break
            elif key == ord('s') and sort_queue and not is_sorting: 
                begin_sorting()
            elif key == ord('h'):
                hsv_learning_Callback()
            elif key in [ord('r'), ord('g'), ord('b'), ord('y')] and hsv_learning_mode and len(roi_hsv_range) > 0:
                color_map = {ord('r'): "red", ord('g'): "green", ord('b'): "blue", ord('y'): "yellow"}
                save_hsv_to_file(color_map[key])
                current_learning_color = color_map[key]
            elif key == ord('e') and hsv_learning_mode:
                hsv_learning_mode = False
                model = "General"

        except Exception as e:
            time.sleep(0.1)
            continue

    capture.release()
    cv.destroyAllWindows()
    try:
        mySpeech.close()
    except Exception as e:
        print(f"Speech serial port close failed: {e}")

if __name__ == "__main__":
    try:
        # 初始化配置和机械臂  Initialize config and arm
        load_config()
        arm = Arm_Lib.Arm_Device(com="/dev/ttyUSB0")
        joints_0 = [xy[0], xy[1], 0, 0, 90, 30]
        arm.Arm_serial_servo_write6_array(joints_0, 1000)
        time.sleep(1)
        model = 'Calibration'
        is_sorting = False
        current_sorting_thread = None

        # 启动相机线程  Start camera thread
        cam_thread = threading.Thread(target=camera, daemon=True)
        cam_thread.start()

        # 等待退出  Wait for exit
        input("Press Enter to exit...\n")
        is_running = False
        is_sorting = False
        cam_thread.join(timeout=3)

    except KeyboardInterrupt:
        is_running = False
        is_sorting = False
        
    finally:
        cv.destroyAllWindows()