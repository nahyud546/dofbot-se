from Speech_Lib import Speech
import sys
import cv2 as cv
import threading
import time
import traceback
import Arm_Lib
from speech_identify_target import identify_GetTarget
from dofbot_utils.fps import FPS
from dofbot_utils.dofbot_config import *

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
JOINT_RESET_POS = [90, 106, 0, 0, 90, 30]  # 机械臂复位位置
HSV_CONFIG_PATH = "HSV_config.txt"
XYT_CONFIG_PATH = "XYT_config.txt"
COLOR_LIST = ['red', 'green', 'blue', 'yellow']  # 支持的颜色列表
VOICE_CMD = {
    "start_detect": 88,
    "color_red": 46,
    "color_green": 49,
    "color_blue": 48,
    "color_yellow": 47,
    "prompt_select_color": 93,      # 提示选择颜色
    "sort_complete_confirm": 92,    # 排序完成确认
    "re_detect": 45,                # 重新识别
    "queue_cleared": 84,            # 队列已清空
    "prompt_queue_info": 92,        # 播报队列信息
    
    # 语音识别结果码
    "recognize_red": 87,
    "recognize_green": 88,
    "recognize_blue": 89,
    "recognize_yellow": 90,
    "recognize_cancel": 91,
    "recognize_confirm_grap": 92,
    "recognize_re_detect": 109,
    "recognize_clear_queue": 85,    # 新增：清空队列
    "recognize_stop_select": 86,    # 新增：停止选择
    "recognize_begin_sort": 92      # 新增：开始分拣指令
}

mySpeech = Speech()
target = identify_GetTarget()
calibration = Arm_Calibration()
Arm = Arm_Lib.Arm_Device()  # 机械臂实例

# 颜色HSV配置
color_hsv = {
    "red": [[(0, 0, 0), (18, 255, 255)], [(140, 0, 0), (180, 255, 255)]],
    "green": [(25, 0, 0), (95, 255, 255)],
    "blue": [(87, 205, 89), (255, 228, 255)],
    "yellow": [(10, 0, 0), (40, 255, 255)]
}

dp = []  # 标定参数
xy = [90, 106]  # 初始坐标
threshold = 120  # 阈值
model = "General"  # 运行模式
is_running = True  # 程序运行标志
fps = FPS()  # 帧率统计
msg = {}  # 识别到的颜色及位置
detected_colors = []  # 识别到的颜色列表
sort_queue = []  # 分拣队列
result = 0  # 语音指令识别结果
is_sorting = False  # 是否正在分拣中
current_sorting_thread = None  # 当前分拣线程

def load_config():
    """加载HSV和XYT配置文件"""
    # 加载HSV配置
    try:
        read_HSV(HSV_CONFIG_PATH, color_hsv)
    except Exception as e:
        print(f"❌ 读取HSV_config失败: {e}")
    # 加载XYT配置
    try:
        global xy, threshold
        xy, threshold = read_XYT(XYT_CONFIG_PATH)
    except Exception as e:
        print(f"❌ 读取XYT_config失败: {e}")

def play_voice(cmd_code, desc=""):
    """
    语音播报
    :param cmd_code: 语音指令码
    :param desc: 指令描述
    """
    try:
        mySpeech.void_write(cmd_code)
        time.sleep(0.5)
        if desc:
            print(f"🔊 语音播报: {desc}")
    except Exception as e:
        print(f"❌ 语音播报异常[{desc}]: {e}")

def parse_speech_result(result_code):
    """
    解析语音识别结果码
    :param result_code: 语音识别返回码
    :return: 指令字符串/None
    """
    code2cmd = {
        # 颜色选择指令
        VOICE_CMD["recognize_red"]: "red",
        VOICE_CMD["recognize_green"]: "green",
        VOICE_CMD["recognize_blue"]: "blue",
        VOICE_CMD["recognize_yellow"]: "yellow",
        # 功能指令
        VOICE_CMD["recognize_cancel"]: "cancel",
        VOICE_CMD["recognize_confirm_grap"]: "confirm",
        VOICE_CMD["recognize_re_detect"]: "re_detect",
        VOICE_CMD["recognize_clear_queue"]: "clear_queue",
        VOICE_CMD["recognize_stop_select"]: "stop_select",
        VOICE_CMD["recognize_begin_sort"]: "begin_sort"  # 新增：开始分拣
    }
    return code2cmd.get(result_code)

def update_sort_queue(color):
    """更新分拣队列并播报"""
    global sort_queue
    if color in detected_colors and color not in sort_queue:
        sort_queue.append(color)
        play_voice(VOICE_CMD["prompt_queue_info"], 
                   f"已添加{color}到分拣队列，当前顺序：{', '.join(sort_queue)}")
        print(f"📌 分拣队列更新: {sort_queue}")
    elif color in sort_queue:
        play_voice(VOICE_CMD["prompt_queue_info"], f"{color}已在分拣队列中")
    else:
        play_voice(VOICE_CMD["prompt_queue_info"], f"未识别到{color}，无法添加")

def clear_sort_queue():
    """清空分拣队列"""
    global sort_queue
    sort_queue = []
    play_voice(VOICE_CMD["queue_cleared"], "分拣队列已清空")
    print("📌 分拣队列已清空")

def begin_sorting():
    """开始分拣任务"""
    global model, is_sorting, sort_queue, msg, current_sorting_thread
    
    if not sort_queue:
        play_voice(VOICE_CMD["prompt_queue_info"], "分拣队列为空，无法开始分拣")
        return
    
    # 如果已有分拣线程在运行，先等待它完成
    if current_sorting_thread and current_sorting_thread.is_alive():
        play_voice(VOICE_CMD["prompt_queue_info"], "正在分拣中，请等待当前任务完成")
        return
    
    is_sorting = True
    model = "Grap"
    play_voice(VOICE_CMD["sort_complete_confirm"], f"开始分拣{len(sort_queue)}个颜色")
    print(f"🚀 开始执行分拣任务，队列长度: {len(sort_queue)}")

def execute_grap():
    """执行抓取任务（在线程中运行）"""
    global sort_queue, msg, model, is_sorting
    
    try:
        while is_sorting and sort_queue:
            current_color = sort_queue[0]
            
            if current_color in msg:
                current_pos = msg[current_color]
                print(f"🤖 执行抓取[{len(sort_queue)}]: {current_color}，位置: {current_pos}")
                play_voice(VOICE_CMD["sort_complete_confirm"], f"开始抓取{current_color}")
                
                try:
                    # 执行抓取
                    target.target_run({current_color: current_pos}, xy)
                    
                    # 从队列移除已抓取的颜色
                    sort_queue.pop(0)
                    
                    # 播报抓取结果
                    if sort_queue:
                        play_voice(VOICE_CMD["prompt_queue_info"], 
                                   f"{current_color}抓取完成，下一个抓取{sort_queue[0]}")
                        time.sleep(3)  # 等待机械臂稳定
                    else:
                        play_voice(VOICE_CMD["sort_complete_confirm"], "所有颜色分拣完成！")
                        
                except Exception as e:
                    print(f"❌ 抓取异常: {e}")
                    play_voice(VOICE_CMD["re_detect"], f"{current_color}抓取失败")
                    time.sleep(1)
            else:
                print(f"❌ 颜色{current_color}不在识别结果中")
                sort_queue.pop(0)
        
    except Exception as e:
        print(f"❌ 分拣线程异常: {e}")
    finally:
        # 分拣完成
        is_sorting = False
        if not sort_queue:
            model = "General"
        print("📌 分拣任务结束")

def target_detection_Callback():
    global model
    model = 'Detection'
    print(f"🔄 当前模式切换为: {model}")

def reset_color_list_Callback():
    global model, sort_queue, is_sorting, current_sorting_thread
    sort_queue = []
    is_sorting = False
    # 尝试停止分拣线程
    if current_sorting_thread and current_sorting_thread.is_alive():
        is_sorting = False
        current_sorting_thread.join(timeout=1)
    model = 'Reset_list'
    print(f"🔄 当前模式切换为: {model}")

def grap_Callback():
    global model
    model = 'Grap'
    print(f"🔄 当前模式切换为: {model}")

def exit_button_Callback():
    global model, is_running, is_sorting, current_sorting_thread
    is_sorting = False
    # 尝试停止分拣线程
    if current_sorting_thread and current_sorting_thread.is_alive():
        current_sorting_thread.join(timeout=2)
    model = 'Exit'
    is_running = False
    print(f"🔄 当前模式切换为: {model}，程序即将退出")

def camera():
    global color_hsv, model, dp, msg, detected_colors, sort_queue, result, is_running, is_sorting, current_sorting_thread
    
    # 打开摄像头并配置参数
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
    
    if not capture.isOpened():
        return

    while is_running and capture.isOpened():
        try:
            # 1. 读取语音指令
            try:
                result = mySpeech.speech_read()
            except Exception as e:
                print(f"❌ 语音读取异常: {e}")
                result = 0

            # 2. 读取摄像头帧
            for i in range(5):
                ret, img = capture.read()
                if not ret or img is None:
                    pass
            fps.update_fps()

            # 3. 相机标定阶段
            if model == 'Calibration':
                dp, img = calibration.calibration_map(img, xy, threshold)
                if len(dp) != 0:
                    model = "Detection2"
                    continue

            # 4. 透视变换
            if len(dp) != 0:
                img = calibration.Perspective_transform(dp, img)

            # 5. 取消标定
            if model == 'calibration_Cancel':
                dp = []
                msg = {}
                detected_colors = []
                sort_queue = []
                is_sorting = False
                if current_sorting_thread and current_sorting_thread.is_alive():
                    is_sorting = False
                    current_sorting_thread.join(timeout=1)
                model = "General"
                play_voice(VOICE_CMD["queue_cleared"], "已取消标定")
                print("🔄 取消标定，回到通用模式")

            # 6. 颜色识别阶段
            if model == 'Detection2':
                # 清空历史数据
                msg = {}
                detected_colors = []
                sort_queue = []
                is_sorting = False
                if current_sorting_thread and current_sorting_thread.is_alive():
                    is_sorting = False
                    current_sorting_thread.join(timeout=1)
                
                img, msg, _, _, _, _ = target.select_color(img, color_hsv, COLOR_LIST)
                play_voice(VOICE_CMD["start_detect"], "开始识别颜色")

                # 收集识别到的颜色并播报
                for color in COLOR_LIST:
                    if color in msg:
                        detected_colors.append(color)
                        # 播报对应颜色
                        color_voice_cmd = VOICE_CMD[f"color_{color}"]
                        play_voice(color_voice_cmd, f"识别到{color}")

                print(f"📌 识别到的颜色: {detected_colors}，数量: {len(detected_colors)}")  
                
                if detected_colors:
                    play_voice(VOICE_CMD["prompt_select_color"], 
                               f"共识别到{len(detected_colors)}种颜色：{', '.join(detected_colors)}，请依次说出要分拣的颜色（说停止选择结束）")
                    model = "ColorSelection"  # 进入颜色选择模式
                else:
                    play_voice(VOICE_CMD["prompt_select_color"], "未识别到任何颜色")
                    model = "General"

            # 7. 颜色选择模式
            if model == 'ColorSelection' and result != 0:
                speech_cmd = parse_speech_result(result)
                if speech_cmd:
                    # 7.1 选择颜色
                    if speech_cmd in COLOR_LIST:
                        update_sort_queue(speech_cmd)
                    # 7.2 取消最后一次选择
                    elif speech_cmd == "cancel" and sort_queue:
                        removed_color = sort_queue.pop()
                        play_voice(VOICE_CMD["prompt_queue_info"], 
                                   f"已取消{removed_color}，当前分拣队列：{', '.join(sort_queue) if sort_queue else '空'}")
                        print(f"📌 取消选择: {removed_color}，剩余队列: {sort_queue}")
                    # 7.3 清空队列
                    elif speech_cmd == "clear_queue":
                        clear_sort_queue()
                    # 7.4 停止选择，进入确认阶段
                    elif speech_cmd == "stop_select":
                        if sort_queue:
                            play_voice(VOICE_CMD["sort_complete_confirm"], 
                                       f"分拣顺序确认：{', '.join(sort_queue)}，是否开始分拣？（说'开始分拣'或'确认分拣'）")
                            model = "toGrap"
                        else:
                            play_voice(VOICE_CMD["prompt_queue_info"], "分拣队列为空，请重新选择颜色")
                    # 7.5 重新识别
                    elif speech_cmd == "re_detect":
                        model = "Detection2"
                    # 7.6 开始分拣（新增）
                    elif speech_cmd == "begin_sort":
                        begin_sorting()
                        
                result = 0

            # 8. 分拣准备阶段
            if model == 'toGrap' and result != 0:
                # 在准备阶段，刷新识别结果以确保位置准确
                img, msg, _, _, _, _ = target.select_color(img, color_hsv, COLOR_LIST)
                speech_cmd = parse_speech_result(result)
                
                if speech_cmd in ["confirm", "begin_sort"]:  # 确认分拣或开始分拣
                    result = 0
                    begin_sorting()
                elif speech_cmd == "re_detect":
                    result = 0
                    model = "Detection2"
                    play_voice(VOICE_CMD["re_detect"], "重新识别颜色")
                    print("🔄 收到重新识别指令，重新识别颜色")

            # 9. 重新识别指令
            if result == VOICE_CMD["recognize_re_detect"]:
                result = 0
                msg = {}
                detected_colors = []
                sort_queue = []
                is_sorting = False
                if current_sorting_thread and current_sorting_thread.is_alive():
                    is_sorting = False
                    current_sorting_thread.join(timeout=1)
                model = "Detection2"
                play_voice(VOICE_CMD["re_detect"], "重新识别颜色")
                print("🔄 收到重新识别指令，重新识别颜色")

            # 10. 重置列表
            if model == "Reset_list":
                msg = {}
                detected_colors = []
                sort_queue = []
                is_sorting = False
                if current_sorting_thread and current_sorting_thread.is_alive():
                    is_sorting = False
                    current_sorting_thread.join(timeout=1)
                model = "General"
                play_voice(VOICE_CMD["queue_cleared"], "分拣队列已清空")
                print("🔄 重置颜色列表，回到通用模式")

            # 11. 执行抓取（分拣任务）
            if model == 'Grap' and is_sorting and sort_queue:
                # 检查当前是否有分拣线程在运行
                if current_sorting_thread and current_sorting_thread.is_alive():
                    # 线程正在运行，检查是否需要启动新线程
                    if not is_sorting:  # 如果分拣标志被设为False，说明线程已结束
                        current_sorting_thread = None
                else:
                    # 启动新的分拣线程
                    if current_sorting_thread is None or not current_sorting_thread.is_alive():
                        current_sorting_thread = threading.Thread(
                            target=execute_grap, 
                            daemon=True
                        )
                        current_sorting_thread.start()
                        print("📌 启动新的分拣线程")

            # 12. 检查分拣线程状态
            if current_sorting_thread and not current_sorting_thread.is_alive():
                # 线程已结束，清理资源
                if is_sorting:  # 如果标志仍然为True，说明线程异常结束
                    is_sorting = False
                    model = "General"
                    print("⚠️ 分拣线程异常结束")
                current_sorting_thread = None

            # 13. 退出程序
            if model == 'Exit' or not is_running:
                is_sorting = False
                break

            # 14. 显示画面（按q退出）
            # 在画面上显示状态信息
            status_text = f"Mode: {model}"
            queue_text = f"Queue: {sort_queue}" if sort_queue else "Queue: Empty"
            sorting_text = "Sorting: ACTIVE" if is_sorting else "Sorting: IDLE"
            thread_text = f"Thread: ALIVE" if current_sorting_thread and current_sorting_thread.is_alive() else "Thread: DEAD"
            
            cv.putText(img, status_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(img, queue_text, (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(img, sorting_text, (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv.putText(img, thread_text, (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv.imshow("Color Sorting (Voice Control)", img)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                is_running = False
                is_sorting = False
                break
            elif key == ord('s') and sort_queue and not is_sorting: 
                begin_sorting()

        except Exception as e:
            print(f"❌ 循环内异常: {e}")
            traceback.print_exc()
            time.sleep(0.1)
            continue

    capture.release()
    cv.destroyAllWindows()
    try:
        mySpeech.close()
    except Exception as e:
        print(f"❌ 语音串口关闭异常: {e}")

if __name__ == "__main__":
    try:
        # 初始化流程
        load_config()  # 加载配置文件
        time.sleep(1)
        model = 'Calibration'  # 启动模式：标定
        is_sorting = False
        current_sorting_thread = None

        # 启动相机线程（守护线程）
        cam_thread = threading.Thread(target=camera, daemon=True)
        cam_thread.start()

        input("📌 按回车键退出程序...\n")
        is_running = False
        is_sorting = False
        cam_thread.join(timeout=3)

    except KeyboardInterrupt:
        is_running = False
        is_sorting = False
        print("⚠️ 用户中断程序")
    except Exception as e:
        print(f"❌ 程序入口异常: {e}")
        traceback.print_exc()
    finally:
        cv.destroyAllWindows()
        try:
            mySpeech.close()  # 关闭语音
        except Exception as e:
            print(f"❌ 最终资源释放异常: {e}")
        print("✅ 程序完全退出")
