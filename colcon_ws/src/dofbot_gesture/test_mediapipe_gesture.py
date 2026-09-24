#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from gesture_stack import gesture_stack

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    gesture_handler = gesture_stack()
    
    print("开始手势识别测试...")
    print("请做出以下手势:")
    print("- 握拳 (0个手指): 推倒积木")
    print("- 1个手指: 抓取黄色积木")
    print("- 2个手指: 抓取红色积木")
    print("- 3个手指: 抓取绿色积木")
    print("- 4个手指: 抓取蓝色积木")
    print("- 按 'q' 键退出程序")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 处理手势识别
        processed_frame = gesture_handler.start_gesture(frame)
        
        # 显示处理后的帧
        cv2.imshow('MediaPipe Gesture Recognition', processed_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出")

if __name__ == "__main__":
    main()