#!/usr/bin/env python3
# encoding: utf-8
import cv2 as cv
import time
from dofbot_utils.GestureRecognition import handDetector
import sys                                                                  
from dofbot_utils.fps import FPS
from dofbot_utils.robot_controller import Robot_Controller
from Arm_Lib import Arm_Device
import threading


class Gesture_Grasp:
    def __init__(self):
        self.hand_detector = handDetector(detectorCon=0.75)
        self.pTime = 0

        # 定义手势识别次数
        self.Count_One = 0
        self.Count_Two = 0
        self.Count_Three = 0
        self.Count_Four = 0
        self.Count_Five = 0

        self.arm = Arm_Device()
        self.move_state = False
        self.fps = FPS()
        self.robot = Robot_Controller()
        self.grap_joint = self.robot.get_gripper_value(1)
        self._joint_5 = self.robot.joint5
        
        self.p_Yellow = [65, 25, 60, 52, 270, 135]  # 黄色位置
        self.p_Red = [118, 25, 60, 52, 270, 135]    # 红色位置
        self.p_Green = [136, 66, 20, 29, 270, 135]  # 绿色位置
        self.p_Blue = [44, 66, 20, 28, 270, 135]    # 蓝色位置
        self.p_gray = [90, 52, 37, 29, 270, 90]    #gray位置
        self.look_at = [90, 164, 18, 0, 90, 90]

        self.arm.Arm_serial_servo_write6_array(self.look_at, 1000)


    def ctrl_arm_move(self, index):
        if index == 5:
            for i in range(5):
                self.arm.Arm_serial_servo_write(5, 60, 300)
                time.sleep(0.4)
                self.arm.Arm_serial_servo_write(5, 120, 300)
                time.sleep(0.4)
            self.arm.Arm_serial_servo_write(5, 90, 300)
            time.sleep(0.4)
            return

        # raise 抬起
        joints_uu = [90, 80, 50, 50, 270, 30]
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        time.sleep(2)
        
        # 第一步：移动到gray位置抓取色块
        # Move to gray position to pick up block
        self.arm.Arm_serial_servo_write6_array(self.p_gray, 1000)
        time.sleep(1.5)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        time.sleep(1)
        # put up 架起
        self.arm.Arm_serial_servo_write(2, 70, 1000)
        time.sleep(1)
        joints_up = [90, 80, 50, 50, 270, self.grap_joint]
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        time.sleep(1.5)
        
        # 第二步：移动到对应颜色位置放置色块
        # Move to target color position to place block
        if index == 1:
            # 移动到黄色位置 Move to yellow position
            self.arm.Arm_serial_servo_write6_array(self.p_Yellow, 1000)
        elif index == 2:
            # 移动到红色位置 Move to red position
            self.arm.Arm_serial_servo_write6_array(self.p_Red, 1000)
        elif index == 3:
            # 移动到绿色位置 Move to green position
            self.arm.Arm_serial_servo_write6_array(self.p_Green, 1000)
        elif index == 4:
            # 移动到蓝色位置 Move to blue position
            self.arm.Arm_serial_servo_write6_array(self.p_Blue, 1000)
        time.sleep(1.5)
        
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        time.sleep(1)
        # raise  抬起, 恢复到默认姿态
        self.arm.Arm_serial_servo_write6_array(self.look_at, 1300)
        time.sleep(1.5)
        

    def process(self, frame):
        frame, lmList = self.hand_detector.findHands(frame, draw=False)
        if len(lmList) != 0:
            gesture = self.hand_detector.get_gesture()
            if gesture == 'One':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_One = self.Count_One + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_One >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
            elif gesture == 'Two':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Two = self.Count_Two + 1
                self.Count_Three = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Two >= 10  and self.move_state == False:
                    if not self.move_state:
                        self.move_state = True
                        # print("start arm_ctrl_threading = {}".format(gesture))
                        task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                        task.setDaemon(True)
                        task.start()
            elif gesture == 'Three':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Three = self.Count_Three + 1
                self.Count_Two = 0
                self.Count_Four = 0
                self.Count_Five = 0
                if self.Count_Three >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
            elif gesture == 'Four':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Four = self.Count_Four + 1
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Five = 0
                if self.Count_Four >= 10  and self.move_state == False:
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()
            elif gesture == 'Five':
                cv.putText(frame, gesture, (250, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)
                self.Count_Five = self.Count_Five + 1
                self.Count_One = 0
                self.Count_Two = 0
                self.Count_Three = 0
                self.Count_Four = 0
                if self.Count_Five >= 10  and self.move_state == False:
                    self.Count_Five = 0
                    self.move_state = True
                    # print("start arm_ctrl_threading = {}".format(gesture))
                    task = threading.Thread(target=self.arm_ctrl_threading, name="arm_ctrl_threading", args=(gesture, ))
                    task.setDaemon(True)
                    task.start()

        self.fps.update_fps()
        self.fps.show_fps(frame)
        return frame
        


    def arm_ctrl_threading(self, gesture):
        # print("arm_ctrl_threading gesture = {}".format(gesture))
        if gesture == 'One':
            self.ctrl_arm_move(1)
        elif gesture == 'Two':
            self.ctrl_arm_move(2)
        elif gesture == 'Three':
            self.ctrl_arm_move(3)
        elif gesture == 'Four':
            self.ctrl_arm_move(4)
        elif gesture == 'Five':
            self.ctrl_arm_move(5)
        self.move_state = False



if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    gesture = Gesture_Grasp()
    while capture.isOpened():
        try:
            ret, frame = capture.read()
            action = cv.waitKey(1) & 0xFF
            frame = gesture.process(frame)
            if action == ord('q'):
                break
            cv.imshow('frame', frame)
        except:
            print("break")
            break 
    print("capture.release()")
    capture.release()
    cv.destroyAllWindows()