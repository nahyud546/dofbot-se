#!/usr/bin/env python
# coding: utf-8
import random
import Arm_Lib
import threading
import cv2 as cv
from time import sleep
# from sorting_move import sorting_move

from dofbot_utils.robot_controller import Robot_Controller
class color_sorting:
    def __init__(self, port='/dev/ttyUSB0'):
        '''
        设置初始化参数
        '''
        self.image = None
        # 初始化计数器
        self.num = 0
        # 初始化运动状态
        self.status = 'waiting'
        # 创建抓取实例
        # self.sorting_move = sorting_move()
        self.grap_joint = 135
        self.gripper_release = 30
        # 创建机械臂实例
        self.arm = Arm_Lib.Arm_Device(port)
        self.robot = Robot_Controller(arm=self.arm)

        self.joints = self.robot.P_BLACK_CENTER
        # self.joints = [90, 35, 65, 15, 90,135]
        self.joints[5] = self.robot.get_gripper_value(0)
        self.P_LOOK_MAP = [90, 125, 0, 0, 90,0]

    def get_Sqaure(self, color_name, hsv_lu):
        '''
        颜色识别
        '''
        (lowerb, upperb) = hsv_lu
        # 复制原始图像,避免处理过程中干扰
        img = self.image.copy()
        # mask = self.image.copy()
        #mask = img[260:460, 210:430]
        mask = img[200:480, 160:400]
        # cv.imshow("mask", mask)
        # 将图像转换为HSV。
        HSV_img = cv.cvtColor(mask, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        img = cv.inRange(HSV_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        mask[img == 0] = [0, 0, 0]
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # 将图像转为灰度图
        dst_img = cv.cvtColor(dst_img, cv.COLOR_RGB2GRAY)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            # boundingRect函数计算边框值，x，y是坐标值，w，h是矩形的宽和高
            x, y, w, h = cv.boundingRect(cnt)
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积范围
            if area > 1000:
                # 中心坐标
                x_w_ = float(x + w / 2)
                y_h_ = float(y + h / 2)
                # 在img图像画出矩形，(x, y), (x + w, y + h)是矩形坐标，(0, 255, 0)设置通道颜色，2是设置线条粗度
                # cv.rectangle(self.image, (x , y ), (x + w , y + h ), (0, 255, 0), 2)
                cv.rectangle(self.image, (x + 170, y + 200), (x + w + 210, y + h + 230), (0, 0, 0), 2)
                # 绘制矩形中心
                # cv.circle(self.image, (int(x_w_ ), int(y_h_ )), 5, (0, 0, 255), -1)
                cv.circle(self.image, (int(x_w_ + 210), int(y_h_ + 215)), 5, (0, 0, 255), -1)
                # # 在图片中绘制结果
                # cv.putText(self.image, color_name, (int(x -15), int(y -15)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                cv.putText(self.image, color_name, (int(x + 202), int(y + 215)), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                return (x_w_, y_h_)

    def Sorting_grap(self, img, color_hsv):
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480))
        # 设置随机颜色
        # color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
        # 画矩形框 color[random.randint(0, 254)]
        #cv.rectangle(self.image, (260, 220), (420, 380),(105,105,105), 2)
        # 获取识别的结果
        msg = {}
        # 遍历颜色通道,获取能够识别的结果
        for key, value in color_hsv.items():
            point = self.get_Sqaure(key, value)
            if point != None: msg["name"] = key
        if len(msg) == 1:
            self.num += 1
            # 每当连续识别20次并且运动状态为waiting的情况下,执行抓取任务
            if self.num % 10 == 0 and self.status == 'waiting':
                self.status = "Runing"
                self.arm.Arm_Buzzer_On(1)
                sleep(0.5)
                # 开启抓取线程
                threading.Thread(target=self.sorting_run, args=(msg['name'],)).start()
                self.num = 0 
        return self.image

    def sorting_move(self, joints_target):
        '''
        移动过程
        '''
        joints_up = [90, 80, 50, 50, 90, self.grap_joint]
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # Release clamping jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # Move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(self.joints, 1000)
        sleep(1)
        # Grasp and clamp the clamping claw进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # put up 架起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # rotate 旋转
        self.arm.Arm_serial_servo_write(1, joints_target[0], 1000)
        sleep(1)
        # Move to corresponding position 移动至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_target, 1000)
        sleep(1.5)
        # Release the object and release the clamping jaws释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.8)
        # raise  抬起
        self.arm.Arm_serial_servo_write(2, 90, 1000)
        sleep(1)
        # return to center 返回至中心位置
        self.arm.Arm_serial_servo_write(1, 90, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        self.arm.Arm_serial_servo_write6_array(self.P_LOOK_MAP, 1000)
        sleep(1.5)

    def sorting_run(self, name):
        '''
        机械臂移动函数
        '''
        if name == "red" :
            # print("red")
            # 物体放置位姿
#             joints_target = [115, 20, 80, 40, 90, self.grap_joint]
            joints_target = [117, 19, 66, 56, 90, self.grap_joint]
            # 移动
            self.sorting_move(joints_target)
            self.status = 'waiting'
        if name == "blue":
            # print("blue")
#             joints_target = [45, 80, 0, 40, 90, self.grap_joint]
            joints_target = [44, 66, 20, 28, 90, self.grap_joint]
            self.sorting_move(joints_target)
            self.status = 'waiting'
        if name == "green" :
            # print("green")
#             joints_target = [137, 80, 0, 40, 90, self.grap_joint]
            joints_target = [136, 66, 20, 29, 90, self.grap_joint]
            self.sorting_move(joints_target)
            self.status = 'waiting'
        if name == "yellow" :
            # print("yellow")
#             joints_target = [65, 20, 80, 40, 90, self.grap_joint]
            joints_target = [65, 22, 64, 56, 90, self.grap_joint]
            self.sorting_move(joints_target)
            self.status = 'waiting'


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='DOFBOT color sorting from a fixed center pickup point')
    parser.add_argument('--camera', default='/dev/video2')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--check', action='store_true',
                        help='validate Python dependencies without opening hardware')
    args = parser.parse_args()

    if args.check:
        print('COLOR SORTING SOFTWARE CHECK: PASS')
        return

    capture = cv.VideoCapture(args.camera, cv.CAP_V4L2)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    if not capture.isOpened():
        raise RuntimeError('Cannot open camera {}'.format(args.camera))

    sorter = color_sorting(port=args.port)
    sorter.arm.Arm_serial_servo_write6_array(sorter.P_LOOK_MAP, 1500)
    sleep(1.5)
    color_hsv = {
        'red': ((0, 25, 90), (10, 255, 255)),
        'green': ((53, 36, 40), (80, 255, 255)),
        'blue': ((110, 80, 90), (130, 255, 255)),
        'yellow': ((20, 20, 55), (50, 255, 255)),
    }

    print('RUNNING: camera={}, serial={}'.format(args.camera, args.port))
    print('Dat MOT cube vao vung gap trung tam. Bam q de thoat.')
    try:
        while capture.isOpened():
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Camera frame read failed')
            output = sorter.Sorting_grap(frame, color_hsv)
            cv.imshow('DOFBOT color sorting', output)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        capture.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()
