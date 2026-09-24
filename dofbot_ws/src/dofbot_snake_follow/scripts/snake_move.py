#!/usr/bin/env python
# coding: utf-8
from time import sleep
from dofbot_utils.robot_controller import Robot_Controller

class snake_move:
    def __init__(self):
        self.robot = Robot_Controller()
        # 夹爪加紧角度 Gripper tightening angle
        self.grap_joint = 134
        #蓝色位置 Blue position
        self.blue = [30, 70, 0, 56, 90, 142]
        #绿色位置 Green position
        self.green = [150, 70, 7, 56, 90, 142]
        #红色位置 Red position
        self.red = [50, 70, 7, 56, 90, 142]
        #黄色位置 Yellow position
        self.yellow = [135, 70, 7, 56, 90, 142]

    def arm_move(self, joints_target):
        sleep(1)
        # 转平 turn flat
        self.robot.arm_move_1(5, 90, 700)
        sleep(1)
        # 松开夹爪 Release the jaws
        #self.robot.arm_move_1(6, 30, 700)
        #sleep(1)
        # 夹紧夹爪 Clamping jaws
        self.robot.arm_move_1(6, self.grap_joint, 700)
        sleep(1)
        # 移动至对应位置 move to the corresponding location
        self.robot.arm_move_6(joints_target, 1000)
        sleep(1.5)
        # 松开夹爪 Release the jaws
        self.robot.arm_move_1(6, 0, 700)
        sleep(1)
        self.robot.arm_move_1(2, 90, 1000)
        sleep(0.5)
        # 回到初始位置 return to original position
        # joint_00 = [90, 135, 0, 45, 0, 180]
        joint_00 = self.robot.P_SNAKE_INIT
        self.robot.arm_move_6(joint_00, 700)
        sleep(1)

    def snake_run(self, name):
        if name == "red":
            # print("red")
            # 物体放置位姿 object placement pose
            joints_target = self.red
            self.arm_move(joints_target)
        if name == "blue":
            # print("blue")
            joints_target = self.blue
            self.arm_move(joints_target)
        if name == "green":
            # print("green")
            joints_target = self.green  
            self.arm_move(joints_target)
        if name == "yellow":
            # print("yellow")
            joints_target = self.yellow
            self.arm_move(joints_target)
