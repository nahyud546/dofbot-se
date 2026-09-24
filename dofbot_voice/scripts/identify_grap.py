# #!/usr/bin/env python
# # coding: utf-8
import Arm_Lib
from time import sleep

class identify_grap:
    def __init__(self):
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        self.grap_joint = 140
        # 关节5固定角度（替代原Robot_Controller的joint5）
        self._joint_5 = 265
        
        # 格式：[关节1, 关节2, 关节3, 关节4, 关节5, 夹爪角度]
        self.P_RED = [45, 50, 20, 60, self._joint_5, self.grap_joint]
        self.P_BLUE = [27, 75, 0, 50, self._joint_5, self.grap_joint]
        self.P_GREEN = [152, 75, 0, 50, self._joint_5, self.grap_joint]
        self.P_YELLOW = [137, 50, 20, 60, self._joint_5, self.grap_joint]

    def move(self, joints, joints_down):
        '''
        移动过程
        :param joints: 移动到物体位置的各关节角度
        :param joints_down: 移动到对应颜色目标位置的各关节角度
        '''
        joints_uu = [90, 80, 50, 50, self._joint_5, self.grap_joint]
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1500)
        sleep(1)
        # 开合夹爪预热
        for i in range(5):
            self.arm.Arm_serial_servo_write(6, 180, 100)
            sleep(0.1)
            self.arm.Arm_serial_servo_write(6, 30, 100)
            sleep(0.1)
        # 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 1000)
        sleep(1.5)
        # 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(1)
        # 回退动作（防止碰撞物体）
        back_joints = [joints[0], joints[1]+30, joints[2]-30, joints[3]+10, joints[4], self.grap_joint]
        self.arm.Arm_serial_servo_write6_array(back_joints, 1000)
        sleep(1)
        # 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1.5)
        # 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 1000)
        sleep(1.5)
        # 移动至目标位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1.5)
        # 释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(1)
        # 抬起
        joints_up = [joints_down[0], 80, 50, 50, self._joint_5, 30]
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1.5)

    def identify_move(self, name, joints):
        '''
        机械臂移动函数（按颜色匹配固定目标位置）
        :param name: 识别的颜色
        :param joints: 反解求得的各关节角度
        '''
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], 30]
        if name == "red" and self.move_status == True:
            self.move_status = False
            self.move(joints, self.P_RED)
            self.move_status = True
        if name == "blue" and self.move_status == True:
            self.move_status = False
            self.move(joints, self.P_BLUE)
            self.move_status = True
        if name == "green" and self.move_status == True:
            self.move_status = False
            self.move(joints, self.P_GREEN)
            self.move_status = True
        if name == "yellow" and self.move_status == True:
            self.move_status = False
            self.move(joints, self.P_YELLOW)
            self.move_status = True

