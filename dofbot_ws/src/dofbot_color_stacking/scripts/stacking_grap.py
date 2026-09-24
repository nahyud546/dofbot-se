#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class stacking_grap:
    # --- Tower place poses: 1 truc XY duy nhat, 4 do cao (khong phai 4 XY rieng) ---
    # FK verify 2026-09-24 (URDF dofbot.urdf, base_link -> Gripping_point_Link):
    #   L1 (-0.13111,-0.13949,0.03909)  L2 (-0.13131,-0.13970,0.07032)
    #   L3 (-0.13132,-0.13971,0.10052)  L4 (-0.13121,-0.13959,0.13045)
    #   Lech XY doi nhau <= 0.31mm, Z cach nhau ~30mm = dung chieu cao cube.
    #   Ket luan: tower thang dung san; khong co chuyen "setup 4 toa do ma
    #   tang 2 lech, tang 3/4 thang". Lech tang 2 quan sat thuc te la do TAY
    #   keo/xo cube luc roi di sau khi nha (xem move), khong phai do toa do.
    #   Quy tac: J1/J5 luc dat LUON = TOWER_J1 (cang ngang luc nha, off-square
    #   ~1 do). Khong sua rieng 1 tang ma khong verify lai FK ca 4.
    #   Vi tri tower (-0.13, -0.14, radial 0.191m): ngoai khung hinh trai nen
    #   khong dung hang lan can; la huong duy nhat IK du ca 4 tang toi z=0.13
    #   (gan hong |y|~=0.11 FAIL tu z>=0.11, sat chan X>=-0.12 FAIL ca tang 1).
    TOWER_J1 = 136.4
    PLACE = {
        '1': [TOWER_J1, 28.3, 72.7, 5.3, TOWER_J1],
        '2': [TOWER_J1, 38.8, 72.7, 0.1, TOWER_J1],
        '3': [TOWER_J1, 52.4, 65.6, 1.3, TOWER_J1],
        '4': [TOWER_J1, 61.5, 65.9, 0.3, TOWER_J1],
    }
    # Waypoint nhac thang dung sau khi nha (cung truc tower):
    #   L1->L2, L2->L3, L3->L4: tan dung tang tren lam moc (dXY<=0.3mm,
    #   dz~+30mm). L4 (tang cao nhat) -> LIFT_TOP rieng, FK verify
    #   dXY=0.24mm, dz=+59.7mm.
    LIFT = {
        '1': [TOWER_J1, 38.8, 72.7, 0.1, TOWER_J1],
        '2': [TOWER_J1, 52.4, 65.6, 1.3, TOWER_J1],
        '3': [TOWER_J1, 61.5, 65.9, 0.3, TOWER_J1],
        '4': [TOWER_J1, 76.5, 65.9, 5.3, TOWER_J1],
    }

    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device("/dev/ttyUSB0")
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = 140

    def move(self, joints, joints_down, joints_lift, joints_home):
        '''
        Moving process
        移动过程
        :param joints: 移动到物体位置的各关节角度   The angle of each joint moved to the position of the object
        :param joints_down: 机械臂堆叠各关节角度   Manipulator stacking joint angle
        :param joints_lift: waypoint nhac thang dung doc truc tower sau khi nha,
                            roi moi ve home (chong xo lech cube vua dat)
        '''
        # Match the known color-sorting pick sequence. The old fixed folded
        # waypoint was unrelated to the IK target and could sweep into the base.
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        self.arm.Arm_serial_servo_write6_array(joints, 1000)
        sleep(1)
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # Use the same clearance lift as the working color-sorting routine.
        self.arm.Arm_serial_servo_write(2, 120, 2000)
        sleep(2.5)
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(1.0)
        # Nhac thang dung doc truc tower (dXY<=0.3mm, da verify FK) roi moi ve
        # home. KHONG keo rieng J2 ve 90 o day: TCP di theo cung tron, cang
        # van om sat cube vua nha -> quet ngang, xo lech thap (nghi van chinh
        # gay lech tang 2 so voi tang 1, trong khi tang 3/4 nhe hon/cao hon nen
        # it anh huong, nhin nhu "thang hang").
        self.arm.Arm_serial_servo_write6_array(joints_lift, 800)
        sleep(1)
        self.arm.Arm_serial_servo_write6_array(joints_home, 1500)
        sleep(1)

    def arm_run(self, move_num, joints, joints_home):
        '''
        Manipulator movement function 机械臂移动函数
        :param move_num: 抓取次数   Grab times
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        # 4 tang dat tai CUNG 1 XY tower (chi khac Z); J1/J5 luc dat luon =
        # TOWER_J1 de thap thang. Lay tu PLACE/LIFT, khong hardcode rieng tung
        # tang o day de khong vo tinh lam lech 1 tang.
        if move_num not in self.PLACE or self.move_status != True:
            return
        # It is set here. You can only run down after this operation
        # 此处设置,需执行完本次操作,才能向下运行
        self.move_status = False
        try:
            # J5 pick do target_run tinh san = J1 - SIGN*cube_yaw (cang bam
            # theo goc xoay cube; cube thang thi ve dung J5=J1). Chi clamp
            # [0,270], KHONG ghi de o day nua (ghi de la khoa cung J5).
            # Goc: yaw_mo_cang = J1 + 90 - J5 (do FK full-chain 2026-09-24).
            j5_pick = min(270, max(0, joints[4]))
            joints = [joints[0], joints[1], joints[2], joints[3], j5_pick, 30]
            place5 = self.PLACE[move_num]
            joints_down = [place5[0], place5[1], place5[2], place5[3],
                           place5[4], self.grap_joint]
            lift5 = self.LIFT[move_num]
            joints_lift = [lift5[0], lift5[1], lift5[2], lift5[3],
                           lift5[4], 30]
            self.move(joints, joints_down, joints_lift, joints_home)
        finally:
            self.move_status = True
