#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import transforms3d as tfs
import tf_transformations as tf         # ROS2使用tf_transformations
import threading
from dofbot_driver.vutils import draw_tags
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import Float32,Bool,Int16
import time
import queue
import math
from Arm_Lib import Arm_Device
from dofbot_driver.compute_joint5 import *
from Speech_Lib import Speech

encoding = ['16UC1', '32FC1']

class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect')
        self.Arm = Arm_Device()
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 0.0]
        self.joint5 = Int16()
        self.detect_flag = False
        self.compute_height = True
        self.index = None
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.TargetJoint5_pub = self.create_publisher(Int16, "set_joint5", 10)

        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=10)
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        self.pubPos_flag = False
        self.voice_cmd_flag = False  # 语音命令标志
        self.voice_target_id = None  # 语音指定的目标ID
        self.last_voice_result = 0   # 存储上次语音识别结果
        self.voice_lock = threading.Lock()  # 语音识别锁
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.target_id = 31
        self.Center_x_list = []
        self.Center_y_list = []
        self.heigh = 0.0
        self.CurEndPos = [-0.006,0.116261662208,0.0911289015753,-1.04719,-0.0,0.0]
        self.camera_info_K = [1021.16691, 0.0, 238.3799, 0.0, 1014.00486, 261.07339, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[-0.001 ,-1.000 ,-0.001 , 0.000],
                                     [-0.002  ,0.001 ,-1.000 ,-0.056],
                                     [ 1.000 ,-0.001 ,-0.002 ,-0.097],
                                     [0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00]])
        self.get_current_end_pos()
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        self.dist = 0.13
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        
        # 初始化语音识别
        self.mySpeech = Speech()
        
        # 启动语音识别线程
        self.voice_thread = threading.Thread(target=self.voice_recognition_loop, daemon=True)
        self.voice_thread.start()

        
    def get_current_end_pos(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'dofbot_kinematics' not available!")
            return
        request = Kinemarics.Request()
        request.cur_joint1 = self.init_joints[0]
        request.cur_joint2 = self.init_joints[1]
        request.cur_joint3 = self.init_joints[2]
        request.cur_joint4 = self.init_joints[3]
        request.cur_joint5 = self.init_joints[4]
        request.kin_name = "fk"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) 
        response = future.result()
        if isinstance(response, Kinemarics.Response):
            self.CurEndPos[0] = response.x
            self.CurEndPos[1] = response.y
            self.CurEndPos[2] = response.z
            self.CurEndPos[3] = response.roll
            self.CurEndPos[4] = response.pitch
            self.CurEndPos[5] = response.yaw   
            print("self.CurEndPos: ",self.CurEndPos)

    def voice_recognition_loop(self):
        """语音识别"""
        while True:
            try:
                result = self.mySpeech.speech_read()
                with self.voice_lock:
                    self.last_voice_result = result
                time.sleep(0.05)  # 稍微缩短睡眠时间，提高响应速度
            except Exception as e:
                self.get_logger().error(f"Error in voice recognition loop: {e}")
                time.sleep(0.1)

    def process_speech_command(self):
        """处理语音命令"""
        with self.voice_lock:
            result = self.last_voice_result
            # 重置语音识别结果，避免重复处理
            self.last_voice_result = 999
        
        # 语音指令映射
        # 95 -> AprilTag 1
        # 96 -> AprilTag 2
        # 97 -> AprilTag 3
        # 98 -> AprilTag 4
        if result in [95, 96, 97, 98]:
            target_tag_id = result - 94  # 95->1, 96->2, 97->3, 98->4
            self.voice_cmd_flag = True
            self.voice_target_id = target_tag_id
            self.get_logger().info(f"Received voice command for AprilTag {target_tag_id}")

    def ImageCallback(self,color_frame):
        # 处理语音命令
        self.process_speech_command()
        
        try:
            #rgb_image
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
            result_image = np.copy(rgb_image)
            
            tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
            tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
            draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
            self.Center_x_list = list(range(len(tags)))
            self.Center_y_list = list(range(len(tags)))
            
            # 检测空格键 - 只在没有处理抓取时才允许设置
            key = cv2.waitKey(1) & 0xFF
            if key == 32 and not self.voice_cmd_flag:  # 空格键，但仅在没有语音命令时有效
                if not self.detect_flag:  # 只有在没有处理抓取时才允许设置
                    self.pubPos_flag = True
            
            if len(tags) > 0 :
                for i in range(len(tags)):
                    
                    center_x, center_y = tags[i].center
                    self.Center_x_list[i] = center_x
                    self.Center_y_list[i] = center_y
                    cx = center_x
                    cy = center_y
                    cz = self.dist
                    (a, b) = (round(((cx - 320) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.13, 5))
                    print("a: ",a)
                    print("b: ",b)   
                    
                    # 检查是否有语音命令指定的标签ID
                    if self.voice_cmd_flag and self.voice_target_id is not None:
                        # 语音控制模式：必须检测到指定的AprilTag ID才能抓取
                        if tags[i].tag_id == self.voice_target_id:
                            pos = AprilTagInfo()
                            pos.x = b
                            pos.y = -a
                            pos.z = 0.03
                            pos.id = tags[i].tag_id
                            self.pos_info_pub.publish(pos)
                            self.get_logger().info(f"Publishing position for voice-specified AprilTag {self.voice_target_id}")
                            # 重置语音命令标志
                            self.voice_cmd_flag = False
                            self.voice_target_id = None
                            self.detect_flag = True  # 设置检测标志，防止重复发布
                            break  # 发布后立即退出循环，避免重复发布
                    else:
                        # 非语音控制模式：只有当检测到空格键且当前没有在处理抓取时才发布
                        if self.pubPos_flag == True:
                            pos = AprilTagInfo()
                            pos.x = a
                            pos.y = b
                            pos.z = 0.03
                            pos.id = tags[i].tag_id
                            self.pos_info_pub.publish(pos)
                            # 发布后立即设置标志位
                            self.pubPos_flag = False
                            self.detect_flag = True  # 设置检测标志，防止重复发布
                            break  # 发布后立即退出循环，避免重复发布
            else:
                # 没有检测到标签时的处理
                if self.pubPos_flag == True:
                    self.pubPos_flag = False
                else:
                    # 添加调试信息
                    if self.pubPos_flag and self.detect_flag:
                        print("wait for grasp done...")
                    elif not self.pubPos_flag and self.detect_flag:
                        print("Grasp done, waiting for next detection...")
                    elif not self.pubPos_flag and not self.detect_flag:
                        print("wait space key to continue...")
                           
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cur_time = time.time()
            fps = str(int(1/(cur_time - self.pr_time)))
            self.pr_time = cur_time
            cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 安全地显示图像窗口
            cv2.imshow("result_image", result_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # 如果需要退出，可以在这里处理
                pass
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            # 即使图像处理失败，也要确保语音命令继续处理
            pass


    def GraspStatusCallback(self,msg):
        if msg.data == True:
            self.get_logger().info("Receive grasp_done msg: {}".format(msg.data))
            self.detect_flag = False  # 重置检测标志，允许下一次检测
            self.compute_height = True  # 重置高度计算标志
            self.pubPos_flag = False   # 重置发布标志
            self.voice_cmd_flag = False  # 重置语音命令标志
            self.voice_target_id = None  # 清空语音目标ID

    def compute_heigh(self,x,y,z):
        camera_location = self.pixel_to_camera_depth((x,y),z)
        PoseEndMat = np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(camera_location, (0, 0, 0)))
        EndPointMat = self.get_end_point_mat()
        WorldPose = np.matmul(EndPointMat, PoseEndMat) 
        pose_T, pose_R = self.mat_to_xyz_euler(WorldPose)
        return pose_T
        

    def get_end_point_mat(self):
        end_w,end_x,end_y,end_z = self.euler_to_quaternion(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        endpoint_mat = self.xyz_quat_to_mat([self.CurEndPos[0],self.CurEndPos[1],self.CurEndPos[2]],[end_w,end_x,end_y,end_z])
        return endpoint_mat
    
    #像素坐标转换到深度相机三维坐标坐标，也就是深度相机坐标系下的抓取点三维坐标
    def pixel_to_camera_depth(self,pixel_coords, depth):
        fx, fy, cx, cy = self.camera_info_K[0],self.camera_info_K[4],self.camera_info_K[2],self.camera_info_K[5]
        px, py = pixel_coords
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return np.array([x, y, z])
    
    #通过平移向量和旋转的欧拉角得到变换矩阵    
    def xyz_euler_to_mat(self,xyz, euler, degrees=False):
        if degrees:
            mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
        else:
            mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
        mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
        return mat        
    
    #欧拉角转四元数
    def euler_to_quaternion(self,roll,pitch, yaw):
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        qw = quaternion[3]
        qx = quaternion[0]
        qy = quaternion[1]
        qz = quaternion[2]
        #print("quaternion: ",quaternion )
        return np.array([qw, qx, qy, qz])

    #通过平移向量和旋转的四元数得到变换矩阵
    def xyz_quat_to_mat(self,xyz, quat):
        mat = tfs.quaternions.quat2mat(np.asarray(quat))
        mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
        return mat

    #把旋转变换矩阵转换成平移向量和欧拉角
    def mat_to_xyz_euler(self,mat, degrees=False):
        t, r, _, _ = tfs.affines.decompose(mat)
        if degrees:
            euler = np.degrees(tfs.euler.mat2euler(r))
        else:
            euler = tfs.euler.mat2euler(r)
        return t, euler

def main(args=None):
    rclpy.init(args=args)
    tag_detect = AprilTagDetectNode()
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理OpenCV窗口
        cv2.destroyAllWindows()
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()