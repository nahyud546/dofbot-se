#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2 as cv
from dt_apriltags import Detector
import threading
from dofbot_sorting_3d.vutils import draw_tags
from dofbot_interface.srv import Kinemarics
from dofbot_interface.msg import *
from std_msgs.msg import Float32,Bool,Int16,String
import time
import os
from dofbot_sorting_3d.compute_joint5 import *
#exit_code = os.system('sudo v4l2-ctl -d /dev/video0 -c brightness=10')
from Arm_Lib import Arm_Device

class AprilTagDetectNode(Node):
    def __init__(self):
        super().__init__('apriltag_detect')
        self.Arm = Arm_Device()
        self.init_joints = [90.0, 120.0, 0.0, 0.0, 90.0, 0.0]
        self.client = self.create_client(Kinemarics, 'dofbot_kinemarics')
        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.subscription = self.create_subscription(Bool,'grasp_done',self.GraspStatusCallback,qos_profile=1)
        self.rgb_bridge = CvBridge()
        self.pubPos_flag = False
        self.pr_time = time.time()
        self.at_detector = Detector(searchpath=['apriltags'], 
                                    families='tag36h11',
                                    nthreads=8,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        self.TargetID = 2.0
        self.declare_parameter('target_id', 2.0)
        self.TargetID = int(self.get_parameter('target_id').get_parameter_value().double_value)
        print("Get self.target_id is ",self.TargetID)
        self.Arm.Arm_serial_servo_write6_array(self.init_joints,2000)
        self.subscription = self.create_subscription(Image,'/image_raw',self.ImageCallback,1)
        self.count = True
        self.start_time = time.time()
        self.detect_flag = False
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',2)
        self.pub_joint5 = self.create_publisher(Int16, "set_joint5", 10)
        
    def ImageCallback(self,color_frame):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8')
        result_image = np.copy(rgb_image)
        tags = self.at_detector.detect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY), False, None, 0.025)
        tags = sorted(tags, key=lambda tag: tag.tag_id) # 貌似出来就是升序排列的不需要手动进行排列
        draw_tags(result_image, tags, corners_color=(0, 0, 255), center_color=(0, 255, 0))
        key = cv2.waitKey(10)
        if key == 32:
            self.pubPos_flag = True
        if self.count==True:
            if (time.time() - self.start_time)>10:
                self.pubPos_flag = True
                self.count = False
        if len(tags) > 0 :
            for i in range(len(tags)):
                
                center_x, center_y = tags[i].center
                #cv2.circle(depth_to_color_image,(int(center_x),int(center_y)),1,(255,255,255),10)
                cur_id = tags[i].tag_id
                if self.pubPos_flag == True and cur_id == self.TargetID:
                    self.detect_flag = True
                    print("Found the target.")
                    cx = center_x
                    cy = center_y
                    (a, b) = (round(((320 - cx ) / 4000), 5), round(((480 - cy) / 3000) * 0.8+0.13, 5))
                    print("a: ",a)
                    print("b: ",b) 
                    vx = int(tags[i].corners[0][0]) - int(tags[i].corners[1][0])
                    vy = int(tags[i].corners[0][1]) - int(tags[i].corners[1][1])
                    target_joint5 = compute_joint5(vx,vy)
                    print("target_joint5: ",target_joint5)
                    if self.pubPos_flag == True:
                        self.pubPos_flag = False
                        joint5 = Int16()
                        joint5.data = int(target_joint5)
                        self.pub_joint5.publish(joint5)
                        pos = AprilTagInfo()
                        pos.x = b
                        pos.y = a
                        pos.z = 0.03
                        pos.id = tags[i].tag_id
                        self.pos_info_pub.publish(pos)
            if 	self.detect_flag == False and self.TargetID!=0 and self.pubPos_flag==True:
                self.pubPos_flag = False
                print("Did not find the target.")
                self.largemodel_arm_done_pub.publish( String(data='apriltag_sort_done'))
                self.TargetID = 0
        elif self.pubPos_flag==True and len(tags) == 0 :
            self.pubPos_flag = False
            print("Did not find any apriltags.")
            self.largemodel_arm_done_pub.publish( String(data='apriltag_sort_done'))
            self.TargetID = 0

        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cur_time = time.time()
        fps = str(int(1/(cur_time - self.pr_time)))
        self.pr_time = cur_time
        cv2.putText(result_image, fps, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result_image", result_image)
        key = cv2.waitKey(1)

    def GraspStatusCallback(self,msg):
        print("**")
        if msg.data == True:
            self.pubPos_flag = True
            self.detect_flag = False


def main(args=None):
    rclpy.init(args=args)
    tag_detect = AprilTagDetectNode()
    try:    
        rclpy.spin(tag_detect)
    except KeyboardInterrupt:
        pass
    finally:
        tag_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()













