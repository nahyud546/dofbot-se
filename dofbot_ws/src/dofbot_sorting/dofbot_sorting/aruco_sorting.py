#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import cv2 as cv
from dofbot_interface.msg import *
import time
from std_msgs.msg import Bool

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber_node')
        
        # 1. 创建TF缓存和监听器（核心）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. 设置定时器，周期性查询TF变换（10Hz）
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 3. 配置要查询的坐标系（替换为你的实际坐标系名）
        self.aruco11_frame = "aruco_marker_11"    # 目标坐标系（父坐标系）
        self.aruco22_frame = "aruco_marker_22"    # 目标坐标系（父坐标系）
        self.source_frame = "base_link"   # 源坐标系（子坐标系）
        
        self.get_logger().info("TF订阅节点已启动，开始查询 {} → {} 的变换".format(
            self.aruco11_frame ,self.source_frame
        ))
        self.pose_11 = [0,0,0]
        self.pose_22 = [0,0,0]
        self.rgb_bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/aruco/result',self.ImageCallback,1)
        self.height_limit = 0.04
        self.pubPos_flag = False
        self.pos_info_pub = self.create_publisher(AprilTagInfo, "PosInfo", qos_profile=10)
        self.get_pose_11 = False
        self.get_pose_22 = False
        self.sub_grasp_status = self.create_subscription(Bool,"grasp_done",self.get_graspStatusCallBack,100)
        self.grasp_done = True


    def get_graspStatusCallBack(self,msg):
        if msg.data == True:
            time.sleep(2)
            self.pubPos_flag = True
            self.grasp_done = True
    

    def ImageCallback(self,color_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'rgb8') 
        key = cv2.waitKey(1)
        if key == 32:
            self.pubPos_flag = True   
        if self.pubPos_flag == True:
            if self.pose_11[2]>self.height_limit and self.get_pose_11 == True:
                self.pubPos_flag = False
                pos = AprilTagInfo()
                pos.x = self.pose_11[0]
                pos.y = self.pose_11[1]
                pos.z = self.pose_11[2]
                self.pos_info_pub.publish(pos)
                self.get_pose_11 = False
                self.grasp_done = False
                self.pose_11 = [0,0,0]
                
            elif self.pose_22[2]>self.height_limit and self.get_pose_22 == True:
                self.pubPos_flag = False
                pos = AprilTagInfo()
                pos.x = self.pose_22[0]
                pos.y = self.pose_22[1]
                pos.z = self.pose_22[2]
                self.pos_info_pub.publish(pos)  
                self.get_pose_22 = False
                self.grasp_done = False
                self.pose_22 = [0,0,0]

            

                
        result_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("result_image", result_image)
        

    

    def timer_callback(self):
        """周期性查询TF变换"""
        try:
            # 4. 查询最新的TF变换（核心API）
            # 参数说明：
            #   target_frame: 目标坐标系
            #   source_frame: 源坐标系
            #   rclpy.time.Time()：查询最新的变换
            #   timeout: 超时时间（避免阻塞）
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.aruco11_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 5. 解析TF变换数据（平移+旋转）
            # 平移（x,y,z，单位：米）
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
            trans_z = transform.transform.translation.z
            if self.grasp_done == True:
                self.pose_11[0] = trans_x
                self.pose_11[1] = trans_y
                self.pose_11[2] = trans_z
                self.get_pose_11 = True
                print("self.pose_11: ",self.pose_11)

            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.aruco22_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 5. 解析TF变换数据（平移+旋转）
            # 平移（x,y,z，单位：米）
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
            trans_z = transform.transform.translation.z
            if self.grasp_done == True:
                self.pose_22[0] = trans_x
                self.pose_22[1] = trans_y
                self.pose_22[2] = trans_z
                self.get_pose_22 = True
                print("self.pose_22: ",self.pose_22)      
            
        except LookupException:
            # 坐标系不存在/未找到
            self.get_logger().warn(f"未找到 {self.source_frame} → {self.aruco11_frame} 的TF变换")
        except ConnectivityException:
            # TF树连接异常
            self.get_logger().error("TF树连接异常，请检查TF发布节点是否运行")
        except ExtrapolationException:
            # 时间戳异常（如查询的时间超出缓存范围）
            self.get_logger().warn("TF变换时间戳异常，可能是发布频率过低")
        except Exception as e:
            # 其他异常
            self.get_logger().error(f"查询TF变换失败：{str(e)}")

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建并运行节点
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    
    # 销毁节点
    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()