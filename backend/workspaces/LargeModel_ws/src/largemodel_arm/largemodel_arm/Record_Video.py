from cv_bridge import CvBridge
import cv2
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
from Arm_Lib import Arm_Device
import os


class RecordVideoNode(Node):
    def __init__(self):
        super().__init__('video_recording_node')
        self.Path_video = "/home/yahboom/save_video.mp4"
        self.output_path = "/home/yahboom/record_video.mp4"
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = None
        self.largemodel_arm_done_pub = self.create_publisher(String,'/largemodel_arm_done',1)
        self.rgb_bridge = CvBridge()
        self.Time_ = 0.0
        self.declare_parameter('time_', 20.0)
        self.Time_ = int(self.get_parameter('time_').get_parameter_value().double_value)
        print("Get self.target_id is ",self.Time_)
        self.record_done = False
        self.Arm = Arm_Device()
        self.beep_start = False
        self.start_time = time.time()
        self.sub_rgb = self.create_subscription(Image,"/image_raw",self.get_RGBCallBack,100)
        print("init done.")
        

    def get_RGBCallBack(self,msg):
        
        if self.beep_start==False:
            self.start_time = time.time()
            self.beep_start = True
            self.Arm.Arm_Buzzer_On()
            time.sleep(0.5)
            self.Arm.Arm_Buzzer_Off()
            self.record_done = False
            
        if self.record_done!=True: 
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg,'bgr8')
            if self.out is None:
                #print("-----------")
                height, width, _ = rgb_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4编码器
                self.out = cv2.VideoWriter(
                    self.Path_video, 
                    fourcc, 
                    30, 
                    (width, height)
                )
            #print("self.out: ",self.out)
            self.out.write(rgb_image)
            cv2.imshow('Video Recording', rgb_image)
            key = cv2.waitKey(1)
            elapsed = time.time() - self.start_time 
            if elapsed >= self.Time_ :
                cv2.destroyAllWindows()
                self.record_done = True
                self.out.release()
                self.Arm.Arm_Buzzer_On()
                time.sleep(0.5)
                self.Arm.Arm_Buzzer_Off()
                self.compress_video()
                

    def compress_video(self):
        scale_ratio  = 0.5
        target_fps = 15.0
        codec = 'mp4v'
        cap = cv2.VideoCapture(self.Path_video)
        if not cap.isOpened():
            print(f"错误：无法打开输入视频 {self.Path_video}")
            return False
    
        # 3. 获取原视频参数
        original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        original_fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        target_width = int(original_width * scale_ratio)
        target_height = int(original_height * scale_ratio)
        target_width = target_width if target_width % 2 == 0 else target_width - 1
        target_height = target_height if target_height % 2 == 0 else target_height - 1
        
        # 4.2 计算帧抽取间隔（用于降低帧率）
        frame_interval = max(1, int(original_fps / target_fps))
    
        # 4.3 设置编码格式（优先兼容的mp4v）
        fourcc = cv2.VideoWriter_fourcc(*codec)
    
        # 5. 初始化视频写入器

        out = cv2.VideoWriter(
            self.output_path,
            fourcc=fourcc,
            fps=target_fps,
            frameSize=(target_width, target_height),
        )
        
        if not out.isOpened():
            print(f"错误：无法初始化视频写入器，可能是编码 {codec} 不支持！")
            cap.release()
            return False
            
        processed_frames = 0
        current_frame = 0  

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break  # 读取完所有帧
            
            # 按间隔抽取帧（降低帧率）
            if current_frame % frame_interval == 0:
                # 缩放分辨率
                resized_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)            
                # 写入压缩帧
                out.write(resized_frame)
                processed_frames += 1
            
            current_frame += 1
    
            # 打印进度（可选）
            if current_frame % 100 == 0:
                progress = (current_frame / total_frames) * 100
                print(f"进度：{progress:.1f}% ({current_frame}/{total_frames}帧)")

        # 8. 输出压缩结果
        original_size = os.path.getsize(self.Path_video) / (1024 * 1024)  # MB
        compressed_size = os.path.getsize(self.output_path) / (1024 * 1024)  # MB
        compression_ratio = (1 - compressed_size / original_size) * 100
    
        print(f"\n压缩完成！")
        print(f"原文件大小：{original_size:.2f} MB")
        print(f"压缩后大小：{compressed_size:.2f} MB")
        print(f"压缩率：{compression_ratio:.1f}%")
        self.largemodel_arm_done_pub.publish(String(data='record_video_done'))

        
                    
def main(args=None):
    rclpy.init(args=args)
    record_video = RecordVideoNode()
    try:    
        rclpy.spin(record_video)
    except KeyboardInterrupt:
        pass
    finally:
        record_video.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()            
            
        
            
        
        
        
        
        
        
    
    