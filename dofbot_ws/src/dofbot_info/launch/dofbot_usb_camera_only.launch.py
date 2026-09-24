from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 仅启动 USB 摄像头节点
    cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_node',
        remappings=[
            ('/image_raw', '/camera/color/image_raw'),  
        ],
        
        parameters=[{
            'video_device': '/dev/video0',  
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv'  # 指定像素格式为YUYV
            'io_method': 'mmap' 
        }]
    )

    return LaunchDescription([cam_node])
