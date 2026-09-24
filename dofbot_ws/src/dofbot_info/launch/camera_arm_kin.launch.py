from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kin_node = Node(
     package='dofbot_info',
     executable='kinemarics_dofbot',
     name='kinemarics_dofbot',
    )

    cam_node = Node(
     package='usb_cam',
     executable='usb_cam_node_exe',
     name='camera_node',

     parameters=[{
         'video_device': '/dev/video0',
         'image_width': 640,
         'image_height': 480,
         'framerate': 10.0,     #如果想调高帧率改这里，但可能造成部分程序兼容性问题    
         'pixel_format': 'yuyv',
         'io_method': 'mmap',

         'focus_auto': False,
         'white_balance_temperature_auto': False,
         'exposure_auto': 1
     }],

     output='screen'
    )

    return LaunchDescription([kin_node, cam_node])


