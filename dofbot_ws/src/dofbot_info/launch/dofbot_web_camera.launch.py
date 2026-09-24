from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 声明启动
    open_view_arg = DeclareLaunchArgument(
        'open_view',
        default_value='false',
        description='Whether to open image_view window (true/false)'
    )

    # 2. 启动 usb_cam 节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',  # ROS2 usb_cam 节点的可执行文件名称
        name='usb_cam',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},  # 摄像头设备路径
            {'image_width': 640},            # 图像宽度
            {'image_height': 480},           # 图像高度
            {'pixel_format': 'yuyv'},        # 像素格式
            {'camera_frame_id': 'usb_cam'},  # 摄像头坐标系ID
            {'io_method': 'mmap'}            # 读写方式（内存映射）
        ],
        remappings=[
            ('/image_raw', '/usb_cam/image_raw'),  
            ('/camera_info', '/usb_cam/camera_info')
        ]
    )

    # 3. 启动 web_video_server 节点
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[
            {'port': 8080}  # web 8080
        ]
    )

    # 4. 条件启动 image_view
    image_view_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('open_view')),
        actions=[
            Node(
                package='image_view',
                executable='image_view',
                name='image_view',
                output='screen',
                remappings=[
                    ('image', '/usb_cam/image_raw')  
                ],
                parameters=[
                    {'autosize': True}  # 自动调整窗口大小
                ]
            )
        ]
    )

    # 5. 组装所有启动项
    return LaunchDescription([
        open_view_arg,
        usb_cam_node,
        web_video_server_node,
        image_view_group
    ])
