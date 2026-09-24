from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_markers',
            executable='aruco_markers',
            name='aruco_detect_node',
            parameters=[
                {'image_topic': '/image_raw'},
                {'camera_info_topic': '/camera_info'},
                {'aruco_dictionary': 'DICT_ARUCO_ORIGINAL'},
                {'marker_size': 0.03},
                {'publish_tf': True}
            ]
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'camera_link',
            'camera_rgb_optical_frame'
        ]
    ),
        Node(
        package='dofbot_ctrl',
        executable='arm_ctrl',
        name='arm_ctrl_node'
    ),
        Node(
        package='dofbot_ctrl',
        executable='urdf_arm_ctrl',
        name='urdf_arm_ctrl_node'
    ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        FindPackageShare('dofbot_urdf'),
        '/launch/urdf_display.launch.py'
        ]))
    ])