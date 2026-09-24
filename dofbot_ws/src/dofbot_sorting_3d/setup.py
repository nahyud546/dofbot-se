from setuptools import find_packages, setup

package_name = 'dofbot_sorting_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1324760365@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'apriltag_sorting = dofbot_sorting_3d.apriltag_sorting:main',
         'grasp = dofbot_sorting_3d.grasp:main',
         'objection_grasp = dofbot_sorting_3d.objection_grasp:main',
         'color_sorting = dofbot_sorting_3d.color_sorting:main',
         'mediapipe_gesture = dofbot_sorting_3d.mediapipe_gesture:main',
         'apriltag_gesture_id = dofbot_sorting_3d.apriltag_gesture_id:main',
         'point_to = dofbot_sorting_3d.point_to:main',
         'apriltag_gesture_dist = dofbot_sorting_3d.apriltag_gesture_dist:main',
         'shape_sorting = dofbot_sorting_3d.shape_sorting:main',
         'aruco_sorting = dofbot_sorting_3d.aruco_sorting:main',
         'yolov11_garbage = dofbot_sorting_3d.yolov11_garbage:main',
         'aruco_gesture_height = dofbot_sorting_3d.aruco_gesture_height:main',
         'apriltag_sorting_voice = dofbot_sorting_3d.apriltag_sorting_voice:main',
         'cam_pub = dofbot_sorting_3d.cam_pub:main',
        ],
    },
)
