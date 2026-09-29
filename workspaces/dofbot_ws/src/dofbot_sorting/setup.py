from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'dofbot_sorting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),glob(os.path.join('config', '*.yaml'))),
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
         'apriltag_sorting = dofbot_sorting.apriltag_sorting:main',
         'grasp = dofbot_sorting.grasp:main',
         'objection_grasp = dofbot_sorting.objection_grasp:main',
         'color_sorting = dofbot_sorting.color_sorting:main',
         'mediapipe_gesture = dofbot_sorting.mediapipe_gesture:main',
         'apriltag_gesture_id = dofbot_sorting.apriltag_gesture_id:main',
         'point_to = dofbot_sorting.point_to:main',
         'apriltag_gesture_dist = dofbot_sorting.apriltag_gesture_dist:main',
         'shape_sorting = dofbot_sorting.shape_sorting:main',
         'aruco_sorting = dofbot_sorting.aruco_sorting:main',
         'aruco_gesture_height = dofbot_sorting.aruco_gesture_height:main',
         'get_offset = dofbot_sorting.get_offset:main',
         'yolov11_garbage = dofbot_sorting.yolov11_garbage:main',
        ],
    },
)
