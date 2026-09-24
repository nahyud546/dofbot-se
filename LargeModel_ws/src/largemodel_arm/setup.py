from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'largemodel_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ALM_KCF_Tracker = largemodel_arm.ALM_KCF_Tracker:main',
            'apriltag_sorting = largemodel_arm.apriltag_sorting:main',
            'Detect_Obj = largemodel_arm.Detect_Obj:main',
            'grasp = largemodel_arm.grasp:main',
            'grasp_obj = largemodel_arm.grasp_obj:main',
            'yolov11_garbage = largemodel_arm.yolov11_garbage:main',
            'point_to = largemodel_arm.point_to:main',
            'Change_Pose = largemodel_arm.Change_Pose:main',
            'apriltag_follow_2D = largemodel_arm.apriltag_follow_2D:main',
            'Obj_follow = largemodel_arm.Obj_follow:main',
            'Record_Video = largemodel_arm.Record_Video:main',
            'Record_pose = largemodel_arm.Record_pose:main'
        ],
    },
)