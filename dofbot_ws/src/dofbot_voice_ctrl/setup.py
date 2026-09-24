from setuptools import find_packages, setup

package_name = 'dofbot_voice_ctrl'

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
            'apriltag_detect = dofbot_voice_ctrl.AprilTag.apriltag_detect:main',
            'grasp_VC = dofbot_voice_ctrl.AprilTag.apriltag_grasp_VC:main',
            'apriltag_follow_2D = dofbot_voice_ctrl.AprilTag.apriltag_follow_2D:main',
            'color_sorting = dofbot_voice_ctrl.Color.color_sorting:main' ,
            'color_follow_2D = dofbot_voice_ctrl.Color.color_follow_2D:main',
            'yolov11_garbage = dofbot_voice_ctrl.Yolov11.yolov11_garbage:main',
            'KCF_follow = dofbot_voice_ctrl.KCF.KCF_follow:main'
        ],
    },
)
