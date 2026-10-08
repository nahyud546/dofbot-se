from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dofbot_driver'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'dofbot_driver = dofbot_driver.dofbot_driver:main',
        	'arm_driver = dofbot_driver.arm_driver:main',
            'calculate_volume = dofbot_driver.calculate_volume:main',
            'grasp = dofbot_driver.grasp:main',
            'apriltag_detect = dofbot_driver.apriltag_detect:main',
            'test = dofbot_driver.test:main',
            'apriltag_list = dofbot_driver.apriltag_list:main',
            'apriltag_remove_higher = dofbot_driver.apriltag_remove_higher:main'
           
        ],
    },
)
