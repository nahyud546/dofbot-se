from setuptools import setup
import os

package_name = 'dofbot_pro_kcf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='yahboom',
    maintainer='yahboom',
    description='DOFBOT Pro KCF tracking package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'KCF_Tracker = ' + package_name + '.KCF_Tracker:main',
            'Dofbot_Track = ' + package_name + '.Dofbot_Track:main',
            'KCF_TrackAndGrap = ' + package_name + '.KCF_TrackAndGrap:main',
        ],
    },
)