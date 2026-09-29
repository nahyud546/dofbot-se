from setuptools import find_packages, setup

package_name = 'dofbot_follow'

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
            'apriltag_follow_2D = dofbot_follow.apriltag_follow_2D:main',
            'color_follow_2D = dofbot_follow.color_follow_2D:main',
            'KCF_follow = dofbot_follow.KCF_follow:main'
        ],
    },
)
