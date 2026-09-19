from setuptools import find_packages, setup

package_name = "cap_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/cap_mapping.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/camera.yaml",
            "config/homography.yaml",
        ]),
    ],
    # OpenCV/numpy/pyyaml cài ở môi trường Python (apt python3-opencv,
    # python3-numpy, python3-yaml); cv_bridge là ROS package trong image.
    # Không khai báo chúng ở đây để colcon không gọi pip trong lúc build offline.
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AnhDuy",
    maintainer_email="lehoanhduy5426@gmail.com",
    description="Camera that cho cap_sorting: homography + detect nap + RViz.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_test = cap_vision.camera_test:main",
            "calibrate_table = cap_vision.calibrate_table:main",
            "test_pixel_to_xy = cap_vision.test_pixel_to_xy:main",
            "cap_detector = cap_vision.cap_detector:main",
            "cap_rviz_publisher = cap_vision.cap_rviz_publisher:main",
        ],
    },
)
