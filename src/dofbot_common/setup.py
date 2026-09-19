from setuptools import find_packages, setup

package_name = "dofbot_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dofbot team",
    maintainer_email="you@example.com",
    description="Shared constants for Dofbot demos.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
