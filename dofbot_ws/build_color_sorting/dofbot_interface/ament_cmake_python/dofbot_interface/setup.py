from setuptools import find_packages
from setuptools import setup

setup(
    name='dofbot_interface',
    version='0.0.0',
    packages=find_packages(
        include=('dofbot_interface', 'dofbot_interface.*')),
)
