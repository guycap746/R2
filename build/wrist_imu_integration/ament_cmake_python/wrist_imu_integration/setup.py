from setuptools import find_packages
from setuptools import setup

setup(
    name='wrist_imu_integration',
    version='1.0.0',
    packages=find_packages(
        include=('wrist_imu_integration', 'wrist_imu_integration.*')),
)
