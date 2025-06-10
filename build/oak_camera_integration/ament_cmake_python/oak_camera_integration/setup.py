from setuptools import find_packages
from setuptools import setup

setup(
    name='oak_camera_integration',
    version='1.0.0',
    packages=find_packages(
        include=('oak_camera_integration', 'oak_camera_integration.*')),
)
