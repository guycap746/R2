from setuptools import find_packages
from setuptools import setup

setup(
    name='roarm_lerobot_integration',
    version='0.0.0',
    packages=find_packages(
        include=('roarm_lerobot_integration', 'roarm_lerobot_integration.*')),
)
