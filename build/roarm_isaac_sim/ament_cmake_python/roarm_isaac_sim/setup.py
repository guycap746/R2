from setuptools import find_packages
from setuptools import setup

setup(
    name='roarm_isaac_sim',
    version='0.0.0',
    packages=find_packages(
        include=('roarm_isaac_sim', 'roarm_isaac_sim.*')),
)
