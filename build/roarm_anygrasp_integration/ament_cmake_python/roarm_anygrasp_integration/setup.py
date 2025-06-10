from setuptools import find_packages
from setuptools import setup

setup(
    name='roarm_anygrasp_integration',
    version='0.0.0',
    packages=find_packages(
        include=('roarm_anygrasp_integration', 'roarm_anygrasp_integration.*')),
)
