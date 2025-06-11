from setuptools import setup
import os
from glob import glob

package_name = 'xarm_6_isaac_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'environments'), glob('environments/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoArm Team',
    maintainer_email='dev@roarm.com',
    description='Isaac Sim integration for UFactory xArm 6 robotic arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xarm_6_isaac_launcher = xarm_6_isaac_sim.xarm_6_isaac_launcher:main',
            'xarm_6_simulation_controller = xarm_6_isaac_sim.xarm_6_simulation_controller:main',
        ],
    },
)