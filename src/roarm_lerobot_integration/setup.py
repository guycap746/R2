from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'roarm_lerobot_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'training'), glob('training/*')),
    ],
    install_requires=[
        'setuptools',
        'torch>=2.0.0',
        'torchvision',
        'transformers>=4.30.0',
        'datasets>=2.10.0',
        'huggingface_hub>=0.15.0',
        'opencv-python>=4.8.0',
        'numpy>=1.21.0',
        'pillow>=8.3.0',
        'matplotlib>=3.5.0',
        'wandb>=0.15.0',
        'tqdm>=4.64.0',
        'hydra-core>=1.3.0',
        'omegaconf>=2.3.0',
        'tensorboard>=2.12.0',
        'scipy>=1.9.0',
        'scikit-learn>=1.1.0',
        'imageio>=2.20.0',
        'ffmpeg-python>=0.2.0',
    ],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='LeRobot integration for RoArm M3 robotic manipulation with Hugging Face AI models',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lerobot_bridge = roarm_lerobot_integration.lerobot_bridge:main',
            'lerobot_data_collector = roarm_lerobot_integration.lerobot_data_collector:main',
            'lerobot_policy_trainer = roarm_lerobot_integration.lerobot_policy_trainer:main',
            'lerobot_policy_executor = roarm_lerobot_integration.lerobot_policy_executor:main',
            'lerobot_teleop_interface = roarm_lerobot_integration.lerobot_teleop_interface:main',
            'lerobot_dataset_manager = roarm_lerobot_integration.lerobot_dataset_manager:main',
            'lerobot_evaluation_node = roarm_lerobot_integration.lerobot_evaluation_node:main',
        ],
    },
)