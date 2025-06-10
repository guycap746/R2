from setuptools import setup

package_name = 'roarm_anygrasp_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Interactive AnyGrasp integration for RoArm M3 with user selection workflow',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anygrasp_node = roarm_anygrasp_integration.anygrasp_node:main',
            'anygrasp_interactive_node = roarm_anygrasp_integration.anygrasp_interactive_node:main',
            'grasp_coordinator = roarm_anygrasp_integration.grasp_coordinator:main',
            'roboflow_integration_node = roarm_anygrasp_integration.roboflow_integration_node:main',
            'dual_camera_capture_node = roarm_anygrasp_integration.dual_camera_capture_node:main',
            'grasp_verification_coordinator = roarm_anygrasp_integration.grasp_verification_coordinator:main',
            'charuco_calibration_node = roarm_anygrasp_integration.charuco_calibration_node:main',
            'hand_eye_calibration_node = roarm_anygrasp_integration.hand_eye_calibration_node:main',
        ],
    },
)