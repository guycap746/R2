#!/bin/bash

# AnyGrasp SDK Setup Script for ROS2 + Roarm M3
# This script sets up AnyGrasp SDK with license registration and integration

set -e

echo "Setting up AnyGrasp SDK for ROS2 + Roarm M3..."

# Create AnyGrasp workspace
mkdir -p /ros2_ws/src/anygrasp_workspace
cd /ros2_ws/src/anygrasp_workspace

# Clone AnyGrasp SDK
echo "Cloning AnyGrasp SDK..."
git clone https://github.com/graspnet/anygrasp_sdk.git

cd anygrasp_sdk

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Install additional dependencies for point cloud processing
pip3 install \
    open3d \
    trimesh \
    scipy \
    matplotlib \
    opencv-python \
    pyyaml

# Check if license directory exists
if [ ! -d "license_registration" ]; then
    echo "Error: license_registration directory not found in AnyGrasp SDK"
    echo "Please check the repository structure or contact AnyGrasp support"
    exit 1
fi

# Get machine feature ID for license registration
echo "Getting machine feature ID for license registration..."
cd license_registration
python3 get_feature_id.py > machine_feature_id.txt
echo "Machine Feature ID saved to machine_feature_id.txt"
echo "Feature ID: $(cat machine_feature_id.txt)"

echo ""
echo "=== LICENSE REGISTRATION REQUIRED ==="
echo "1. Copy the Feature ID above: $(cat machine_feature_id.txt)"
echo "2. Visit: https://graspnet.net/anygrasp.html"
echo "3. Fill out the license application form with your Feature ID"
echo "4. Wait for license approval (usually 1-2 business days)"
echo "5. Download the license file and place it in the license_registration/ directory"
echo "6. After receiving license, run: source /ros2_ws/setup_anygrasp_post_license.sh"
echo "========================================="

# Create post-license setup script
cat > /ros2_ws/setup_anygrasp_post_license.sh << 'EOF'
#!/bin/bash
# Run this script after receiving AnyGrasp license

set -e

echo "Setting up AnyGrasp after license approval..."

cd /ros2_ws/src/anygrasp_workspace/anygrasp_sdk

# Check if license file exists
if [ ! -f "license_registration/license.json" ]; then
    echo "Error: license.json not found in license_registration/"
    echo "Please download your license file from AnyGrasp and place it there"
    exit 1
fi

# Test AnyGrasp installation
echo "Testing AnyGrasp installation..."
cd grasp_detection
python3 demo.py --checkpoint_path /ros2_ws/src/anygrasp_workspace/anygrasp_sdk/log/checkpoint_detection.tar \
                --camera realsense \
                --debug

echo "AnyGrasp SDK setup completed successfully!"
echo "You can now use AnyGrasp with your ROS2 + Roarm M3 setup"
EOF

chmod +x /ros2_ws/setup_anygrasp_post_license.sh

# Create ROS2 integration package for AnyGrasp
echo "Creating ROS2 integration package for AnyGrasp..."
cd /ros2_ws/src
ros2 pkg create --build-type ament_python roarm_anygrasp_integration \
    --dependencies rclpy geometry_msgs sensor_msgs std_msgs tf2_ros moveit_msgs

# Create basic AnyGrasp ROS2 node
cat > roarm_anygrasp_integration/roarm_anygrasp_integration/anygrasp_node.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import sys
import os

# Add AnyGrasp SDK to path
sys.path.append('/ros2_ws/src/anygrasp_workspace/anygrasp_sdk')

try:
    from gsnet import AnyGrasp
except ImportError:
    print("AnyGrasp SDK not properly installed or licensed")
    AnyGrasp = None

class AnyGraspNode(Node):
    def __init__(self):
        super().__init__('anygrasp_node')
        
        # Publishers
        self.grasp_poses_pub = self.create_publisher(PoseArray, '/anygrasp/grasp_poses', 10)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.pointcloud_callback, 10)
        
        # Initialize AnyGrasp
        if AnyGrasp is not None:
            try:
                checkpoint_path = '/ros2_ws/src/anygrasp_workspace/anygrasp_sdk/log/checkpoint_detection.tar'
                self.anygrasp = AnyGrasp(checkpoint_path)
                self.get_logger().info('AnyGrasp initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize AnyGrasp: {e}')
                self.anygrasp = None
        else:
            self.anygrasp = None
            self.get_logger().error('AnyGrasp SDK not available')
    
    def pointcloud_callback(self, msg):
        if self.anygrasp is None:
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            # This is a simplified conversion - you'll need proper conversion
            # based on your specific point cloud format
            
            # Process with AnyGrasp
            # grasp_poses = self.anygrasp.detect_grasps(point_cloud_data)
            
            # Convert to ROS message and publish
            pose_array = PoseArray()
            pose_array.header = Header()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = msg.header.frame_id
            
            # Add detected grasp poses to pose_array.poses
            # This is where you'd convert AnyGrasp output to ROS Pose messages
            
            self.grasp_poses_pub.publish(pose_array)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Create package setup.py
cat > roarm_anygrasp_integration/setup.py << 'EOF'
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
    description='AnyGrasp integration for RoArm M3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anygrasp_node = roarm_anygrasp_integration.anygrasp_node:main',
        ],
    },
)
EOF

echo ""
echo "AnyGrasp SDK setup script completed!"
echo "Next steps:"
echo "1. Register for AnyGrasp license using the Feature ID above"
echo "2. After receiving license, run: source /ros2_ws/setup_anygrasp_post_license.sh"
echo "3. Build the ROS2 workspace: cd /ros2_ws && colcon build"
echo "4. Use AnyGrasp with: ros2 run roarm_anygrasp_integration anygrasp_node"