# ROS2 + RoArm M3 + AnyGrasp Integration Setup

This document provides comprehensive setup instructions for integrating AnyGrasp motion planning and grasping solution with ROS2 and the RoArm M3 robotic arm.

## Overview

This setup includes:
- **ROS2 Humble** with MoveIt2 motion planning
- **RoArm M3** robotic arm support with driver and URDF models
- **AnyGrasp SDK** for 6DOF grasp detection
- **Intel ROS2 Grasp Library** as an alternative grasping solution
- **RealSense D405** camera integration for depth perception
- **Native ROS2 environment** for optimal performance

## Prerequisites

- ROS2 Humble Desktop Full installed
- Python 3.8+ with pip
- At least 8GB RAM and 20GB disk space
- USB access for RoArm M3 and RealSense camera

## Quick Start

### 1. Set Up ROS2 Environment

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash
cd /root/ros2_workspace
```

### 2. Install Dependencies

```bash
# Install required Python packages
pip3 install numpy opencv-python open3d
sudo apt install python3-rosdep python3-colcon-common-extensions
```

### 3. Set Up AnyGrasp SDK

```bash
# Run the AnyGrasp setup script
./setup_anygrasp.sh
```

This will:
- Clone the AnyGrasp SDK
- Install Python dependencies
- Generate your machine's Feature ID for license registration
- Create ROS2 integration packages

### 4. Register for AnyGrasp License

1. Note the Feature ID displayed by the setup script
2. Visit [AnyGrasp Registration](https://graspnet.net/anygrasp.html)
3. Fill out the license application form
4. Wait for license approval (1-2 business days)
5. Download the license file to `src/anygrasp_workspace/anygrasp_sdk/license_registration/`

### 5. Complete AnyGrasp Setup

After receiving your license:

```bash
source /ros2_ws/setup_anygrasp_post_license.sh
```

### 6. Build the ROS2 Workspace

```bash
cd /root/ros2_workspace
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Basic RoArm M3 Control

```bash
# Terminal 1: Start the robot driver
ros2 run roarm_driver roarm_driver

# Terminal 2: Launch MoveIt2 planning interface
ros2 launch roarm_moveit interact.launch.py
```

### Full AnyGrasp Integration

```bash
# Launch the complete pipeline with Foxglove
ros2 launch roarm_moveit roarm_anygrasp_demo.launch.py
```

This launches:
- RoArm M3 driver
- MoveIt2 motion planning
- RealSense camera
- AnyGrasp grasp detection
- Integrated grasping pipeline
- **Foxglove Bridge for web visualization**

### Foxglove Studio Integration

```bash
# Launch Foxglove-specific configuration
ros2 launch roarm_moveit foxglove_roarm.launch.py

# Access via web browser or Foxglove Studio app
# URL: ws://YOUR_ROBOT_IP:8765
```

**Foxglove Features:**
- 🖥️ **Web-based control interface** accessible from any device
- 🤖 **3D robot visualization** with real-time updates
- 📷 **Camera streams** and point cloud visualization
- 🎯 **Interactive grasp selection** and execution
- 📊 **System monitoring** and diagnostics
- 🎮 **Custom control panels** for robot operation

See [FOXGLOVE_SETUP.md](FOXGLOVE_SETUP.md) for detailed instructions.

### Camera-only Setup

```bash
# Test RealSense camera
ros2 launch realsense_launch d405_rgbd.launch.py

# View point cloud
rviz2
# Add PointCloud2 display with topic: /camera/depth/color/points
```

### Alternative: Intel ROS2 Grasp Library

If AnyGrasp licensing is not available, use the Intel ROS2 Grasp Library:

```bash
# Launch Intel grasp detection
ros2 launch ros2_grasp_library intel_grasp_demo.launch.py
```

## Configuration

### Camera Configuration

Edit camera parameters in `/root/ros2_workspace/src/realsense_launch/config/d405_config.yaml`:

```yaml
camera:
  pointcloud:
    enable: true
  depth_module:
    depth_profile: 640,480,30
  rgb_camera:
    color_profile: 640,480,30
```

### AnyGrasp Parameters

Configure grasp detection in the launch file:

```python
anygrasp_node = Node(
    package='roarm_anygrasp_integration',
    executable='anygrasp_node',
    parameters=[{
        'detection_confidence_threshold': 0.8,  # Adjust sensitivity
        'max_grasps': 10,                       # Maximum grasps to detect
        'gripper_width': 0.08,                  # RoArm M3 gripper width
    }]
)
```

### MoveIt2 Configuration

The RoArm M3 MoveIt2 configuration is located in:
- `/root/ros2_workspace/src/roarm_main/roarm_moveit/config/`
- Key files: `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`

## Troubleshooting

### Common Issues

1. **License Error**: Ensure license file is in correct location and valid
2. **Camera Not Detected**: Check USB permissions and device mounting
3. **Robot Connection Issues**: Verify serial port permissions (`sudo chmod 666 /dev/ttyUSB0`)
4. **Build Errors**: Update package dependencies with `rosdep install --from-paths src --ignore-src -r -y`

### Debug Commands

```bash
# Check topics
ros2 topic list

# Monitor grasp poses
ros2 topic echo /anygrasp/grasp_poses

# View TF tree
ros2 run tf2_tools view_frames

# Check node status
ros2 node list
```

## Development

### Adding Custom Grasp Strategies

1. Extend the `AnyGraspNode` class in `roarm_anygrasp_integration/anygrasp_node.py`
2. Implement custom filtering and ranking algorithms
3. Add parameters for fine-tuning

### Integration with Custom Planners

1. Create new MoveIt2 planning pipeline configurations
2. Implement custom constraint solvers
3. Add collision avoidance strategies

## Hardware Setup

### RoArm M3 Connection

1. Connect RoArm M3 via USB-C to the middle port on PCB
2. Power on with 12V 5A supply
3. Verify connection: `ls /dev/tty*` should show `/dev/ttyUSB0`

### RealSense D405 Setup

1. Connect via USB 3.0
2. Install RealSense SDK
3. Test with `realsense-viewer` (if available) or ROS2 launch

## Performance Optimization

### GPU Acceleration

For faster grasp detection, ensure NVIDIA drivers are properly installed:

```bash
# Check NVIDIA driver installation
nvidia-smi

# Install NVIDIA drivers if needed
sudo apt install nvidia-driver-470
```

### Memory Management

- Adjust point cloud downsampling for performance
- Configure grasp detection ROI to reduce computation
- Use efficient collision checking in MoveIt2

## Support and Resources

- **RoArm M3 Documentation**: See `/root/ros2_workspace/src/RoArm-M3/README.md`
- **AnyGrasp SDK**: https://github.com/graspnet/anygrasp_sdk
- **Intel ROS2 Grasp Library**: https://github.com/intel/ros2_grasp_library
- **MoveIt2 Documentation**: https://moveit.ros.org/

## License

This integration follows the respective licenses of:
- RoArm M3: Open source components
- AnyGrasp: Requires commercial license
- Intel ROS2 Grasp Library: Apache 2.0
- ROS2 and MoveIt2: Apache 2.0