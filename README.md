# ROS2 RoArm M3 Development Environment

This is a native ROS2 Humble development environment for the RoArm M3 robotic manipulation system.

## Quick Start

1. **Install ROS2 Humble:**
   ```bash
   # Install ROS2 Humble Desktop Full
   sudo apt update
   sudo apt install ros-humble-desktop-full
   source /opt/ros/humble/setup.bash
   ```

2. **Build your workspace:**
   ```bash
   cd /root/ros2_workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Create your first package:**
   ```bash
   cd src
   ros2 pkg create --build-type ament_python my_package
   ```

## Directory Structure

- `src/` - Your ROS2 packages source code
- `install/` - Built packages (auto-generated)
- `build/` - Build artifacts (auto-generated) 
- `log/` - Build logs (auto-generated)

## Available Commands

### Build Commands
- `colcon build` - Build all packages
- `colcon build --symlink-install` - Build with symbolic links
- `colcon build --packages-select <package>` - Build specific package
- `colcon test` - Run tests
- `colcon test --packages-select <package>` - Test specific package

## Features

- ROS2 Humble Desktop Full
- Python development tools (pytest, flake8, mypy, black)
- Debugging tools (gdb, valgrind)
- GUI support for RViz and other visualization tools
- Persistent workspace
- Native ROS2 communication

## Integrated Systems

- **RoArm M3 Robot**: 6-DOF robotic arm with gripper
- **AnyGrasp Integration**: AI-powered grasp detection
- **MoveIt2**: Motion planning and control
- **Camera Integration**: RealSense D405 and OAK-D cameras
- **Foxglove Studio**: Web-based visualization and control
- **LeRobot Integration**: Dataset collection and training
- **Isaac Sim**: Photorealistic simulation environment

## Getting Started

1. **Hardware Setup**: Connect RoArm M3 and cameras
2. **Launch System**: Use provided launch files
3. **Test Integration**: Run system tests
4. **Start Development**: Create custom packages

## Portability

All ROS2 code and configurations are stored in the `src/` directory, making it easy to:
- Version control your entire workspace
- Share projects between team members  
- Move between different development machines
- Deploy to different environments