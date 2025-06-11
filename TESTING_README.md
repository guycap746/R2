# RoArm M3 System Testing Guide

This directory contains comprehensive testing tools for the RoArm M3 robotic grasping system. Use these tests to verify system functionality after major changes, migrations, or deployments.

## 🚀 Quick Start

```bash
# Quick health check (2-3 minutes)
./run_tests.sh quick

# Full system integration test (5-10 minutes)
./run_tests.sh full

# Test with hardware simulation
./run_tests.sh hardware
```

## 📋 Available Test Scripts

### 1. `run_tests.sh` - Main Test Runner

**Usage:**
```bash
./run_tests.sh [MODE] [OPTIONS]
```

**Test Modes:**
- `quick` - Quick system health check (2-3 minutes)
- `full` - Complete integration test (5-10 minutes) 
- `hardware` - Test with dummy hardware simulation
- `build` - Test build system only
- `packages` - Test package functionality only

**Options:**
- `--verbose` - Show detailed output
- `--no-cleanup` - Don't clean up test processes
- `--save-logs` - Save test logs to file

**Examples:**
```bash
./run_tests.sh quick                    # Quick health check
./run_tests.sh full --verbose           # Full test with details
./run_tests.sh hardware --save-logs     # Hardware test with logs
```

### 2. `test_system_integration.sh` - Comprehensive Integration Test

Detailed system test covering:
- ✅ ROS2 environment setup
- ✅ Package availability
- ✅ Launch file validation
- ✅ Service/message definitions
- ✅ Hardware simulation
- ✅ MoveIt integration
- ✅ Foxglove bridge
- ✅ Web interface components
- ✅ System integration

**Usage:**
```bash
./test_system_integration.sh
```

### 3. `scripts/dummy_hardware_nodes.py` - Hardware Simulation

Provides dummy nodes for testing without physical hardware:
- 🤖 **RoArm M3 robot arm** - Joint states and movement simulation
- 📷 **Dual RealSense D405 cameras** - Point clouds and images
- 🎯 **AnyGrasp detection** - Simulated grasp candidates
- 🗺️ **Planning scene objects** - Table and obstacles

**Usage:**
```bash
# Start all dummy hardware
python3 scripts/dummy_hardware_nodes.py

# Or start individually in separate terminals
python3 -c "from scripts.dummy_hardware_nodes import DummyRoArmDriver; import rclpy; rclpy.init(); rclpy.spin(DummyRoArmDriver())"
```

## 🔧 Test Components

### Core System Tests
- **ROS2 Environment**: Humble distro, package management
- **Workspace Build**: Colcon build system, dependencies
- **Package Integrity**: All roarm packages present and functional

### Hardware Integration Tests
- **Robot Driver**: Joint state publishing, movement simulation
- **Camera Simulation**: Point cloud data, camera info, images
- **Sensor Integration**: RealSense camera topics and services

### Motion Planning Tests
- **MoveIt Configuration**: Move group, planning scene
- **Robot Description**: URDF models, joint definitions
- **IK Solver**: Inverse kinematics plugin functionality

### Perception Tests
- **AnyGrasp Integration**: Grasp detection simulation
- **Camera Calibration**: Calibration workflow validation
- **Object Detection**: Dummy object recognition

### Visualization Tests
- **Foxglove Bridge**: WebSocket connectivity, message streaming
- **RViz Integration**: Robot visualization, planning displays
- **Web Interface**: ROS2 web components, custom panels

## 📊 Test Results

Tests provide color-coded output:
- 🟢 **PASS** - Component working correctly
- 🔴 **FAIL** - Component has issues
- 🟡 **WARNING** - Component has minor issues

### Sample Output
```
🤖 RoArm M3 System Integration Test
==========================================

Test 1: ROS2 Environment Setup
✓ PASS: ROS2 Installation
✓ PASS: ROS2 Humble Distro

Test 2: Package Availability  
✓ PASS: Package: roarm_description
✓ PASS: Package: roarm_driver
✓ PASS: Package: roarm_moveit
...

📊 Test Summary
Tests Passed: 28
Tests Failed: 0
Total Tests: 28

🎉 ALL TESTS PASSED!
✅ RoArm M3 system is ready for operation
```

## 🛠️ Troubleshooting

### Common Issues

**1. Package Not Found**
```bash
# Rebuild workspace
colcon build --symlink-install
source install/setup.bash
```

**2. ROS2 Environment Issues**
```bash
# Check ROS2 installation
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # Should show "humble"
```

**3. Hardware Simulation Fails**
```bash
# Install missing Python dependencies
pip3 install numpy

# Check ROS2 Python bindings
python3 -c "import rclpy; print('ROS2 Python OK')"
```

**4. Launch File Errors**
```bash
# Test launch file syntax
ros2 launch roarm_description display.launch.py --show-args
```

**5. Service Definition Issues**
```bash
# Rebuild messages and services
colcon build --packages-select roarm_moveit roarm_anygrasp_integration
source install/setup.bash
```

### Debug Mode

Run tests with detailed output:
```bash
./run_tests.sh full --verbose --save-logs
```

Check logs in `/tmp/roarm_test_logs_*/`

## 🎯 When to Run Tests

### Required Testing Scenarios
1. **After System Updates** - Verify native ROS2 functionality
2. **After Major Code Changes** - Ensure no regressions
3. **Before Deployment** - Validate system readiness
4. **After Environment Changes** - Verify dependencies
5. **Weekly Health Checks** - Preventive monitoring

### Test Schedule
- **Quick Test**: Daily during development
- **Full Test**: Before major deployments
- **Hardware Test**: When testing new features
- **Build Test**: After dependency changes

## 📁 Test Output Files

Tests create temporary files in:
- `/tmp/ros2_system_tests_YYYYMMDD_HHMMSS/` - Test outputs
- `/tmp/roarm_test_logs_YYYYMMDD_HHMMSS/` - Detailed logs

## 🔄 Integration with CI/CD

These tests can be integrated into automated pipelines:

```yaml
# Example GitHub Actions workflow
name: RoArm M3 Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Setup ROS2 Humble
        run: |
          sudo apt update
          sudo apt install ros-humble-desktop-full
      - name: Build Workspace
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --symlink-install
      - name: Run Tests
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          ./run_tests.sh full
```

## 📚 Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **MoveIt Documentation**: https://moveit.ros.org/
- **Foxglove Documentation**: https://foxglove.dev/docs
- **RealSense ROS2**: https://github.com/IntelRealSense/realsense-ros

## 🤝 Contributing

When adding new components:
1. Add corresponding tests to `test_system_integration.sh`
2. Update dummy hardware nodes if needed
3. Add new test modes to `run_tests.sh`
4. Update this documentation

## 📞 Support

If tests fail consistently:
1. Check system requirements and dependencies
2. Verify ROS2 Humble installation
3. Review workspace build logs
4. Consult component-specific documentation

The test system is designed to catch issues early and provide clear guidance for resolution.