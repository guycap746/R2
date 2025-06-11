#!/bin/bash

# Test script for ROS2 + RoArm M3 + AnyGrasp setup validation

set -e

echo "Testing ROS2 + RoArm M3 + AnyGrasp Setup..."

# Check ROS2 environment
if [ -n "$ROS_DISTRO" ]; then
    echo "✅ ROS2 environment detected: $ROS_DISTRO"
else
    echo "⚠️  ROS2 environment not sourced"
fi

# Test ROS2 installation
echo "Testing ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 found: $(ros2 --version)"
else
    echo "❌ ROS2 not found"
    exit 1
fi

# Test workspace setup
echo "Testing workspace setup..."
if [ -d "/root/ros2_workspace" ]; then
    echo "✅ Workspace directory exists"
    cd /root/ros2_workspace
    
    # Check for key packages
    if [ -d "src/roarm_main" ]; then
        echo "✅ RoArm packages found"
    else
        echo "❌ RoArm packages not found"
    fi
    
    if [ -d "src/RoArm-M3" ]; then
        echo "✅ RoArm M3 documentation found"
    else
        echo "❌ RoArm M3 documentation not found"
    fi
    
    if [ -d "src/realsense_launch" ]; then
        echo "✅ RealSense packages found"
    else
        echo "❌ RealSense packages not found"
    fi
else
    echo "❌ Workspace directory not found"
    exit 1
fi

# Test if build files exist
echo "Testing build status..."
if [ -d "install" ] && [ -d "build" ]; then
    echo "✅ Build directories exist"
    
    # Source and test basic ROS2 functionality
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "✅ Workspace sourced successfully"
        
        # Test package availability
        if ros2 pkg list | grep -q "roarm_driver"; then
            echo "✅ RoArm driver package available"
        else
            echo "⚠️  RoArm driver package not built yet"
        fi
        
        if ros2 pkg list | grep -q "roarm_moveit"; then
            echo "✅ RoArm MoveIt package available"
        else
            echo "⚠️  RoArm MoveIt package not built yet"
        fi
    else
        echo "⚠️  Workspace not built yet"
    fi
else
    echo "⚠️  Workspace not built yet"
fi

# Test AnyGrasp setup
echo "Testing AnyGrasp setup..."
if [ -f "setup_anygrasp.sh" ]; then
    echo "✅ AnyGrasp setup script found"
else
    echo "❌ AnyGrasp setup script not found"
fi

if [ -d "src/anygrasp_workspace" ]; then
    echo "✅ AnyGrasp workspace directory exists"
    
    if [ -d "src/anygrasp_workspace/anygrasp_sdk" ]; then
        echo "✅ AnyGrasp SDK cloned"
        
        if [ -f "src/anygrasp_workspace/anygrasp_sdk/license_registration/license.json" ]; then
            echo "✅ AnyGrasp license found"
        else
            echo "⚠️  AnyGrasp license not installed yet"
        fi
    else
        echo "⚠️  AnyGrasp SDK not cloned yet"
    fi
else
    echo "⚠️  AnyGrasp workspace not set up yet"
fi

# Test Python dependencies
echo "Testing Python dependencies..."
python3 -c "import numpy; print('✅ NumPy available')" 2>/dev/null || echo "❌ NumPy not available"
python3 -c "import cv2; print('✅ OpenCV available')" 2>/dev/null || echo "❌ OpenCV not available"
python3 -c "import open3d; print('✅ Open3D available')" 2>/dev/null || echo "❌ Open3D not available"

# Test launch files
echo "Testing launch files..."
if [ -f "src/roarm_main/roarm_moveit/launch/roarm_anygrasp_demo.launch.py" ]; then
    echo "✅ AnyGrasp demo launch file found"
else
    echo "❌ AnyGrasp demo launch file not found"
fi

echo ""
echo "=== Setup Status Summary ==="
echo "🔧 To complete setup:"
echo "1. Source ROS2: source /opt/ros/humble/setup.bash"
echo "2. Build workspace: cd /root/ros2_workspace && colcon build --symlink-install"
echo "3. Set up AnyGrasp: ./setup_anygrasp.sh"
echo "4. Register for AnyGrasp license"
echo "5. Complete AnyGrasp setup: source setup_anygrasp_post_license.sh"
echo ""
echo "🚀 To test the system:"
echo "1. Connect RoArm M3 and RealSense camera"
echo "2. Run: ros2 launch roarm_moveit roarm_anygrasp_demo.launch.py"
echo ""