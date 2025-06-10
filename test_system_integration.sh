#!/bin/bash

# RoArm M3 System Integration Test
# Tests all major components with dummy hardware simulation
# Run this after major changes to verify system functionality

set -e  # Exit on any error

echo "=========================================="
echo "🤖 RoArm M3 System Integration Test"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results tracking
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_TESTS=()

# Function to print test status
print_test_result() {
    local test_name="$1"
    local status="$2"
    local details="$3"
    
    if [ "$status" = "PASS" ]; then
        echo -e "${GREEN}✓ PASS${NC}: $test_name"
        ((TESTS_PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC}: $test_name"
        if [ -n "$details" ]; then
            echo -e "  ${YELLOW}Details:${NC} $details"
        fi
        ((TESTS_FAILED++))
        FAILED_TESTS+=("$test_name")
    fi
}

# Function to run command with timeout
run_with_timeout() {
    local timeout_duration="$1"
    local command="$2"
    
    timeout "$timeout_duration" bash -c "$command" 2>/dev/null
    return $?
}

# Setup environment
echo -e "${BLUE}Setting up test environment...${NC}"
cd /root/ros2_workspace

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS2 Humble environment sourced"
else
    echo "ROS2 Humble not found at /opt/ros/humble/"
    exit 1
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Workspace environment sourced"
else
    echo "Workspace not built - run 'colcon build' first"
    exit 1
fi

# Set ROS domain to avoid conflicts
export ROS_DOMAIN_ID=42

# Create test output directory
TEST_OUTPUT_DIR="/tmp/ros2_system_tests_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_OUTPUT_DIR"
echo "Test outputs will be saved to: $TEST_OUTPUT_DIR"

echo -e "${BLUE}Starting system integration tests...${NC}"
echo ""

# ==========================================
# Test 1: ROS2 Environment Setup
# ==========================================
echo -e "${YELLOW}Test 1: ROS2 Environment Setup${NC}"

if command -v ros2 >/dev/null 2>&1; then
    print_test_result "ROS2 Installation" "PASS"
else
    print_test_result "ROS2 Installation" "FAIL" "ROS2 command not found"
fi

if [ -n "$ROS_DISTRO" ] && [ "$ROS_DISTRO" = "humble" ]; then
    print_test_result "ROS2 Humble Distro" "PASS"
else
    print_test_result "ROS2 Humble Distro" "FAIL" "Expected humble, got $ROS_DISTRO"
fi

# ==========================================
# Test 2: Package Availability
# ==========================================
echo ""
echo -e "${YELLOW}Test 2: Package Availability${NC}"

# List of critical packages to test
CRITICAL_PACKAGES=(
    "roarm_description"
    "roarm_driver" 
    "roarm_moveit"
    "roarm_moveit_cmd"
    "roarm_anygrasp_integration"
    "realsense_launch"
    "foxglove_bridge"
)

for package in "${CRITICAL_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$package$"; then
        print_test_result "Package: $package" "PASS"
    else
        print_test_result "Package: $package" "FAIL" "Package not found"
    fi
done

# ==========================================
# Test 3: Launch File Validation
# ==========================================
echo ""
echo -e "${YELLOW}Test 3: Launch File Validation${NC}"

# Test launch file syntax
if ros2 launch roarm_description display.launch.py --show-args >/dev/null 2>&1; then
    print_test_result "Robot Description Launch" "PASS"
else
    print_test_result "Robot Description Launch" "FAIL" "Launch file syntax error"
fi

if ros2 launch realsense_launch d405.launch.py --show-args >/dev/null 2>&1; then
    print_test_result "RealSense Camera Launch" "PASS"
else
    print_test_result "RealSense Camera Launch" "FAIL" "Launch file syntax error"
fi

# ==========================================
# Test 4: Service and Message Definitions
# ==========================================
echo ""
echo -e "${YELLOW}Test 4: Service and Message Definitions${NC}"

# Test custom services
SERVICES=(
    "roarm_moveit/srv/GetPoseCmd"
    "roarm_moveit/srv/MovePointCmd"
    "roarm_anygrasp_integration/srv/GetGraspCandidates"
    "roarm_anygrasp_integration/srv/SelectGrasp"
)

for service in "${SERVICES[@]}"; do
    if ros2 interface show "$service" >/dev/null 2>&1; then
        print_test_result "Service: $service" "PASS"
    else
        print_test_result "Service: $service" "FAIL" "Service definition not found"
    fi
done

# ==========================================
# Test 5: Dummy Hardware Simulation
# ==========================================
echo ""
echo -e "${YELLOW}Test 5: Dummy Hardware Simulation${NC}"

# Start dummy robot driver in background
echo "Starting dummy robot driver..."
ros2 run roarm_driver roarm_driver --ros-args -p simulation_mode:=true &
ROBOT_PID=$!
sleep 3

if kill -0 $ROBOT_PID 2>/dev/null; then
    print_test_result "Dummy Robot Driver" "PASS"
else
    print_test_result "Dummy Robot Driver" "FAIL" "Driver failed to start"
fi

# Test robot driver topics
if run_with_timeout 5 "ros2 topic list | grep -q '/roarm'"; then
    print_test_result "Robot Driver Topics" "PASS"
else
    print_test_result "Robot Driver Topics" "FAIL" "Robot topics not found"
fi

# Clean up robot driver
if kill -0 $ROBOT_PID 2>/dev/null; then
    kill $ROBOT_PID
    wait $ROBOT_PID 2>/dev/null || true
fi

# ==========================================
# Test 6: Dummy Camera Simulation  
# ==========================================
echo ""
echo -e "${YELLOW}Test 6: Dummy Camera Simulation${NC}"

# Create dummy point cloud publisher
cat > "$TEST_OUTPUT_DIR/dummy_camera.py" << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import Header
import sys

class DummyCameraNode(Node):
    def __init__(self):
        super().__init__('dummy_camera_node')
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/d405/depth/color/points', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/d405/color/camera_info', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.get_logger().info('Dummy camera node started')

    def publish_data(self):
        # Publish dummy point cloud
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'd405_color_optical_frame'
        pc_msg.height = 480
        pc_msg.width = 640
        self.pointcloud_pub.publish(pc_msg)
        
        # Publish dummy camera info
        cam_info = CameraInfo()
        cam_info.header = pc_msg.header
        cam_info.height = 480
        cam_info.width = 640
        self.camera_info_pub.publish(cam_info)

def main():
    rclpy.init()
    node = DummyCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x "$TEST_OUTPUT_DIR/dummy_camera.py"

# Start dummy camera
echo "Starting dummy camera..."
python3 "$TEST_OUTPUT_DIR/dummy_camera.py" &
CAMERA_PID=$!
sleep 3

if kill -0 $CAMERA_PID 2>/dev/null; then
    print_test_result "Dummy Camera Node" "PASS"
else
    print_test_result "Dummy Camera Node" "FAIL" "Camera node failed to start"
fi

# Test camera topics
if run_with_timeout 5 "ros2 topic list | grep -q '/d405'"; then
    print_test_result "Camera Topics Available" "PASS"
else
    print_test_result "Camera Topics Available" "FAIL" "Camera topics not found"
fi

# Test IMU topics
if run_with_timeout 5 "ros2 topic list | grep -q '/wrist_imu'"; then
    print_test_result "Wrist IMU Topics Available" "PASS"
else
    print_test_result "Wrist IMU Topics Available" "FAIL" "IMU topics not found"
fi

# Test OAK camera topics
if run_with_timeout 5 "ros2 topic list | grep -q '/oak_d'"; then
    print_test_result "OAK-D Topics Available" "PASS"
else
    print_test_result "OAK-D Topics Available" "FAIL" "OAK-D topics not found"
fi

if run_with_timeout 5 "ros2 topic list | grep -q '/oak_1'"; then
    print_test_result "OAK-1 Topics Available" "PASS"
else
    print_test_result "OAK-1 Topics Available" "FAIL" "OAK-1 topics not found"
fi

# Test point cloud data
if run_with_timeout 10 "ros2 topic echo /d405/depth/color/points --once"; then
    print_test_result "Point Cloud Data" "PASS"
else
    print_test_result "Point Cloud Data" "FAIL" "No point cloud data received"
fi

# Test IMU data
if run_with_timeout 10 "ros2 topic echo /wrist_imu/imu --once"; then
    print_test_result "Wrist IMU Data" "PASS"
else
    print_test_result "Wrist IMU Data" "FAIL" "No IMU data received"
fi

# Clean up camera
if kill -0 $CAMERA_PID 2>/dev/null; then
    kill $CAMERA_PID
    wait $CAMERA_PID 2>/dev/null || true
fi

# ==========================================
# Test 7: MoveIt Integration
# ==========================================
echo ""
echo -e "${YELLOW}Test 7: MoveIt Integration${NC}"

# Test move_group node startup (brief test)
echo "Testing MoveIt move_group startup..."
timeout 15 ros2 launch roarm_moveit move_group.launch.py use_fake_hardware:=true &
MOVEIT_PID=$!
sleep 10

if kill -0 $MOVEIT_PID 2>/dev/null; then
    print_test_result "MoveIt Move Group" "PASS"
    
    # Test MoveIt services
    if run_with_timeout 5 "ros2 service list | grep -q 'move_group'"; then
        print_test_result "MoveIt Services" "PASS"
    else
        print_test_result "MoveIt Services" "FAIL" "MoveIt services not available"
    fi
else
    print_test_result "MoveIt Move Group" "FAIL" "Move group failed to start"
fi

# Clean up MoveIt
if kill -0 $MOVEIT_PID 2>/dev/null; then
    kill $MOVEIT_PID
    wait $MOVEIT_PID 2>/dev/null || true
fi

# ==========================================
# Test 8: Foxglove Bridge
# ==========================================
echo ""
echo -e "${YELLOW}Test 8: Foxglove Bridge${NC}"

# Test foxglove bridge startup
echo "Testing Foxglove bridge..."
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 &
FOXGLOVE_PID=$!
sleep 3

if kill -0 $FOXGLOVE_PID 2>/dev/null; then
    print_test_result "Foxglove Bridge Startup" "PASS"
    
    # Test WebSocket connection (basic check)
    if netstat -ln | grep -q ":8765"; then
        print_test_result "Foxglove WebSocket Port" "PASS"
    else
        print_test_result "Foxglove WebSocket Port" "FAIL" "Port 8765 not listening"
    fi
else
    print_test_result "Foxglove Bridge Startup" "FAIL" "Bridge failed to start"
fi

# Clean up Foxglove
if kill -0 $FOXGLOVE_PID 2>/dev/null; then
    kill $FOXGLOVE_PID
    wait $FOXGLOVE_PID 2>/dev/null || true
fi

# ==========================================
# Test 9: ROS2 Web Integration
# ==========================================
echo ""
echo -e "${YELLOW}Test 9: ROS2 Web Integration${NC}"

# Test ros2web packages
if ros2 pkg list | grep -q "ros2web"; then
    print_test_result "ROS2 Web Packages" "PASS"
else
    print_test_result "ROS2 Web Packages" "FAIL" "ros2web packages not found"
fi

# Test web app interface definitions
if ros2 interface show ros2web_interfaces/msg/WebRTCSignal >/dev/null 2>&1; then
    print_test_result "Web Interface Messages" "PASS"
else
    print_test_result "Web Interface Messages" "FAIL" "Web interface messages not found"
fi

# ==========================================
# Test 10: System Integration
# ==========================================
echo ""
echo -e "${YELLOW}Test 10: System Integration${NC}"

# Test TF tree
if run_with_timeout 10 "ros2 run tf2_tools view_frames.py --help"; then
    print_test_result "TF2 Tools Available" "PASS"
else
    print_test_result "TF2 Tools Available" "FAIL" "TF2 tools not working"
fi

# Test parameter server
if ros2 param list >/dev/null 2>&1; then
    print_test_result "Parameter Server" "PASS"
else
    print_test_result "Parameter Server" "FAIL" "Parameter server not responding"
fi

# Test rosbag2 functionality
if ros2 bag --help >/dev/null 2>&1; then
    print_test_result "ROS Bag Recording" "PASS"
else
    print_test_result "ROS Bag Recording" "FAIL" "rosbag2 not working"
fi

# ==========================================
# Test Summary
# ==========================================
echo ""
echo "=========================================="
echo -e "${BLUE}📊 Test Summary${NC}"
echo "=========================================="

echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo -e "Total Tests:  $(($TESTS_PASSED + $TESTS_FAILED))"

if [ $TESTS_FAILED -eq 0 ]; then
    echo ""
    echo -e "${GREEN}🎉 ALL TESTS PASSED!${NC}"
    echo -e "${GREEN}✅ RoArm M3 system is ready for operation${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}❌ SOME TESTS FAILED:${NC}"
    for test in "${FAILED_TESTS[@]}"; do
        echo -e "  ${RED}•${NC} $test"
    done
    echo ""
    echo -e "${YELLOW}⚠️  Please review failed tests before proceeding${NC}"
    exit 1
fi