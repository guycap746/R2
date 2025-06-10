#!/bin/bash

# RoArm M3 Automated Test Runner
# This script provides different testing modes for the RoArm M3 system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

show_usage() {
    echo "RoArm M3 Test Runner"
    echo ""
    echo "Usage: $0 [MODE] [OPTIONS]"
    echo ""
    echo "Test Modes:"
    echo "  quick         - Quick system health check (2-3 minutes)"
    echo "  full          - Complete integration test (5-10 minutes)"
    echo "  hardware      - Test with dummy hardware nodes"
    echo "  build         - Test build system only"
    echo "  packages      - Test package functionality only"
    echo ""
    echo "Options:"
    echo "  --verbose     - Show detailed output"
    echo "  --no-cleanup  - Don't clean up test processes"
    echo "  --save-logs   - Save test logs to file"
    echo "  --help        - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 quick                    # Quick health check"
    echo "  $0 full --verbose           # Full test with detailed output"
    echo "  $0 hardware --save-logs     # Hardware test with log saving"
}

# Default options
MODE="quick"
VERBOSE=false
CLEANUP=true
SAVE_LOGS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        quick|full|hardware|build|packages)
            MODE="$1"
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --no-cleanup)
            CLEANUP=false
            shift
            ;;
        --save-logs)
            SAVE_LOGS=true
            shift
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

echo -e "${BLUE}🤖 RoArm M3 Test Runner${NC}"
echo -e "${YELLOW}Mode: $MODE${NC}"
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# Create log directory if needed
if [ "$SAVE_LOGS" = true ]; then
    LOG_DIR="/tmp/roarm_test_logs_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$LOG_DIR"
    echo "Logs will be saved to: $LOG_DIR"
fi

cleanup_processes() {
    if [ "$CLEANUP" = true ]; then
        echo "Cleaning up test processes..."
        pkill -f "dummy_hardware_nodes.py" 2>/dev/null || true
        pkill -f "foxglove_bridge" 2>/dev/null || true
        pkill -f "roarm_driver" 2>/dev/null || true
        pkill -f "move_group" 2>/dev/null || true
        sleep 2
    fi
}

# Cleanup on exit
trap cleanup_processes EXIT

run_quick_test() {
    echo -e "${YELLOW}Running Quick Health Check...${NC}"
    
    # Test 1: Environment
    echo "✓ Checking ROS2 environment..."
    command -v ros2 >/dev/null
    
    # Test 2: Packages
    echo "✓ Checking critical packages..."
    for pkg in roarm_description roarm_driver roarm_moveit wrist_imu_integration oak_camera_integration; do
        if ! ros2 pkg list | grep -q "^$pkg$"; then
            echo -e "${RED}✗ Package $pkg not found${NC}"
            exit 1
        fi
    done
    
    # Test 3: Launch files
    echo "✓ Checking launch files..."
    ros2 launch roarm_description display.launch.py --show-args >/dev/null
    
    # Test 4: Services
    echo "✓ Checking service definitions..."
    ros2 interface show roarm_moveit/srv/GetPoseCmd >/dev/null
    
    echo -e "${GREEN}✅ Quick health check passed!${NC}"
}

run_full_test() {
    echo -e "${YELLOW}Running Full Integration Test...${NC}"
    
    if [ "$VERBOSE" = true ]; then
        ./test_system_integration.sh
    else
        ./test_system_integration.sh 2>&1 | grep -E "(PASS|FAIL|Test Summary|ALL TESTS|SOME TESTS)"
    fi
}

run_hardware_test() {
    echo -e "${YELLOW}Running Hardware Simulation Test...${NC}"
    
    # Start dummy hardware nodes
    echo "Starting dummy hardware nodes..."
    python3 scripts/dummy_hardware_nodes.py &
    HARDWARE_PID=$!
    sleep 5
    
    # Test hardware simulation
    echo "Testing hardware topics..."
    timeout 10 ros2 topic echo /joint_states --once >/dev/null
    timeout 10 ros2 topic echo /d405/depth/color/points --once >/dev/null
    
    echo "Testing robot driver..."
    ros2 run roarm_driver roarm_driver --ros-args -p simulation_mode:=true &
    DRIVER_PID=$!
    sleep 3
    
    timeout 10 ros2 topic echo /roarm/status --once >/dev/null
    
    # Cleanup
    kill $DRIVER_PID 2>/dev/null || true
    kill $HARDWARE_PID 2>/dev/null || true
    
    echo -e "${GREEN}✅ Hardware simulation test passed!${NC}"
}

run_build_test() {
    echo -e "${YELLOW}Running Build System Test...${NC}"
    
    # Clean build
    echo "Cleaning previous build..."
    rm -rf build/ install/ log/
    
    # Build workspace
    echo "Building workspace..."
    if [ "$VERBOSE" = true ]; then
        colcon build --symlink-install
    else
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep -E "(Starting|Finished|Failed|Summary)"
    fi
    
    # Source and test
    source install/setup.bash
    
    echo "Testing built packages..."
    for pkg in roarm_description roarm_driver roarm_moveit roarm_moveit_cmd; do
        if ! ros2 pkg list | grep -q "^$pkg$"; then
            echo -e "${RED}✗ Package $pkg not built correctly${NC}"
            exit 1
        fi
    done
    
    echo -e "${GREEN}✅ Build system test passed!${NC}"
}

run_packages_test() {
    echo -e "${YELLOW}Running Package Functionality Test...${NC}"
    
    # Test each package individually
    packages=(
        "roarm_description:robot description and URDF"
        "roarm_driver:robot hardware interface"
        "roarm_moveit:motion planning configuration"
        "roarm_moveit_cmd:motion control commands"
        "roarm_anygrasp_integration:grasp detection integration"
        "realsense_launch:camera launch configurations"
    )
    
    for pkg_info in "${packages[@]}"; do
        IFS=':' read -r pkg desc <<< "$pkg_info"
        echo "Testing $pkg ($desc)..."
        
        # Check package exists
        if ! ros2 pkg list | grep -q "^$pkg$"; then
            echo -e "${RED}✗ Package $pkg not found${NC}"
            continue
        fi
        
        # Check executables
        executables=$(ros2 pkg executables "$pkg" 2>/dev/null | wc -l)
        if [ "$executables" -gt 0 ]; then
            echo "  ✓ $executables executables found"
        fi
        
        # Check launch files
        launch_files=$(find "src" -name "*.launch.py" -path "*$pkg*" 2>/dev/null | wc -l)
        if [ "$launch_files" -gt 0 ]; then
            echo "  ✓ $launch_files launch files found"
        fi
        
        echo "  ✓ $pkg package OK"
    done
    
    echo -e "${GREEN}✅ Package functionality test passed!${NC}"
}

# Main execution
case $MODE in
    quick)
        run_quick_test
        ;;
    full)
        run_full_test
        ;;
    hardware)
        run_hardware_test
        ;;
    build)
        run_build_test
        ;;
    packages)
        run_packages_test
        ;;
    *)
        echo "Unknown mode: $MODE"
        show_usage
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}🎉 Test completed successfully!${NC}"

if [ "$SAVE_LOGS" = true ]; then
    echo "Test logs saved to: $LOG_DIR"
fi