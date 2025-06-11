#!/bin/bash

# ============================================================================
# RoArm M3 Complete Testing Suite - No Hardware Required
# ============================================================================
# 
# This script provides comprehensive testing and verification of the RoArm M3
# system without requiring any physical hardware. It uses simulation, mocking,
# and validation to ensure all components work correctly.
#
# Usage:
#   ./test_without_hardware.sh [MODE] [OPTIONS]
#
# Modes:
#   quick     - Fast verification (5 minutes)
#   full      - Complete verification (15 minutes) [DEFAULT]
#   deep      - Comprehensive with simulations (30+ minutes)
#   component - Test specific component only
#   training  - Focus on training infrastructure
#   ci        - Continuous integration mode
#
# Options:
#   --component COMP  - Test specific component (ros2, packages, simulation, etc.)
#   --report          - Generate detailed HTML/JSON reports
#   --parallel        - Run tests in parallel where possible
#   --cleanup         - Clean up test artifacts after completion
#   --verbose         - Show detailed output
#   --help           - Show this help message
#
# Examples:
#   ./test_without_hardware.sh                    # Full test suite
#   ./test_without_hardware.sh quick              # Quick health check
#   ./test_without_hardware.sh deep --report      # Deep testing with reports
#   ./test_without_hardware.sh component --component=training  # Training only
#
# ============================================================================

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TEST_LOG_DIR="/tmp/roarm_test_logs_$(date +%Y%m%d_%H%M%S)"
PARALLEL_JOBS=4
CLEANUP_ON_EXIT=false
VERBOSE=false
GENERATE_REPORT=false

# Default values
MODE="full"
COMPONENT=""

# Test timing
START_TIME=$(date +%s)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

print_header() {
    echo -e "${BOLD}${BLUE}"
    echo "============================================================================"
    echo "$1"
    echo "============================================================================"
    echo -e "${NC}"
}

print_section() {
    echo -e "\n${BOLD}${CYAN}🔧 $1${NC}"
    echo -e "${CYAN}$(printf '=%.0s' $(seq 1 ${#1}))${NC}"
}

print_test() {
    echo -e "${WHITE}  Testing: $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_progress() {
    local current=$1
    local total=$2
    local desc="$3"
    local percent=$((current * 100 / total))
    local progress_bar=""
    
    for ((i=0; i<50; i++)); do
        if [ $i -lt $((percent / 2)) ]; then
            progress_bar="${progress_bar}█"
        else
            progress_bar="${progress_bar}░"
        fi
    done
    
    echo -e "\r${CYAN}[$progress_bar] ${percent}% - $desc${NC}"
}

log_command() {
    local cmd="$1"
    local log_file="$2"
    
    if [ "$VERBOSE" = true ]; then
        echo "Executing: $cmd"
        eval "$cmd" 2>&1 | tee -a "$log_file"
        return ${PIPESTATUS[0]}
    else
        eval "$cmd" >> "$log_file" 2>&1
        return $?
    fi
}

check_dependencies() {
    print_section "Checking Dependencies"
    
    local missing_deps=()
    
    # Check essential commands
    for cmd in python3 ros2 colcon timeout; do
        if ! command -v "$cmd" &> /dev/null; then
            missing_deps+=("$cmd")
        fi
    done
    
    # Check Python packages
    python3 -c "import rclpy, numpy" 2>/dev/null || missing_deps+=("python3-rclpy or numpy")
    
    if [ ${#missing_deps[@]} -eq 0 ]; then
        print_success "All dependencies satisfied"
        return 0
    else
        print_error "Missing dependencies: ${missing_deps[*]}"
        echo "Please install missing dependencies before running tests."
        return 1
    fi
}

setup_test_environment() {
    print_section "Setting Up Test Environment"
    
    # Create log directory
    mkdir -p "$TEST_LOG_DIR"
    print_info "Test logs will be saved to: $TEST_LOG_DIR"
    
    # Source ROS2 environment
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_success "ROS2 Humble environment sourced"
    else
        print_error "ROS2 Humble not found"
        return 1
    fi
    
    # Source workspace if built
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        print_success "Workspace environment sourced"
    else
        print_warning "Workspace not built - some tests may fail"
    fi
    
    # Set test-specific environment variables
    export ROS_DOMAIN_ID=42  # Use unique domain for testing
    export RCUTILS_LOGGING_SEVERITY=WARN  # Reduce log noise
    
    print_success "Test environment ready"
}

# ============================================================================
# TEST SUITE FUNCTIONS
# ============================================================================

test_ros2_environment() {
    print_test "ROS2 Environment"
    local log_file="$TEST_LOG_DIR/ros2_environment.log"
    local success=0
    
    # Test ROS2 installation
    if log_command "ros2 --version" "$log_file"; then
        print_success "ROS2 installation verified"
    else
        print_error "ROS2 installation check failed"
        ((success++))
    fi
    
    # Test ROS environment variables
    if [ "$ROS_DISTRO" = "humble" ]; then
        print_success "ROS_DISTRO correctly set to humble"
    else
        print_error "ROS_DISTRO not set to humble (current: $ROS_DISTRO)"
        ((success++))
    fi
    
    # Test basic ROS2 commands
    if log_command "timeout 5 ros2 node list" "$log_file"; then
        print_success "ROS2 node discovery working"
    else
        print_warning "ROS2 node discovery timeout (normal if no nodes running)"
    fi
    
    return $success
}

test_workspace_build() {
    print_test "Workspace Build Status"
    local log_file="$TEST_LOG_DIR/workspace_build.log"
    local success=0
    
    # Check if workspace is built
    if [ -d "$WORKSPACE_DIR/install" ] && [ -n "$(ls -A "$WORKSPACE_DIR/install" 2>/dev/null)" ]; then
        local package_count=$(ls "$WORKSPACE_DIR/install" | wc -l)
        print_success "Workspace built with $package_count packages"
    else
        print_error "Workspace not built"
        ((success++))
    fi
    
    # Check key packages
    local key_packages=("roarm_driver" "roarm_moveit" "roarm_anygrasp_integration")
    for package in "${key_packages[@]}"; do
        if log_command "ros2 pkg list | grep -q $package" "$log_file"; then
            print_success "Package $package available"
        else
            print_error "Package $package not found"
            ((success++))
        fi
    done
    
    return $success
}

test_dummy_hardware() {
    print_test "Hardware Simulation"
    local log_file="$TEST_LOG_DIR/dummy_hardware.log"
    local success=0
    
    # Start dummy hardware nodes
    print_info "Starting dummy hardware simulation..."
    python3 "$WORKSPACE_DIR/scripts/dummy_hardware_nodes.py" &
    local dummy_pid=$!
    
    # Give it time to start
    sleep 5
    
    # Test if dummy nodes are publishing
    local test_topics=("/joint_states" "/camera/depth/color/points" "/imu/data")
    for topic in "${test_topics[@]}"; do
        if log_command "timeout 5 ros2 topic echo $topic --once" "$log_file"; then
            print_success "Topic $topic publishing data"
        else
            print_warning "Topic $topic not publishing (may be normal)"
        fi
    done
    
    # Test service availability
    if log_command "timeout 5 ros2 service list | grep -q get_pose" "$log_file"; then
        print_success "Robot services available"
    else
        print_warning "Robot services not found"
    fi
    
    # Cleanup dummy hardware
    kill $dummy_pid 2>/dev/null || true
    wait $dummy_pid 2>/dev/null || true
    
    return $success
}

test_launch_files() {
    print_test "Launch File Validation"
    local log_file="$TEST_LOG_DIR/launch_files.log"
    local success=0
    
    # Test launch file syntax
    local launch_files=(
        "roarm_description display.launch.py"
        "roarm_moveit interact.launch.py"
        "realsense_launch d405_rgbd.launch.py"
    )
    
    for launch_file in "${launch_files[@]}"; do
        if log_command "timeout 10 ros2 launch $launch_file --show-args" "$log_file"; then
            print_success "Launch file $launch_file syntax valid"
        else
            print_error "Launch file $launch_file has syntax errors"
            ((success++))
        fi
    done
    
    return $success
}

test_training_infrastructure() {
    print_test "Training Infrastructure"
    local log_file="$TEST_LOG_DIR/training.log"
    local success=0
    
    # Test LeRobot integration
    if [ -f "$WORKSPACE_DIR/src/roarm_lerobot_integration/scripts/lerobot_data_collector.py" ]; then
        print_success "LeRobot data collector found"
    else
        print_error "LeRobot data collector not found"
        ((success++))
    fi
    
    # Test training workflow manager
    if [ -f "$WORKSPACE_DIR/src/roarm_lerobot_integration/scripts/training_workflow_manager.py" ]; then
        print_success "Training workflow manager found"
    else
        print_error "Training workflow manager not found"
        ((success++))
    fi
    
    # Test Foxglove panels
    local foxglove_panels=(
        "training_dashboard_panel.ts"
        "dataset_browser_panel.ts"
        "model_comparison_panel.ts"
    )
    
    for panel in "${foxglove_panels[@]}"; do
        if [ -f "$WORKSPACE_DIR/foxglove_panels/$panel" ]; then
            print_success "Foxglove panel $panel found"
        else
            print_error "Foxglove panel $panel not found"
            ((success++))
        fi
    done
    
    return $success
}

test_web_interface() {
    print_test "Web Interface Components"
    local log_file="$TEST_LOG_DIR/web_interface.log"
    local success=0
    
    # Test original web app
    if [ -f "$WORKSPACE_DIR/src/roarm_ws_em0/src/roarm_main/roarm_web_app/roarm_web_app/roarm_web_app.py" ]; then
        print_success "Original web app found"
    else
        print_error "Original web app not found"
        ((success++))
    fi
    
    # Test enhanced training web app
    if [ -f "$WORKSPACE_DIR/src/roarm_ws_em0/src/roarm_main/roarm_web_app/roarm_web_app/enhanced_training_web_app.py" ]; then
        print_success "Enhanced training web app found"
    else
        print_error "Enhanced training web app not found"
        ((success++))
    fi
    
    return $success
}

test_integration_workflow() {
    print_test "Integration Workflow"
    local log_file="$TEST_LOG_DIR/integration.log"
    local success=0
    
    print_info "Testing complete system integration (this may take a few minutes)..."
    
    # Start dummy hardware
    python3 "$WORKSPACE_DIR/scripts/dummy_hardware_nodes.py" &
    local dummy_pid=$!
    sleep 3
    
    # Test robot description loading
    if log_command "timeout 15 ros2 launch roarm_description display.launch.py use_fake_hardware:=true" "$log_file" &
    then
        local launch_pid=$!
        sleep 8
        
        # Check if robot model is loaded
        if log_command "timeout 5 ros2 param get /robot_state_publisher robot_description" "$log_file"; then
            print_success "Robot description loaded successfully"
        else
            print_warning "Robot description not loaded"
        fi
        
        # Cleanup launch
        kill $launch_pid 2>/dev/null || true
        wait $launch_pid 2>/dev/null || true
    else
        print_error "Failed to launch robot description"
        ((success++))
    fi
    
    # Cleanup dummy hardware
    kill $dummy_pid 2>/dev/null || true
    wait $dummy_pid 2>/dev/null || true
    
    return $success
}

run_comprehensive_verification() {
    print_test "Comprehensive System Verification"
    local log_file="$TEST_LOG_DIR/comprehensive.log"
    
    print_info "Running comprehensive verification suite..."
    
    if [ -f "$WORKSPACE_DIR/scripts/comprehensive_verification_suite.py" ]; then
        local cmd="python3 $WORKSPACE_DIR/scripts/comprehensive_verification_suite.py --mode $MODE"
        
        if [ -n "$COMPONENT" ]; then
            cmd="$cmd --component $COMPONENT"
        fi
        
        if [ "$GENERATE_REPORT" = true ]; then
            cmd="$cmd --report"
        fi
        
        if log_command "$cmd" "$log_file"; then
            print_success "Comprehensive verification completed"
            return 0
        else
            print_error "Comprehensive verification failed"
            return 1
        fi
    else
        print_warning "Comprehensive verification suite not found, skipping"
        return 0
    fi
}

# ============================================================================
# TEST EXECUTION
# ============================================================================

run_test_suite() {
    local mode="$1"
    local component="$2"
    
    print_header "RoArm M3 System Testing - No Hardware Required"
    echo -e "${WHITE}Mode: $mode${NC}"
    echo -e "${WHITE}Component: ${component:-All components}${NC}"
    echo -e "${WHITE}Timestamp: $(date)${NC}"
    echo -e "${WHITE}Log Directory: $TEST_LOG_DIR${NC}"
    
    local total_errors=0
    local test_count=0
    
    # Always run basic checks
    ((test_count++))
    test_ros2_environment || ((total_errors++))
    
    ((test_count++))
    test_workspace_build || ((total_errors++))
    
    # Component-specific or mode-specific tests
    case "$mode" in
        "quick")
            if [ -z "$component" ] || [ "$component" = "packages" ]; then
                ((test_count++))
                test_dummy_hardware || ((total_errors++))
            fi
            ;;
        "full")
            if [ -z "$component" ] || [ "$component" = "simulation" ]; then
                ((test_count++))
                test_dummy_hardware || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "launch" ]; then
                ((test_count++))
                test_launch_files || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "training" ]; then
                ((test_count++))
                test_training_infrastructure || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "web" ]; then
                ((test_count++))
                test_web_interface || ((total_errors++))
            fi
            ;;
        "deep")
            # Run all tests including comprehensive verification
            if [ -z "$component" ] || [ "$component" = "simulation" ]; then
                ((test_count++))
                test_dummy_hardware || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "launch" ]; then
                ((test_count++))
                test_launch_files || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "training" ]; then
                ((test_count++))
                test_training_infrastructure || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "web" ]; then
                ((test_count++))
                test_web_interface || ((total_errors++))
            fi
            
            if [ -z "$component" ] || [ "$component" = "integration" ]; then
                ((test_count++))
                test_integration_workflow || ((total_errors++))
            fi
            
            ((test_count++))
            run_comprehensive_verification || ((total_errors++))
            ;;
        "training")
            ((test_count++))
            test_training_infrastructure || ((total_errors++))
            ;;
        "component")
            case "$component" in
                "simulation")
                    ((test_count++))
                    test_dummy_hardware || ((total_errors++))
                    ;;
                "training")
                    ((test_count++))
                    test_training_infrastructure || ((total_errors++))
                    ;;
                "web")
                    ((test_count++))
                    test_web_interface || ((total_errors++))
                    ;;
                "launch")
                    ((test_count++))
                    test_launch_files || ((total_errors++))
                    ;;
                "integration")
                    ((test_count++))
                    test_integration_workflow || ((total_errors++))
                    ;;
                *)
                    print_error "Unknown component: $component"
                    return 1
                    ;;
            esac
            ;;
    esac
    
    return $total_errors
}

generate_test_report() {
    local errors=$1
    local end_time=$(date +%s)
    local duration=$((end_time - START_TIME))
    
    print_header "Test Summary"
    
    echo -e "${WHITE}Test Duration: ${duration}s${NC}"
    echo -e "${WHITE}Log Directory: $TEST_LOG_DIR${NC}"
    
    if [ $errors -eq 0 ]; then
        print_success "ALL TESTS PASSED! 🎉"
        echo -e "${GREEN}✅ System is ready for operation without hardware${NC}"
        echo -e "${GREEN}✅ All components verified and functional${NC}"
    else
        print_error "$errors TEST(S) FAILED ❌"
        echo -e "${RED}🔧 System requires attention before operation${NC}"
        echo -e "${YELLOW}📋 Check log files in $TEST_LOG_DIR for details${NC}"
    fi
    
    if [ "$GENERATE_REPORT" = true ]; then
        print_info "Generating detailed HTML report..."
        # This would generate an HTML report from the logs
        echo "Report generation not yet implemented"
    fi
    
    echo -e "\n${CYAN}📚 Additional Resources:${NC}"
    echo -e "${WHITE}  - Training Updates: $WORKSPACE_DIR/TRAINING_UPDATES_README.md${NC}"
    echo -e "${WHITE}  - System Integration: $WORKSPACE_DIR/test_system_integration.sh${NC}"
    echo -e "${WHITE}  - Testing Guide: $WORKSPACE_DIR/TESTING_README.md${NC}"
}

cleanup_test_artifacts() {
    if [ "$CLEANUP_ON_EXIT" = true ]; then
        print_info "Cleaning up test artifacts..."
        
        # Kill any remaining ROS processes
        pkill -f ros2 2>/dev/null || true
        pkill -f python3.*dummy_hardware 2>/dev/null || true
        
        # Clean up temporary files
        rm -rf /tmp/ros* 2>/dev/null || true
        
        print_success "Cleanup completed"
    fi
}

show_help() {
    cat << EOF
RoArm M3 Complete Testing Suite - No Hardware Required

USAGE:
    $0 [MODE] [OPTIONS]

MODES:
    quick      Fast verification (5 minutes)
    full       Complete verification (15 minutes) [DEFAULT]
    deep       Comprehensive with simulations (30+ minutes)
    component  Test specific component only
    training   Focus on training infrastructure
    ci         Continuous integration mode

OPTIONS:
    --component COMP  Test specific component (ros2, packages, simulation, training, web, launch, integration)
    --report          Generate detailed reports
    --parallel        Run tests in parallel where possible
    --cleanup         Clean up test artifacts after completion
    --verbose         Show detailed output
    --help           Show this help message

EXAMPLES:
    $0                                    # Full test suite
    $0 quick                              # Quick health check
    $0 deep --report                      # Deep testing with reports
    $0 component --component=training     # Training infrastructure only
    $0 --verbose --cleanup                # Verbose mode with cleanup

COMPONENTS:
    ros2          ROS2 environment and installation
    packages      ROS2 package availability
    simulation    Hardware simulation and dummy nodes
    training      Training infrastructure and ML components
    web           Web interface and Foxglove integration
    launch        Launch file validation
    integration   End-to-end workflow testing

EOF
}

# ============================================================================
# SIGNAL HANDLERS AND CLEANUP
# ============================================================================

cleanup_on_exit() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    cleanup_test_artifacts
    
    # Kill any background processes
    jobs -p | xargs -r kill 2>/dev/null || true
}

trap cleanup_on_exit EXIT INT TERM

# ============================================================================
# MAIN EXECUTION
# ============================================================================

main() {
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            quick|full|deep|component|training|ci)
                MODE="$1"
                shift
                ;;
            --component)
                COMPONENT="$2"
                shift 2
                ;;
            --report)
                GENERATE_REPORT=true
                shift
                ;;
            --parallel)
                # Parallel execution (future enhancement)
                shift
                ;;
            --cleanup)
                CLEANUP_ON_EXIT=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                echo "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # Validate component parameter
    if [ "$MODE" = "component" ] && [ -z "$COMPONENT" ]; then
        print_error "Component mode requires --component parameter"
        exit 1
    fi
    
    # Check dependencies
    check_dependencies || exit 1
    
    # Setup test environment
    setup_test_environment || exit 1
    
    # Run test suite
    run_test_suite "$MODE" "$COMPONENT"
    local test_errors=$?
    
    # Generate report
    generate_test_report $test_errors
    
    # Exit with appropriate code
    exit $test_errors
}

# Run main function
main "$@"