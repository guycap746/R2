#!/bin/bash

# R2 RoArm M3 System Setup Script for Google Cloud VM
# This script automatically sets up the complete R2 system on a fresh Ubuntu 22.04 VM

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="$HOME/ros2_workspace"
ROS_DISTRO="humble"
PYTHON_VERSION="3.10"

# Logging functions
log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

log_header() {
    echo -e "\n${PURPLE}🚀 $1${NC}"
    echo -e "${PURPLE}$(printf '=%.0s' {1..50})${NC}"
}

# Check if running as root
check_user() {
    if [[ $EUID -eq 0 ]]; then
        log_error "This script should not be run as root"
        exit 1
    fi
}

# Check system requirements
check_system() {
    log_header "Checking System Requirements"
    
    # Check Ubuntu version
    if ! grep -q "22.04" /etc/os-release; then
        log_warning "This script is designed for Ubuntu 22.04. Continuing anyway..."
    fi
    
    # Check available memory
    TOTAL_MEM=$(free -g | awk '/^Mem:/{print $2}')
    if [[ $TOTAL_MEM -lt 8 ]]; then
        log_warning "Less than 8GB RAM detected. Some operations may be slow."
    fi
    
    # Check available disk space
    AVAILABLE_SPACE=$(df / | awk 'NR==2{print $4}')
    if [[ $AVAILABLE_SPACE -lt 20971520 ]]; then  # 20GB in KB
        log_warning "Less than 20GB free space available. Consider expanding disk."
    fi
    
    log_success "System check completed"
}

# Update system packages
update_system() {
    log_header "Updating System Packages"
    
    sudo apt update
    sudo apt upgrade -y
    
    # Install essential tools
    sudo apt install -y \
        curl \
        wget \
        git \
        vim \
        nano \
        htop \
        build-essential \
        software-properties-common \
        apt-transport-https \
        ca-certificates \
        gnupg \
        lsb-release \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev
    
    log_success "System packages updated"
}

# Install ROS2 Humble
install_ros2() {
    log_header "Installing ROS2 Humble"
    
    # Add ROS2 repository
    sudo add-apt-repository universe -y
    
    # Add ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add ROS2 repository to sources
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update package list
    sudo apt update
    
    # Install ROS2 Humble Desktop Full
    log_info "Installing ROS2 Humble Desktop Full (this may take a while)..."
    sudo apt install -y ros-humble-desktop-full
    
    # Install development tools
    sudo apt install -y ros-dev-tools python3-colcon-common-extensions
    
    # Install additional ROS2 packages
    log_info "Installing additional ROS2 packages..."
    sudo apt install -y \
        ros-humble-moveit \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-tf2-tools \
        ros-humble-rqt \
        ros-humble-rviz2
    
    # Source ROS2 in bashrc
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    source /opt/ros/humble/setup.bash
    
    log_success "ROS2 Humble installed successfully"
}

# Install Python dependencies
install_python_deps() {
    log_header "Installing Python Dependencies"
    
    # Upgrade pip
    python3 -m pip install --upgrade pip
    
    # Install essential Python packages
    log_info "Installing essential Python packages..."
    pip3 install \
        numpy \
        scipy \
        matplotlib \
        opencv-python \
        transforms3d \
        pyyaml \
        lxml \
        pillow \
        requests \
        flask \
        fastapi \
        uvicorn \
        websockets \
        jsonschema
    
    # Install AI/ML packages
    log_info "Installing AI/ML packages..."
    pip3 install \
        torch \
        torchvision \
        torchaudio \
        --index-url https://download.pytorch.org/whl/cpu
    
    # Install computer vision packages
    pip3 install \
        ultralytics \
        roboflow \
        supervision
    
    log_success "Python dependencies installed"
}

# Clone and setup workspace
setup_workspace() {
    log_header "Setting Up R2 Workspace"
    
    # Remove existing workspace if it exists
    if [[ -d "$WORKSPACE_DIR" ]]; then
        log_warning "Existing workspace found. Backing up..."
        mv "$WORKSPACE_DIR" "${WORKSPACE_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
    fi
    
    # Clone repository
    log_info "Cloning R2 repository..."
    git clone https://github.com/guycap746/R2.git "$WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
    
    # Install Python requirements if available
    if [[ -f "requirements.txt" ]]; then
        log_info "Installing Python requirements from repository..."
        pip3 install -r requirements.txt
    fi
    
    log_success "Workspace cloned successfully"
}

# Build ROS2 workspace
build_workspace() {
    log_header "Building ROS2 Workspace"
    
    cd "$WORKSPACE_DIR"
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    
    # Build workspace
    log_info "Building workspace (this may take several minutes)..."
    colcon build --symlink-install --parallel-workers $(nproc)
    
    # Source workspace in bashrc
    if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
        echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    fi
    
    source "$WORKSPACE_DIR/install/setup.bash"
    
    log_success "Workspace built successfully"
}

# Configure environment
configure_environment() {
    log_header "Configuring Environment"
    
    # Set environment variables
    cat >> ~/.bashrc << 'EOF'

# R2 RoArm M3 System Configuration
export ROS_DOMAIN_ID=42
export ROBOT_SIMULATION=true
export USE_FAKE_HARDWARE=true
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=$PYTHONPATH:$HOME/ros2_workspace/src

# Performance optimizations
export MAKEFLAGS="-j$(nproc)"

EOF
    
    # Create useful aliases
    cat >> ~/.bashrc << 'EOF'
# R2 System Aliases
alias r2_build="cd ~/ros2_workspace && colcon build --symlink-install"
alias r2_source="source ~/ros2_workspace/install/setup.bash"
alias r2_clean="cd ~/ros2_workspace && rm -rf build install log"
alias r2_test="cd ~/ros2_workspace && colcon test"
alias r2_gazebo="ros2 launch xarm_6_gazebo xarm_6_gazebo.launch.py"
alias r2_moveit="ros2 launch xarm_6_moveit_config demo.launch.py"
alias r2_rviz="rviz2"

EOF
    
    log_success "Environment configured"
}

# Install additional tools
install_additional_tools() {
    log_header "Installing Additional Development Tools"
    
    # Install VS Code (optional)
    if command -v code &> /dev/null; then
        log_info "VS Code already installed"
    else
        log_info "Installing VS Code..."
        wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
        sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
        sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
        sudo apt update
        sudo apt install -y code
    fi
    
    # Install Docker (for future use)
    if ! command -v docker &> /dev/null; then
        log_info "Installing Docker..."
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt update
        sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
        sudo usermod -aG docker $USER
    fi
    
    # Install system monitoring tools
    sudo apt install -y \
        tree \
        ncdu \
        iotop \
        nethogs \
        tmux \
        screen
    
    log_success "Additional tools installed"
}

# Run system tests
run_tests() {
    log_header "Running System Tests"
    
    cd "$WORKSPACE_DIR"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # Test ROS2 installation
    log_info "Testing ROS2 installation..."
    timeout 10s ros2 run demo_nodes_cpp talker &
    TALKER_PID=$!
    sleep 2
    timeout 5s ros2 run demo_nodes_py listener &
    LISTENER_PID=$!
    sleep 3
    kill $TALKER_PID $LISTENER_PID 2>/dev/null || true
    
    # Test workspace build
    log_info "Testing workspace packages..."
    if ros2 pkg list | grep -q "roarm"; then
        log_success "RoArm packages found"
    else
        log_warning "RoArm packages not found - may need manual investigation"
    fi
    
    # Test MoveIt installation
    log_info "Testing MoveIt installation..."
    if ros2 pkg list | grep -q "moveit"; then
        log_success "MoveIt packages found"
    else
        log_warning "MoveIt packages not found"
    fi
    
    log_success "System tests completed"
}

# Create desktop shortcuts and convenience scripts
create_shortcuts() {
    log_header "Creating Convenience Scripts"
    
    # Create desktop directory if it doesn't exist
    mkdir -p ~/Desktop
    
    # Create startup script
    cat > ~/start_r2_system.sh << 'EOF'
#!/bin/bash
# R2 System Startup Script

echo "🚀 Starting R2 RoArm M3 System..."

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/ros2_workspace/install/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=42
export ROBOT_SIMULATION=true
export USE_FAKE_HARDWARE=true

echo "✅ R2 System ready!"
echo "Available commands:"
echo "  r2_gazebo  - Start Gazebo simulation"
echo "  r2_moveit  - Start MoveIt planning"
echo "  r2_rviz    - Start RViz visualization"
echo "  r2_build   - Build workspace"

# Start a new shell with all configurations loaded
exec bash
EOF
    
    chmod +x ~/start_r2_system.sh
    
    # Create update script
    cat > ~/update_r2_system.sh << 'EOF'
#!/bin/bash
# R2 System Update Script

echo "🔄 Updating R2 System..."

cd ~/ros2_workspace

# Pull latest changes
git pull origin main

# Rebuild workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Update Python packages
if [[ -f "requirements.txt" ]]; then
    pip3 install -r requirements.txt --upgrade
fi

echo "✅ R2 System updated!"
EOF
    
    chmod +x ~/update_r2_system.sh
    
    log_success "Convenience scripts created"
}

# Final system information
show_final_info() {
    log_header "Installation Complete!"
    
    echo -e "${GREEN}🎉 R2 RoArm M3 System successfully installed on Google VM!${NC}\n"
    
    echo -e "${BLUE}📍 Installation Summary:${NC}"
    echo -e "   • ROS2 Humble: ✅ Installed"
    echo -e "   • MoveIt2: ✅ Installed"
    echo -e "   • Python packages: ✅ Installed"
    echo -e "   • R2 workspace: ✅ Built and ready"
    echo -e "   • Environment: ✅ Configured for VM simulation"
    
    echo -e "\n${BLUE}🚀 Quick Start Commands:${NC}"
    echo -e "   • Start system: ${YELLOW}./start_r2_system.sh${NC}"
    echo -e "   • Update system: ${YELLOW}./update_r2_system.sh${NC}"
    echo -e "   • Build workspace: ${YELLOW}r2_build${NC}"
    echo -e "   • Start Gazebo: ${YELLOW}r2_gazebo${NC}"
    echo -e "   • Start MoveIt: ${YELLOW}r2_moveit${NC}"
    
    echo -e "\n${BLUE}📁 Important Paths:${NC}"
    echo -e "   • Workspace: ${YELLOW}~/ros2_workspace${NC}"
    echo -e "   • Setup guide: ${YELLOW}~/ros2_workspace/GOOGLE_VM_SETUP_GUIDE.md${NC}"
    
    echo -e "\n${YELLOW}⚠️  Next Steps:${NC}"
    echo -e "   1. Restart your terminal or run: ${YELLOW}source ~/.bashrc${NC}"
    echo -e "   2. Test the installation: ${YELLOW}./start_r2_system.sh${NC}"
    echo -e "   3. Read the setup guide for detailed usage instructions"
    
    echo -e "\n${PURPLE}🔗 Resources:${NC}"
    echo -e "   • GitHub: https://github.com/guycap746/R2"
    echo -e "   • ROS2 Documentation: https://docs.ros.org/en/humble/"
    echo -e "   • MoveIt2 Documentation: https://moveit.picknik.ai/humble/"
    
    echo -e "\n${GREEN}Happy coding with your R2 system! 🤖${NC}"
}

# Main installation function
main() {
    log_header "R2 RoArm M3 System Setup for Google Cloud VM"
    log_info "This script will install and configure the complete R2 system"
    log_info "Estimated time: 15-30 minutes depending on VM performance"
    log_info "Installation log will be saved to: $LOG_FILE"
    
    # Confirm installation
    read -p "Do you want to proceed with the installation? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Installation cancelled by user"
        exit 0
    fi
    
    # Run installation steps
    check_user
    check_system
    update_system
    install_ros2
    install_python_deps
    setup_workspace
    build_workspace
    configure_environment
    install_additional_tools
    run_tests
    create_shortcuts
    show_final_info
    
    log_success "Installation completed successfully!"
    log_info "Full installation log saved to: $LOG_FILE"
}

# Create log file for debugging
LOG_FILE="/tmp/r2_setup_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$LOG_FILE" >&2)

# Trap errors and cleanup
trap 'log_error "Installation failed at line $LINENO. Check the output above for details."; log_error "Full log saved to: $LOG_FILE"' ERR

# Run main installation
main "$@"