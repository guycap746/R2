# Docker Removal Risk Analysis & Migration Plan

## 🚨 **CRITICAL RISKS IDENTIFIED**

### **HIGH RISK - System Architecture**
- **No native ROS2 installation**: System designed to run ROS2 exclusively in Docker
- **Missing core dependencies**: 50+ packages need native installation
- **Hardcoded paths**: 20+ files contain `/ros2_ws` Docker-specific paths
- **Environment dependencies**: Complete ROS2 ecosystem not configured natively

### **HIGH RISK - Hardware Integration**
- **USB device access**: Docker provides privileged access to robot/camera hardware
- **Driver compatibility**: RealSense SDK installed only in container
- **Permission management**: User groups and device permissions need manual setup
- **Real-time performance**: Docker isolation may be beneficial for hardware timing

### **MEDIUM RISK - Development Workflow**
- **Build environment**: Complete C++/Python development stack only in Docker
- **Package management**: AnyGrasp and specialized dependencies require complex setup
- **Testing isolation**: Docker provides clean, reproducible test environment
- **Multi-user support**: Native installation affects entire system

### **MEDIUM RISK - Network & Communication**
- **ROS2 DDS**: Docker uses host networking for ROS node communication
- **Foxglove Bridge**: Web-based visualization configured for container environment
- **Port management**: Service ports may conflict with host services
- **Firewall configuration**: Native setup requires network policy changes

## 📋 **MIGRATION PLAN**

### **Phase 1: Pre-Migration Analysis & Backup**

#### 1.1 Complete System Backup
```bash
# Create full workspace backup
tar -czf roarm_workspace_backup_$(date +%Y%m%d).tar.gz /root/ros2_workspace/

# Docker image backup
docker save $(docker images -q) | gzip > roarm_docker_images_$(date +%Y%m%d).tar.gz

# System configuration backup
cp -r /etc/ros2/ /backup/ 2>/dev/null || echo "No ROS2 config found"
dpkg --get-selections > installed_packages_$(date +%Y%m%d).txt
```

#### 1.2 Dependency Analysis
```bash
# List all Docker dependencies
docker image inspect osrf/ros:humble-desktop-full
docker history osrf/ros:humble-desktop-full

# Check current system packages
apt list --installed | grep -E "(ros|realsense|opencv|python3)"
```

#### 1.3 GitHub Repository Setup
```bash
# Initialize Git repository
cd /root/ros2_workspace
git init
git add .
git commit -m "Initial commit - Docker-based RoArm M3 system"

# Create .gitignore for build artifacts
echo "build/\ninstall/\nlog/\n*.pyc\n__pycache__/\n.DS_Store" > .gitignore
```

### **Phase 2: Native Environment Preparation**

#### 2.1 ROS2 Humble Native Installation
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop Full
sudo apt update
sudo apt install ros-humble-desktop-full

# Install additional ROS2 packages
sudo apt install ros-humble-moveit ros-humble-moveit-planners ros-humble-moveit-plugins
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
sudo apt install ros-humble-foxglove-bridge ros-humble-joint-state-publisher
```

#### 2.2 Hardware Drivers & Dependencies
```bash
# Intel RealSense SDK
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Python dependencies
pip3 install opencv-python numpy scipy transforms3d roboflow
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Development tools
sudo apt install build-essential cmake git python3-colcon-common-extensions
```

#### 2.3 User Permissions Setup
```bash
# Add user to required groups
sudo usermod -a -G dialout,plugdev,video $USER

# USB device rules for robot
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", MODE="0666"' | sudo tee /etc/udev/rules.d/99-roarm.rules

# RealSense camera rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-realsense.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```

### **Phase 3: Code Migration & Path Updates**

#### 3.1 Workspace Path Migration
```bash
# Create new native workspace
mkdir -p /home/$USER/roarm_ws/src
cd /home/$USER/roarm_ws

# Copy source code (excluding Docker files)
cp -r /root/ros2_workspace/src/* ./src/
```

#### 3.2 Critical File Updates
**Files requiring immediate path updates:**

1. **`anygrasp_interactive_node.py`**:
```python
# OLD: ANYGRASP_SDK_PATH = "/ros2_ws/src/anygrasp_workspace/anygrasp_sdk"
# NEW: ANYGRASP_SDK_PATH = os.path.expanduser("~/roarm_ws/src/anygrasp_workspace/anygrasp_sdk")
```

2. **`foxglove_roarm.launch.py`**:
```python
# OLD: urdf_file = "/ros2_ws/src/roarm_main/roarm_description/urdf/roarm_description.urdf"
# NEW: Use FindPackageShare for dynamic path resolution
```

3. **Setup scripts**: Update all workspace references from `/ros2_ws` to `~/roarm_ws`

#### 3.3 Environment Configuration
```bash
# Create setup script for native environment
cat > ~/roarm_ws/setup_native_env.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export PYTHONPATH="$HOME/roarm_ws/install/lib/python3.10/site-packages:$PYTHONPATH"
export ROARM_WS="$HOME/roarm_ws"
source $HOME/roarm_ws/install/setup.bash
EOF

chmod +x ~/roarm_ws/setup_native_env.sh
```

### **Phase 4: GitHub Integration Strategy**

#### 4.1 Repository Structure
```
roarm-m3-grasping/
├── README.md
├── INSTALL_NATIVE.md
├── DOCKER_LEGACY.md
├── .gitignore
├── src/                    # ROS2 packages
├── config/                 # Configuration files
├── docs/                   # Documentation
├── scripts/               # Installation and setup scripts
└── calibration/           # Calibration data and procedures
```

#### 4.2 Version Control Strategy
```bash
# Create feature branches for migration
git checkout -b native-migration
git checkout -b docker-legacy

# Use Git LFS for large files
git lfs track "*.bag"
git lfs track "*.png"
git lfs track "*.jpg"
git lfs track "calibration/*.json"
```

#### 4.3 CI/CD Pipeline Setup
```yaml
# .github/workflows/test-native.yml
name: Native Installation Test
on: [push, pull_request]
jobs:
  test-ubuntu-22:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Install ROS2 Humble
        run: ./scripts/install_ros2_humble.sh
      - name: Build workspace
        run: ./scripts/build_workspace.sh
      - name: Run tests
        run: ./scripts/run_tests.sh
```

## ⚠️ **SPECIFIC RISKS & MITIGATION**

### **Risk 1: AnyGrasp SDK Integration**
- **Risk**: Complex AnyGrasp dependencies may not install cleanly
- **Mitigation**: Create conda environment for AnyGrasp isolation
- **Fallback**: Keep Docker container only for AnyGrasp processing

### **Risk 2: Hardware Driver Conflicts**
- **Risk**: Native drivers may conflict with existing system
- **Mitigation**: Test on isolated VM first, backup current drivers
- **Fallback**: Use Docker for hardware access only

### **Risk 3: Performance Degradation**
- **Risk**: Native installation may have different performance characteristics
- **Mitigation**: Benchmark key operations before/after migration
- **Fallback**: Hybrid approach with performance-critical parts in containers

### **Risk 4: Multi-User Development**
- **Risk**: Native installation affects entire system
- **Mitigation**: Use ROS2 workspaces and virtual environments
- **Fallback**: Personal Docker containers for development

### **Risk 5: System Contamination**
- **Risk**: Native packages may interfere with other projects
- **Mitigation**: Use package managers carefully, document changes
- **Fallback**: Virtual machines for project isolation

## 🔄 **ROLLBACK PLAN**

### Immediate Rollback (if migration fails):
```bash
# Stop all ROS2 services
sudo systemctl stop ros2-* 2>/dev/null

# Restore Docker environment
docker load < roarm_docker_images_YYYYMMDD.tar.gz
cd /root/ros2_workspace
docker-compose up -d

# Restore workspace
tar -xzf roarm_workspace_backup_YYYYMMDD.tar.gz
```

### Partial Rollback (hybrid approach):
- Keep Docker for AnyGrasp and complex dependencies
- Use native ROS2 for robot control and basic operations
- Bridge communication between Docker and native components

## 📊 **RECOMMENDED APPROACH**

### **Option 1: Full Native Migration** ⭐ 
- **Pros**: Complete control, better performance, simpler deployment
- **Cons**: High risk, significant effort, potential compatibility issues
- **Timeline**: 2-3 weeks with extensive testing

### **Option 2: Hybrid Approach** ⭐⭐⭐
- **Pros**: Lower risk, gradual migration, keeps working components
- **Cons**: More complex architecture, requires container management
- **Timeline**: 1 week initial setup, gradual component migration

### **Option 3: Docker Optimization** ⭐⭐
- **Pros**: Minimal risk, leverages existing work, portable
- **Cons**: Continues Docker dependency, may limit deployment options
- **Timeline**: Few days to optimize and document

## 🎯 **NEXT STEPS**

1. **Create complete backup** of current system
2. **Set up GitHub repository** with proper structure
3. **Test native ROS2 installation** on separate VM
4. **Start with hybrid approach**: native ROS2 + Docker for AnyGrasp
5. **Gradually migrate components** based on success and requirements
6. **Document everything** for future maintenance and team members

**Recommendation**: Start with **Option 2 (Hybrid)** to minimize risk while moving toward native deployment.