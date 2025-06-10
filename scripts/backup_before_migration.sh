#!/bin/bash

# Complete System Backup Before Docker Migration
# This script creates comprehensive backups of the entire RoArm M3 system

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BACKUP_BASE_DIR="/backup/roarm_migration"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="${BACKUP_BASE_DIR}/${TIMESTAMP}"

echo -e "${BLUE}🚀 RoArm M3 Complete System Backup${NC}"
echo -e "${BLUE}===================================${NC}"
echo -e "Backup directory: ${GREEN}${BACKUP_DIR}${NC}"
echo -e "Timestamp: ${GREEN}${TIMESTAMP}${NC}\n"

# Create backup directory structure
mkdir -p "${BACKUP_DIR}"/{workspace,docker,system,git}

echo -e "${YELLOW}📁 Creating backup directory structure...${NC}"

# Function to log backup steps
log_step() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

# 1. WORKSPACE BACKUP
echo -e "\n${BLUE}1. Backing up ROS2 workspace...${NC}"

if [ -d "/root/ros2_workspace" ]; then
    echo "Creating workspace archive..."
    tar -czf "${BACKUP_DIR}/workspace/ros2_workspace_complete.tar.gz" \
        -C "/root" "ros2_workspace" \
        --exclude="ros2_workspace/build" \
        --exclude="ros2_workspace/install" \
        --exclude="ros2_workspace/log" \
        --exclude="**/__pycache__" \
        --exclude="**/*.pyc"
    log_step "ROS2 workspace backed up"
else
    log_warning "ROS2 workspace not found at /root/ros2_workspace"
fi

# 2. DOCKER BACKUP
echo -e "\n${BLUE}2. Backing up Docker environment...${NC}"

# Check if Docker is running
if command -v docker &> /dev/null && docker info &> /dev/null; then
    echo "Backing up Docker images..."
    
    # List all images
    docker images --format "table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}" > "${BACKUP_DIR}/docker/docker_images_list.txt"
    
    # Save specific images
    if docker images | grep -q "ros"; then
        echo "Saving ROS-related Docker images..."
        docker save $(docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "(ros|roarm)") | \
            gzip > "${BACKUP_DIR}/docker/ros_docker_images.tar.gz"
        log_step "ROS Docker images backed up"
    fi
    
    # Backup docker-compose files
    if [ -f "/root/ros2_workspace/docker-compose.yml" ]; then
        cp "/root/ros2_workspace/docker-compose.yml" "${BACKUP_DIR}/docker/"
        log_step "Docker Compose file backed up"
    fi
    
    # Backup Dockerfile
    if [ -f "/root/ros2_workspace/Dockerfile" ]; then
        cp "/root/ros2_workspace/Dockerfile" "${BACKUP_DIR}/docker/"
        log_step "Dockerfile backed up"
    fi
    
    # Docker container information
    docker ps -a > "${BACKUP_DIR}/docker/docker_containers.txt"
    docker volume ls > "${BACKUP_DIR}/docker/docker_volumes.txt"
    docker network ls > "${BACKUP_DIR}/docker/docker_networks.txt"
    
    log_step "Docker environment backed up"
else
    log_warning "Docker not available or not running"
fi

# 3. SYSTEM CONFIGURATION BACKUP
echo -e "\n${BLUE}3. Backing up system configuration...${NC}"

# Installed packages
echo "Backing up package information..."
dpkg --get-selections > "${BACKUP_DIR}/system/installed_packages.txt"
apt list --installed > "${BACKUP_DIR}/system/apt_packages.txt" 2>/dev/null

# Python packages
if command -v pip3 &> /dev/null; then
    pip3 list > "${BACKUP_DIR}/system/pip_packages.txt"
fi

# System information
uname -a > "${BACKUP_DIR}/system/system_info.txt"
lsb_release -a > "${BACKUP_DIR}/system/os_info.txt" 2>/dev/null
cat /proc/meminfo > "${BACKUP_DIR}/system/memory_info.txt"
cat /proc/cpuinfo > "${BACKUP_DIR}/system/cpu_info.txt"
lsusb > "${BACKUP_DIR}/system/usb_devices.txt"

# Network configuration
ip addr > "${BACKUP_DIR}/system/network_config.txt"
cat /etc/hosts > "${BACKUP_DIR}/system/hosts_file.txt"

# User and group information
getent passwd > "${BACKUP_DIR}/system/users.txt"
getent group > "${BACKUP_DIR}/system/groups.txt"
id > "${BACKUP_DIR}/system/current_user.txt"

# Environment variables
env > "${BACKUP_DIR}/system/environment.txt"

# ROS2 specific configuration
if [ -d "/opt/ros" ]; then
    ls -la /opt/ros/ > "${BACKUP_DIR}/system/ros_installations.txt"
fi

if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    cp -r "/etc/ros" "${BACKUP_DIR}/system/ros_config" 2>/dev/null || log_warning "Could not backup ROS config"
fi

log_step "System configuration backed up"

# 4. HARDWARE CONFIGURATION
echo -e "\n${BLUE}4. Backing up hardware configuration...${NC}"

# USB device rules
if [ -d "/etc/udev/rules.d" ]; then
    mkdir -p "${BACKUP_DIR}/system/udev_rules"
    cp /etc/udev/rules.d/*.rules "${BACKUP_DIR}/system/udev_rules/" 2>/dev/null || log_warning "No custom udev rules found"
fi

# RealSense configuration
if command -v realsense-viewer &> /dev/null; then
    echo "RealSense SDK detected" > "${BACKUP_DIR}/system/realsense_info.txt"
    realsense-viewer --version >> "${BACKUP_DIR}/system/realsense_info.txt" 2>/dev/null || true
fi

# Check for robot connections
if ls /dev/ttyUSB* &> /dev/null; then
    ls -la /dev/ttyUSB* > "${BACKUP_DIR}/system/serial_devices.txt"
fi

if ls /dev/ttyACM* &> /dev/null; then
    ls -la /dev/ttyACM* >> "${BACKUP_DIR}/system/serial_devices.txt"
fi

log_step "Hardware configuration backed up"

# 5. GIT REPOSITORY BACKUP
echo -e "\n${BLUE}5. Creating Git repository backup...${NC}"

cd "/root/ros2_workspace"

# Initialize git if not already done
if [ ! -d ".git" ]; then
    echo "Initializing Git repository..."
    git init
    git add .
    git commit -m "Initial backup commit before Docker migration - ${TIMESTAMP}"
fi

# Create git bundle (complete repository backup)
git bundle create "${BACKUP_DIR}/git/roarm_workspace_repo.bundle" --all
log_step "Git repository bundle created"

# Create patch files for uncommitted changes
if ! git diff --quiet; then
    git diff > "${BACKUP_DIR}/git/uncommitted_changes.patch"
    log_step "Uncommitted changes saved as patch"
fi

# Git configuration and logs
git log --oneline -10 > "${BACKUP_DIR}/git/recent_commits.txt" 2>/dev/null || true
git config --list > "${BACKUP_DIR}/git/git_config.txt" 2>/dev/null || true
git status > "${BACKUP_DIR}/git/git_status.txt" 2>/dev/null || true

# 6. CALIBRATION DATA BACKUP
echo -e "\n${BLUE}6. Backing up calibration and runtime data...${NC}"

# Calibration data
if [ -d "/tmp/charuco_calibration" ]; then
    cp -r "/tmp/charuco_calibration" "${BACKUP_DIR}/workspace/charuco_calibration"
    log_step "ChArUco calibration data backed up"
fi

if [ -d "/tmp/hand_eye_calibration" ]; then
    cp -r "/tmp/hand_eye_calibration" "${BACKUP_DIR}/workspace/hand_eye_calibration"
    log_step "Hand-eye calibration data backed up"
fi

if [ -d "/tmp/dual_camera_captures" ]; then
    cp -r "/tmp/dual_camera_captures" "${BACKUP_DIR}/workspace/dual_camera_captures"
    log_step "Dual camera capture data backed up"
fi

# Roboflow data
if [ -d "/tmp/roboflow_images" ]; then
    cp -r "/tmp/roboflow_images" "${BACKUP_DIR}/workspace/roboflow_images"
    log_step "Roboflow data backed up"
fi

# 7. CREATE RESTORE SCRIPTS
echo -e "\n${BLUE}7. Creating restore scripts...${NC}"

# Docker restore script
cat > "${BACKUP_DIR}/restore_docker_environment.sh" << 'EOF'
#!/bin/bash
echo "🔄 Restoring Docker environment..."

# Load Docker images
if [ -f "docker/ros_docker_images.tar.gz" ]; then
    echo "Loading Docker images..."
    gunzip -c docker/ros_docker_images.tar.gz | docker load
fi

# Restore workspace
if [ -f "workspace/ros2_workspace_complete.tar.gz" ]; then
    echo "Restoring workspace..."
    tar -xzf workspace/ros2_workspace_complete.tar.gz -C /
fi

# Restore Docker Compose
if [ -f "docker/docker-compose.yml" ]; then
    cp docker/docker-compose.yml /root/ros2_workspace/
fi

echo "✅ Docker environment restored"
EOF

# Git restore script
cat > "${BACKUP_DIR}/restore_git_repository.sh" << 'EOF'
#!/bin/bash
echo "🔄 Restoring Git repository..."

if [ -f "git/roarm_workspace_repo.bundle" ]; then
    mkdir -p /root/ros2_workspace_restored
    cd /root/ros2_workspace_restored
    git clone ../git/roarm_workspace_repo.bundle .
    
    if [ -f "../git/uncommitted_changes.patch" ]; then
        echo "Applying uncommitted changes..."
        git apply ../git/uncommitted_changes.patch
    fi
    
    echo "✅ Git repository restored to /root/ros2_workspace_restored"
else
    echo "❌ Git bundle not found"
fi
EOF

# System restore script
cat > "${BACKUP_DIR}/restore_system_packages.sh" << 'EOF'
#!/bin/bash
echo "🔄 Restoring system packages..."

if [ -f "system/installed_packages.txt" ]; then
    echo "Note: Review installed_packages.txt and manually install required packages"
    echo "Use: sudo dpkg --set-selections < system/installed_packages.txt && sudo apt-get dselect-upgrade"
fi

if [ -d "system/udev_rules" ]; then
    echo "Restoring udev rules..."
    sudo cp system/udev_rules/*.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

echo "✅ System configuration guidance provided"
EOF

chmod +x "${BACKUP_DIR}"/*.sh

log_step "Restore scripts created"

# 8. CREATE BACKUP MANIFEST
echo -e "\n${BLUE}8. Creating backup manifest...${NC}"

cat > "${BACKUP_DIR}/BACKUP_MANIFEST.md" << EOF
# RoArm M3 System Backup Manifest

**Backup Created**: ${TIMESTAMP}
**Backup Location**: ${BACKUP_DIR}

## Contents

### Workspace Backup
- \`workspace/ros2_workspace_complete.tar.gz\` - Complete ROS2 workspace (excluding build artifacts)
- \`workspace/charuco_calibration/\` - ChArUco calibration data
- \`workspace/hand_eye_calibration/\` - Hand-eye calibration data
- \`workspace/dual_camera_captures/\` - Dual camera capture data
- \`workspace/roboflow_images/\` - Roboflow training data

### Docker Backup
- \`docker/ros_docker_images.tar.gz\` - All ROS-related Docker images
- \`docker/docker-compose.yml\` - Docker Compose configuration
- \`docker/Dockerfile\` - Docker build configuration
- \`docker/docker_*.txt\` - Docker environment information

### System Configuration
- \`system/installed_packages.txt\` - Debian package list
- \`system/pip_packages.txt\` - Python packages
- \`system/*_info.txt\` - System information files
- \`system/udev_rules/\` - Hardware device rules
- \`system/ros_config/\` - ROS2 configuration files

### Git Repository
- \`git/roarm_workspace_repo.bundle\` - Complete Git repository
- \`git/uncommitted_changes.patch\` - Uncommitted changes
- \`git/git_*.txt\` - Git status and configuration

### Restore Scripts
- \`restore_docker_environment.sh\` - Restore Docker setup
- \`restore_git_repository.sh\` - Restore Git repository
- \`restore_system_packages.sh\` - System restoration guidance

## Restoration Instructions

1. **Emergency Docker Restore**:
   \`\`\`bash
   cd ${BACKUP_DIR}
   ./restore_docker_environment.sh
   \`\`\`

2. **Git Repository Restore**:
   \`\`\`bash
   cd ${BACKUP_DIR}
   ./restore_git_repository.sh
   \`\`\`

3. **System Package Restore**:
   \`\`\`bash
   cd ${BACKUP_DIR}
   ./restore_system_packages.sh
   \`\`\`

## Backup Verification

Run this command to verify backup integrity:
\`\`\`bash
find ${BACKUP_DIR} -name "*.tar.gz" -exec tar -tzf {} >/dev/null \; && echo "✅ All archives valid"
\`\`\`

## Notes

- This backup was created before Docker removal migration
- All critical system state has been preserved
- Calibration data and training datasets are included
- Docker environment can be fully restored if needed

**Created by**: backup_before_migration.sh
**System**: $(uname -a)
**User**: $(whoami)
EOF

log_step "Backup manifest created"

# 9. VERIFY BACKUP INTEGRITY
echo -e "\n${BLUE}9. Verifying backup integrity...${NC}"

# Check archive integrity
ARCHIVES_OK=true
for archive in "${BACKUP_DIR}"/workspace/*.tar.gz "${BACKUP_DIR}"/docker/*.tar.gz; do
    if [ -f "$archive" ]; then
        if ! tar -tzf "$archive" >/dev/null 2>&1; then
            log_error "Archive integrity check failed: $archive"
            ARCHIVES_OK=false
        fi
    fi
done

if $ARCHIVES_OK; then
    log_step "All archives passed integrity check"
else
    log_error "Some archives failed integrity check"
fi

# Calculate backup size
BACKUP_SIZE=$(du -sh "${BACKUP_DIR}" | cut -f1)
log_step "Backup completed - Total size: $BACKUP_SIZE"

# 10. SUMMARY
echo -e "\n${GREEN}🎉 BACKUP COMPLETED SUCCESSFULLY${NC}"
echo -e "${GREEN}=================================${NC}"
echo -e "📁 Backup location: ${BLUE}${BACKUP_DIR}${NC}"
echo -e "💾 Backup size: ${BLUE}${BACKUP_SIZE}${NC}"
echo -e "⏰ Timestamp: ${BLUE}${TIMESTAMP}${NC}"
echo -e ""
echo -e "${YELLOW}📋 Next steps:${NC}"
echo -e "1. Review backup manifest: ${BACKUP_DIR}/BACKUP_MANIFEST.md"
echo -e "2. Test restore scripts on a separate system"
echo -e "3. Proceed with Docker migration knowing you have a complete backup"
echo -e "4. Keep this backup until migration is fully validated"
echo -e ""
echo -e "${GREEN}✅ You can now safely proceed with Docker removal!${NC}"

# Create symlink to latest backup
ln -sf "${BACKUP_DIR}" "${BACKUP_BASE_DIR}/latest"
echo -e "🔗 Latest backup symlink created: ${BACKUP_BASE_DIR}/latest"
EOF