#!/bin/bash
"""
LeRobot Installation Script for ROS2 Workspace

This script installs LeRobot and all required dependencies for integration
with the RoArm M3 robotic manipulation system.
"""

set -e

echo "🤗 Installing LeRobot for ROS2 Integration"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the ROS2 workspace
if [ ! -f "src/roarm_lerobot_integration/package.xml" ]; then
    print_error "Please run this script from the ROS2 workspace root directory"
    exit 1
fi

print_status "Checking system requirements..."

# Check Python version
python_version=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
required_version="3.10"

if [ "$(printf '%s\n' "$required_version" "$python_version" | sort -V | head -n1)" != "$required_version" ]; then
    print_error "Python 3.10+ required. Found: $python_version"
    exit 1
fi
print_success "Python version check passed: $python_version"

# Check if CUDA is available
if command -v nvidia-smi &> /dev/null; then
    print_success "NVIDIA GPU detected"
    GPU_AVAILABLE=true
else
    print_warning "No NVIDIA GPU detected. Will install CPU-only PyTorch"
    GPU_AVAILABLE=false
fi

# Create virtual environment if it doesn't exist
VENV_DIR="$HOME/.lerobot_venv"
if [ ! -d "$VENV_DIR" ]; then
    print_status "Creating Python virtual environment at $VENV_DIR"
    python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
print_status "Activating virtual environment"
source "$VENV_DIR/bin/activate"

# Upgrade pip
print_status "Upgrading pip"
pip install --upgrade pip

# Install system dependencies
print_status "Installing system dependencies"
sudo apt-get update
sudo apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libglib2.0-0 \
    libgtk-3-0 \
    libgl1-mesa-glx \
    libegl1-mesa \
    libxrandr2 \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6 \
    git \
    git-lfs

# Install PyTorch with appropriate CUDA support
print_status "Installing PyTorch"
if [ "$GPU_AVAILABLE" = true ]; then
    # Install CUDA version
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
else
    # Install CPU version
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
fi

# Install core ML dependencies
print_status "Installing core ML dependencies"
pip install \
    transformers>=4.30.0 \
    datasets>=2.10.0 \
    huggingface_hub>=0.15.0 \
    accelerate>=0.20.0 \
    diffusers>=0.18.0 \
    safetensors>=0.3.0

# Install computer vision dependencies
print_status "Installing computer vision dependencies"
pip install \
    opencv-python>=4.8.0 \
    pillow>=8.3.0 \
    imageio>=2.20.0 \
    imageio-ffmpeg>=0.4.7 \
    ffmpeg-python>=0.2.0

# Install scientific computing dependencies
print_status "Installing scientific computing dependencies"
pip install \
    numpy>=1.21.0 \
    scipy>=1.9.0 \
    scikit-learn>=1.1.0 \
    matplotlib>=3.5.0 \
    seaborn>=0.11.0 \
    pandas>=1.4.0

# Install training and logging dependencies
print_status "Installing training and logging dependencies"
pip install \
    wandb>=0.15.0 \
    tensorboard>=2.12.0 \
    tqdm>=4.64.0 \
    hydra-core>=1.3.0 \
    omegaconf>=2.3.0

# Install LeRobot
print_status "Installing LeRobot framework"
if [ ! -d "/tmp/lerobot" ]; then
    print_status "Cloning LeRobot repository"
    git clone https://github.com/huggingface/lerobot.git /tmp/lerobot
fi

cd /tmp/lerobot

# Install LeRobot with all dependencies
print_status "Installing LeRobot with dependencies"
pip install -e .[all]

# Install additional robotics dependencies
print_status "Installing additional robotics dependencies"
pip install \
    gymnasium>=0.28.0 \
    pybullet>=3.2.5 \
    mujoco>=2.3.0 \
    dm-control>=1.0.10 \
    gym>=0.26.0

# Install ROS2 Python dependencies
print_status "Installing ROS2 Python dependencies"
pip install \
    rclpy \
    cv_bridge \
    tf2_ros \
    geometry_msgs \
    sensor_msgs \
    std_msgs \
    trajectory_msgs \
    control_msgs \
    visualization_msgs

# Return to workspace directory
cd - > /dev/null

# Create LeRobot data and model directories
print_status "Creating LeRobot directories"
mkdir -p /tmp/lerobot_data
mkdir -p /tmp/lerobot_models
mkdir -p /tmp/lerobot_datasets
mkdir -p /tmp/lerobot_videos

# Set permissions
chmod -R 755 /tmp/lerobot_*

# Create activation script
print_status "Creating activation script"
cat > activate_lerobot.sh << 'EOF'
#!/bin/bash
# LeRobot Environment Activation Script

echo "🤗 Activating LeRobot Environment"
source ~/.lerobot_venv/bin/activate

# Set environment variables
export LEROBOT_DATA_DIR="/tmp/lerobot_data"
export LEROBOT_MODEL_DIR="/tmp/lerobot_models"
export LEROBOT_CACHE_DIR="/tmp/lerobot_cache"

# Set CUDA environment if available
if command -v nvidia-smi &> /dev/null; then
    export CUDA_VISIBLE_DEVICES=0
fi

# Set Hugging Face cache
export HF_HOME="/tmp/huggingface_cache"
mkdir -p "$HF_HOME"

echo "✅ LeRobot environment activated"
echo "   Data directory: $LEROBOT_DATA_DIR"
echo "   Model directory: $LEROBOT_MODEL_DIR"
echo "   Python: $(which python)"
echo "   PyTorch: $(python -c 'import torch; print(torch.__version__)')"
echo "   CUDA available: $(python -c 'import torch; print(torch.cuda.is_available())')"

# Source ROS2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "   ROS2: Humble sourced"
fi

# Source workspace setup if available
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "   Workspace: Local setup sourced"
fi
EOF

chmod +x activate_lerobot.sh

# Create configuration files
print_status "Creating configuration files"
mkdir -p src/roarm_lerobot_integration/config

cat > src/roarm_lerobot_integration/config/lerobot_config.yaml << 'EOF'
# LeRobot Configuration for RoArm M3

# Robot Configuration
robot:
  type: "roarm_m3"
  dof: 6
  joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
  joint_limits:
    position: [[-3.14159, 3.14159], [-3.14159, 3.14159], [-3.14159, 3.14159], 
               [-3.14159, 3.14159], [-3.14159, 3.14159], [-3.14159, 3.14159]]
    velocity: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    effort: [50.0, 50.0, 50.0, 20.0, 20.0, 20.0]

# Camera Configuration
cameras:
  rgb:
    topic: "/camera/color/image_raw"
    width: 640
    height: 480
    fps: 30
  depth:
    topic: "/camera/depth/image_raw"
    width: 640
    height: 480
    fps: 30

# Data Collection
data_collection:
  fps: 30
  max_episode_steps: 1000
  max_episodes: 100
  auto_save_interval: 10
  compression: true

# Training Configuration
training:
  policy_type: "act"
  batch_size: 32
  learning_rate: 0.0001
  num_epochs: 100
  validation_split: 0.2
  device: "auto"
  num_workers: 4

# Hugging Face Configuration
huggingface:
  dataset_repo: "roarm/manipulation_dataset"
  policy_repo: "roarm/manipulation_policy"
  use_auth_token: true

# Safety Configuration
safety:
  max_joint_velocity: 0.5
  workspace_limits: [-1.0, 1.0, -1.0, 1.0, 0.0, 1.0]
  collision_threshold: 0.1
  emergency_stop: true
EOF

# Create requirements file
print_status "Creating requirements file"
cat > src/roarm_lerobot_integration/requirements.txt << 'EOF'
# LeRobot Core Dependencies
torch>=2.0.0
torchvision>=0.15.0
torchaudio>=2.0.0
transformers>=4.30.0
datasets>=2.10.0
huggingface_hub>=0.15.0
accelerate>=0.20.0
diffusers>=0.18.0
safetensors>=0.3.0

# Computer Vision
opencv-python>=4.8.0
pillow>=8.3.0
imageio>=2.20.0
imageio-ffmpeg>=0.4.7
ffmpeg-python>=0.2.0

# Scientific Computing
numpy>=1.21.0
scipy>=1.9.0
scikit-learn>=1.1.0
matplotlib>=3.5.0
seaborn>=0.11.0
pandas>=1.4.0

# Training and Logging
wandb>=0.15.0
tensorboard>=2.12.0
tqdm>=4.64.0
hydra-core>=1.3.0
omegaconf>=2.3.0

# Robotics
gymnasium>=0.28.0
pybullet>=3.2.5
mujoco>=2.3.0

# LeRobot (installed from source)
# lerobot @ git+https://github.com/huggingface/lerobot.git
EOF

# Test installation
print_status "Testing LeRobot installation"
source "$VENV_DIR/bin/activate"

# Test PyTorch
if python -c "import torch; print(f'PyTorch {torch.__version__} installed successfully')" 2>/dev/null; then
    print_success "PyTorch installation verified"
else
    print_error "PyTorch installation failed"
    exit 1
fi

# Test Transformers
if python -c "import transformers; print(f'Transformers {transformers.__version__} installed successfully')" 2>/dev/null; then
    print_success "Transformers installation verified"
else
    print_error "Transformers installation failed"
    exit 1
fi

# Test LeRobot
if python -c "import lerobot; print('LeRobot installed successfully')" 2>/dev/null; then
    print_success "LeRobot installation verified"
else
    print_warning "LeRobot installation needs verification"
fi

# Create desktop shortcut for easy activation
print_status "Creating desktop shortcut"
cat > ~/Desktop/Activate_LeRobot.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Activate LeRobot
Comment=Activate LeRobot environment for ROS2
Exec=gnome-terminal -- bash -c "cd $(pwd) && source activate_lerobot.sh && bash"
Icon=applications-development
Terminal=true
Categories=Development;
EOF

chmod +x ~/Desktop/Activate_LeRobot.desktop

# Print success message and usage instructions
echo ""
echo "🎉 LeRobot Installation Complete!"
echo "=================================="
echo ""
print_success "Installation completed successfully!"
echo ""
echo "📋 Next Steps:"
echo "1. Activate the LeRobot environment:"
echo "   source activate_lerobot.sh"
echo ""
echo "2. Build the ROS2 package:"
echo "   colcon build --packages-select roarm_lerobot_integration"
echo ""
echo "3. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "4. Launch LeRobot integration:"
echo "   ros2 launch roarm_lerobot_integration lerobot_complete.launch.py"
echo ""
echo "📁 Directories created:"
echo "   • Data: /tmp/lerobot_data"
echo "   • Models: /tmp/lerobot_models"
echo "   • Cache: /tmp/huggingface_cache"
echo ""
echo "🔧 Configuration:"
echo "   • Config file: src/roarm_lerobot_integration/config/lerobot_config.yaml"
echo "   • Requirements: src/roarm_lerobot_integration/requirements.txt"
echo ""
echo "💡 Tips:"
echo "   • Use 'Double-click Desktop/Activate_LeRobot.desktop' for quick access"
echo "   • Set HF_TOKEN environment variable for Hugging Face features"
echo "   • Check GPU availability with: python -c 'import torch; print(torch.cuda.is_available())'"
echo ""

if [ "$GPU_AVAILABLE" = true ]; then
    print_success "GPU acceleration enabled for training"
else
    print_warning "CPU-only mode - training will be slower"
fi

echo ""
echo "🚀 Ready to start LeRobot integration with RoArm M3!"