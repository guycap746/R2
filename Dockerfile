FROM osrf/ros:humble-desktop-full

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    vim \
    nano \
    wget \
    curl \
    build-essential \
    cmake \
    gdb \
    valgrind \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt2 and Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-configs-utils \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install additional robotics and simulation tools
RUN apt-get update && apt-get install -y \
    ros-humble-geometric-shapes \
    ros-humble-srdfdom \
    ros-humble-ompl \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo \
    ros-humble-pilz-industrial-motion-planner \
    ros-humble-chomp-motion-planner \
    ros-humble-perception-pcl \
    ros-humble-pcl-conversions \
    ros-humble-compressed-image-transport \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK and ROS2 packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Add Intel RealSense repository
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | gpg --dearmor > /etc/apt/keyrings/librealsense.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list

# Install RealSense SDK and ROS2 packages
RUN apt-get update && apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-depth-image-proc \
    ros-humble-image-view \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch first (required for MinkowskiEngine)
RUN pip3 install \
    torch \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cpu

# Install Python packages commonly used in ROS2 and robotics
RUN pip3 install \
    setuptools==58.2.0 \
    pytest \
    flake8 \
    mypy \
    black \
    numpy \
    scipy \
    matplotlib \
    transforms3d \
    pyyaml \
    pyserial \
    aiohttp \
    ujson \
    aiortc \
    av \
    opencv-python \
    open3d \
    trimesh \
    pytorch-lightning \
    tensorboard \
    roboflow \
    supervision \
    pillow \
    requests

# Install MinkowskiEngine after PyTorch (for AnyGrasp point cloud processing)
RUN pip3 install MinkowskiEngine --no-deps

# Install additional dependencies for AnyGrasp and Intel ROS2 Grasp Library
RUN apt-get update && apt-get install -y \
    python3-opencv \
    python3-open3d \
    ros-humble-perception-pcl \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-vision-msgs \
    ros-humble-camera-info-manager \
    ros-humble-image-geometry \
    ros-humble-tf2-geometry-msgs \
    ros-humble-geometric-shapes \
    ros-humble-grasp-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Intel OpenVINO for Intel ROS2 Grasp Library (alternative to AnyGrasp)
RUN wget -O - https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | apt-key add - && \
    echo "deb https://apt.repos.intel.com/openvino/2022 jammy main" | tee /etc/apt/sources.list.d/intel-openvino-2022.list && \
    apt-get update && \
    apt-get install -y intel-openvino-runtime-ubuntu22-2022.3.0 && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Clone and build Intel ROS2 Grasp Library (alternative to AnyGrasp)
RUN mkdir -p src && cd src && \
    git clone https://github.com/intel/ros2_grasp_library.git && \
    cd .. && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-select ros2_grasp_library"

# Initialize rosdep
RUN rosdep update

# Source ROS2 setup in bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Create convenient aliases
RUN echo "alias cb='colcon build'" >> ~/.bashrc
RUN echo "alias cbt='colcon build --cmake-target'" >> ~/.bashrc
RUN echo "alias cbp='colcon build --packages-select'" >> ~/.bashrc
RUN echo "alias cbs='colcon build --symlink-install'" >> ~/.bashrc
RUN echo "alias ct='colcon test'" >> ~/.bashrc
RUN echo "alias ctp='colcon test --packages-select'" >> ~/.bashrc
RUN echo "alias ctr='colcon test-result'" >> ~/.bashrc
RUN echo "alias rviz='ros2 run rviz2 rviz2'" >> ~/.bashrc
RUN echo "alias gazebo='ros2 launch gazebo_ros gazebo.launch.py'" >> ~/.bashrc
RUN echo "alias moveit='ros2 launch moveit_setup_assistant setup_assistant.launch.py'" >> ~/.bashrc

# Set environment variables
ENV ROS_DISTRO=humble
ENV PYTHONPATH="/ros2_ws/install/lib/python3.10/site-packages:${PYTHONPATH}"

CMD ["/bin/bash"]