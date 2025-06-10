# ROS2 Docker Development Environment

This setup provides a portable ROS2 Humble development environment using Docker.

## Quick Start

1. **Build and start the development container:**
   ```bash
   docker-compose up --build -d ros2-dev
   ```

2. **Enter the development container:**
   ```bash
   docker-compose exec ros2-dev bash
   ```

3. **Create your first package:**
   ```bash
   cd src
   ros2 pkg create --build-type ament_python my_package
   ```

4. **Build your workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

## Directory Structure

- `src/` - Your ROS2 packages source code
- `install/` - Built packages (auto-generated)
- `build/` - Build artifacts (auto-generated) 
- `log/` - Build logs (auto-generated)

## Available Commands

### Container Management
- `docker-compose up -d ros2-dev` - Start development container
- `docker-compose exec ros2-dev bash` - Enter development container
- `docker-compose down` - Stop all containers

### Build Aliases (inside container)
- `cb` - colcon build
- `cbs` - colcon build --symlink-install
- `cbp <package>` - colcon build --packages-select
- `ct` - colcon test
- `ctp <package>` - colcon test --packages-select

## Features

- ROS2 Humble Desktop Full
- Python development tools (pytest, flake8, mypy, black)
- Debugging tools (gdb, valgrind)
- GUI support (X11 forwarding)
- Persistent workspace volumes
- Network host mode for ROS2 communication

## Portability

All ROS2 code and configurations are stored in the `src/` directory, making it easy to:
- Version control your entire workspace
- Share projects between team members  
- Move between different development machines
- Deploy to different environments