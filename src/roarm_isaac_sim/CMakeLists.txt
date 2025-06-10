cmake_minimum_required(VERSION 3.8)
project(roarm_isaac_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/isaac_sim_launcher.py
  scripts/isaac_ros_bridge.py
  scripts/synthetic_data_generator.py
  scripts/sim_to_real_calibration.py
  scripts/grasp_simulation_environment.py
  scripts/advanced_object_placer.py
  scripts/gazebo_object_interface.py
  scripts/hierarchical_scene_builder.py
  scripts/virtual_object_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Isaac Sim scene files
install(DIRECTORY scenes
  DESTINATION share/${PROJECT_NAME}/
)

# Install robot models for Isaac Sim
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install simulation environments
install(DIRECTORY environments
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()