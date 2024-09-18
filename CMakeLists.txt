cmake_minimum_required(VERSION 3.8)
project(robko01_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)  # Dependency for trajectory_msgs

# Generate action files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
  DEPENDENCIES trajectory_msgs  # Required for using trajectory_msgs in the action definition
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  src/move_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DIRECOTRY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Register package
ament_package()