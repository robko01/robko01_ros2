cmake_minimum_required(VERSION 3.8)
project(robko01_ros2 LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)  # Dependency for trajectory_msgs
# find_package(ros2_control_cmake REQUIRED)
# set_compiler_options()
# export_windows_symbols()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# Generate action files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Move.action"
  DEPENDENCIES trajectory_msgs  # Required for using trajectory_msgs in the action definition
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  # src/move_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DIRECTORY urdf
  DIRECTORY DESTINATION share/${PROJECT_NAME}
)

# Register package
ament_package()