cmake_minimum_required(VERSION 3.5)
project(eventchains)

# Set include directories
include_directories(include)
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables and list their dependencies
add_executable(executor_1 src/executor_1.cpp)

ament_target_dependencies(executor_1 ament_cmake rclcpp std_msgs)

install(TARGETS
  executor_1
  DESTINATION lib/${PROJECT_NAME})
ament_package()
