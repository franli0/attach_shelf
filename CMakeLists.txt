cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create executable for pre_approach node
add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  pre_approach
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz files
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()