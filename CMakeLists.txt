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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service message
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs
)

# Get the include directories
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create executable for pre_approach node
add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Create executable for pre_approach_v2 node
add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(pre_approach_v2 "${cpp_typesupport_target}")

# Create executable for approach_service_server node
add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  nav_msgs
)
target_link_libraries(approach_service_server "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  pre_approach
  pre_approach_v2
  approach_service_server
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