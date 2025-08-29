cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetWaypoint.srv"
  "srv/GetWaypoints.srv"
  "srv/ClearWaypoints.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python modules
install(PROGRAMS
  scripts/navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()