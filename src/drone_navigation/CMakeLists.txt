cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/NavigationStatus.msg"
  "msg/ObstacleAlert.msg"
  "srv/SetPosition.srv"
  "srv/SetVelocity.srv"
  "srv/SetWaypoints.srv"
  "srv/GetWaypoints.srv"
  "action/NavigateToGoal.action"
  "action/FollowPath.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

# Install Python scripts
install(PROGRAMS
  drone_navigation/navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()