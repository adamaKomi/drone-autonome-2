cmake_minimum_required(VERSION 3.8)
project(drone_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Interfaces pour local_navigation_node
  "action/GotoLocalAction.action"
  "srv/UpdateLocal.srv"
  # Messages


  # Messages utilisés dans drone_navigation
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/MissionStatus.msg"
  "msg/NavigationStatus.msg"
  "msg/PathProgress.msg"
  "msg/EmergencyStatus.msg"  # utilisé dans navigation_supervisor_node
  "msg/WaypointReached.msg"

  # Les autres messages sont commentés pour accélérer la compilation
  # "msg/DroneStatus.msg"
  # "msg/DroneState.msg"
  # "msg/FlightMode.msg"
  # "msg/SafetyStatus.msg"
  # "msg/BatteryStatus.msg"
  # "msg/Position3D.msg"
  # "msg/Orientation.msg"
  # "msg/Velocity3D.msg"
  # "msg/NavigationCommand.msg"
  # "msg/Trajectory.msg"
  # "msg/FlowerDetection.msg"
  # "msg/FlowerClassification.msg"
  # "msg/FlowerTarget.msg"
  # "msg/PollinationResult.msg"
  # "msg/MissionProgress.msg"
  # "msg/MissionWaypoint.msg"
  # "msg/ZoneDefinition.msg"
  # "msg/EnvironmentData.msg"
  # "msg/PerformanceMetrics.msg"
  # "msg/SystemAlert.msg"
  # "msg/DiagnosticInfo.msg"
  # "msg/SupervisorCommand.msg"




  # Services utilisés dans drone_navigation
  "srv/LoadWaypoints.srv"
  "srv/SaveWaypoints.srv"
  "srv/GetWaypoint.srv"
  "srv/AddWaypoint.srv"
  "srv/RemoveWaypoint.srv"
  "srv/ClearWaypoints.srv"
  "srv/GotoPosition.srv"
  "srv/GotoLocal.srv"
  "srv/NextWaypoint.srv"
  "srv/UpdatePosition.srv"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  "srv/PauseMission.srv"
  "srv/ResumeMission.srv"
  "srv/GetMissionStatus.srv"
  "srv/SetMissionMode.srv"

  # Les autres services sont commentés pour accélérer la compilation
  # "srv/ArmDrone.srv"
  # "srv/CalibrateSensors.srv"
  # "srv/ConfigureCamera.srv"
  # "srv/DetectFlowers.srv"
  # "srv/DisarmDrone.srv"
  # "srv/EmergencyStop.srv"
  # "srv/ExecutePollination.srv"
  # "srv/GetSystemStatus.srv"
  # "srv/Land.srv"
  # "srv/LoadMission.srv"
  # "srv/NavigateToWaypoint.srv"
  # "srv/ReturnToLaunch.srv"
  # "srv/SafetyCheck.srv"
  # "srv/SaveData.srv"
  # "srv/SetFlightMode.srv"
  # "srv/SetGeofence.srv"
  # "srv/SetTrajectory.srv"
  # "srv/SetWaypoint.srv"
  # "srv/SetWaypoints.srv"
  # "srv/GetWaypoints.srv"
  # "srv/SystemDiagnostic.srv"
  # "srv/Takeoff.srv"
  # "srv/UpdateParameters.srv"
  # "srv/CancelNavigation.srv"

  


  # Actions utilisées dans drone_navigation
  "action/GotoPositionAction.action"

  # Les autres actions sont commentées pour accélérer la compilation
  # "action/CollectData.action"
  # "action/EmergencyLanding.action"
  # "action/ExecuteMission.action"
  # "action/FollowTrajectory.action"
  # "action/MapArea.action"
  # "action/MonitorZone.action"
  # "action/NavigateToPosition.action"
  # "action/PerformMaintenance.action"
  # "action/SearchAndPollinate.action"
  
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  diagnostic_msgs
  nav_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Custom tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_messages test/test_messages.py)
  ament_add_pytest_test(test_services test/test_services.py)
  ament_add_pytest_test(test_actions test/test_actions.py)
endif()

# Install resource marker
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/resource)

ament_package()
