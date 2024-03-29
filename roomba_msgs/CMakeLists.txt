cmake_minimum_required(VERSION 3.8)
project(roomba_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  msg/MultifloorPoint.msg
  msg/MultifloorRectangle.msg
  msg/MultifloorPath.msg
  msg/UpcomingTerrainFeature.msg
  msg/StringArray.msg
  srv/GetPathObstacles.srv
  srv/GetAvailableDestinations.srv
  srv/CanContinue.srv
  srv/GetElevatorPosition.srv
  srv/rviz/LoadConfig.srv
  srv/rviz/AddMapNode.srv
  srv/rviz/SwitchLabellingType.srv
  srv/rviz/AddLabel.srv
  action/TraverseElevator.action
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()

