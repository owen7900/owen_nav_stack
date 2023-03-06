cmake_minimum_required(VERSION 3.8)
project(dummy_elevator_traverser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(roomba_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(owen_common REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)
find_package(tf2 REQUIRED)

add_executable(dummy_elevator_traverser src/dummy_elevator_traverser_node.cpp src/DummyElevatorTraverser.cpp)
target_include_directories(dummy_elevator_traverser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dummy_elevator_traverser PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dummy_elevator_traverser
  "roomba_msgs"
  "std_msgs"
  "owen_common"
  rclcpp
  rclcpp_action
  apriltag_msgs
  yaml-cpp
  tf2_ros
  tf2
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS dummy_elevator_traverser
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
