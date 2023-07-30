cmake_minimum_required(VERSION 3.8)
project(owen_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roomba_msgs REQUIRED)
find_package(owen_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package( cv_bridge REQUIRED)
find_package( sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  "rclcpp"
  "roomba_msgs"
  owen_common
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  sensor_msgs
  )

set(cpp_files
  src/NavigatorNode.cpp
  src/AStarNavigator.cpp 
  src/PathGeneratorFactory.cpp 
  src/mapping/MapManager.cpp
  src/mapping/ObstacleSourceFactory.cpp
  src/mapping/StaticMapObstacleSource.cpp
  src/mapping/LidarObstacleSource.cpp
  src/PathFollowerFactory.cpp
  src/PurePursuit.cpp
  src/RecoveryBehaviourFactory.cpp
  src/BackUpBehaviour.cpp
  src/RRTNavigator.cpp
  )

add_library(${PROJECT_NAME} ${cpp_files})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)

add_executable(owen_navigation_node 
  src/owen_navigation_node.cpp 
  )

target_include_directories(owen_navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(owen_navigation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(owen_navigation_node ${PROJECT_NAME})


ament_target_dependencies(
  owen_navigation_node
  ${dependencies}
)

install(TARGETS owen_navigation_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(planner_tester
  testing_utilities/PlannerTester.cpp
  ${cpp_files}
  )

target_include_directories(planner_tester PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planner_tester PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(planner_tester ${PROJECT_NAME})


ament_target_dependencies(
  planner_tester
  ${dependencies}
  OpenCV
)

install(TARGETS planner_tester
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
