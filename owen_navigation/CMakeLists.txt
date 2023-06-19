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

add_executable(owen_navigation_node 
  src/owen_navigation_node.cpp 
  src/NavigatorNode.cpp
  src/AStarNavigator.cpp 
  src/PathGeneratorFactory.cpp 
  src/mapping/Map.cpp
  src/mapping/MapManager.cpp
  src/mapping/ObstacleSourceFactory.cpp
  src/mapping/StaticMapObstacleSource.cpp
  src/PathFollowerFactory.cpp
  src/PurePursuit.cpp
  )
target_include_directories(owen_navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(owen_navigation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  owen_navigation_node
  "rclcpp"
  "roomba_msgs"
  owen_common
  nav_msgs
  geometry_msgs
)

install(TARGETS owen_navigation_node
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
