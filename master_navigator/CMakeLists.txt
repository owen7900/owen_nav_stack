cmake_minimum_required(VERSION 3.8)
project(master_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(owen_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(roomba_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(master_navigator src/master_navigator.cpp src/MasterNavigator.cpp src/MultifloorPathPlanner.cpp)
target_include_directories(master_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(master_navigator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  master_navigator
  "rclcpp"
  "std_msgs"
  "owen_common"
  "navigation2"
  "roomba_msgs"
  "nav2_msgs"
  "nav2_util"
  "yaml-cpp"

)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS master_navigator
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
