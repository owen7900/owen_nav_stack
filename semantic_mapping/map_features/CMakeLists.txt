cmake_minimum_required(VERSION 3.8)
project(map_features)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roomba_msgs REQUIRED)
find_package(owen_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME} src/feature_node.cpp src/FeaturesInterface.cpp)
ament_target_dependencies(${PROJECT_NAME}
        Eigen3
        yaml-cpp
        rclcpp
        owen_common
        roomba_msgs)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS
        ${PROJECT_NAME}
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
