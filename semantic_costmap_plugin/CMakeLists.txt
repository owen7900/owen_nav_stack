cmake_minimum_required(VERSION 3.8)
project(semantic_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(owen_common REQUIRED)

set(dependencies
        pluginlib
        rclcpp
        nav2_costmap_2d
        owen_common
        )

#add_library(libyaml-cpp STATIC IMPORTED)
set(BUILD_SHARED_LIBS OFF)
add_library(semantic_costmap_plugin SHARED
        src/semantic_costmap_plugin.cpp)

target_compile_features(semantic_costmap_plugin PUBLIC c_std_99 cxx_std_17)

target_include_directories(semantic_costmap_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(semantic_costmap_plugin
        ${dependencies}
        )

target_compile_definitions(semantic_costmap_plugin PRIVATE "SEMANTIC_COSTMAP_PLUGIN_BUILDING_LIBRARY")

install(
        DIRECTORY include
        DESTINATION include
)

install(
        TARGETS semantic_costmap_plugin
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


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

pluginlib_export_plugin_description_file(nav2_costmap_2d semantic_map.xml)

ament_export_dependencies(${dependencies})

ament_export_include_directories(
        include
)

ament_export_libraries(
        semantic_costmap_plugin
)

ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
