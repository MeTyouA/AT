cmake_minimum_required(VERSION 3.8)
project(topo_path_planner)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Topo Path Planner Core Library (shared for -fPIC)
add_library(${PROJECT_NAME}_core SHARED
  src/topo_graph.cpp
  src/topo_node.cpp
  src/visibility_checker.cpp
  src/skeleton_extractor.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  sensor_msgs
  OpenCV
  Eigen3
)

# Topo Path Planner Node Component
add_library(topo_planner_node SHARED src/topo_planner_node.cpp)
target_link_libraries(topo_planner_node ${PROJECT_NAME}_core)
ament_target_dependencies(topo_planner_node
  rclcpp
  rclcpp_components
  rclcpp_action
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

# Register component
rclcpp_components_register_nodes(topo_planner_node "topo_path_planner::TopoPlannerNode")

# Topo Path Planner Nav2 Plugin
add_library(topo_planner_plugin SHARED src/topo_planner_plugin.cpp)
target_link_libraries(topo_planner_plugin ${PROJECT_NAME}_core)
ament_target_dependencies(topo_planner_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_core
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Export plugin description
pluginlib_export_plugin_description_file(nav2_core topo_planner_plugin.xml)

# Testing executable
add_executable(test_topo test/test_topo.cpp)
target_link_libraries(test_topo ${PROJECT_NAME}_core)
ament_target_dependencies(test_topo
  rclcpp
  nav2_costmap_2d
)

# Install
install(TARGETS
  ${PROJECT_NAME}_core
  topo_planner_node
  topo_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(DIRECTORY
  include/
  DESTINATION include/)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
