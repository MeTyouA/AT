cmake_minimum_required(VERSION 3.8)
project(mid360_chunk_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mid360_chunk_cloud_node src/mid360_chunk_cloud_node.cpp)
ament_target_dependencies(
  mid360_chunk_cloud_node
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS mid360_chunk_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
