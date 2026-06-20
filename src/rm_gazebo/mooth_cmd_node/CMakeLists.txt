cmake_minimum_required(VERSION 3.8)
project(smooth_cmd_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(smooth_cmd_node src/smooth_cmd_node.cpp)
ament_target_dependencies(smooth_cmd_node rclcpp geometry_msgs)

install(TARGETS smooth_cmd_node
  DESTINATION lib/${PROJECT_NAME})

# 这一句是关键，用于安装 launch/ 和 config/
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

