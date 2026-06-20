cmake_minimum_required(VERSION 3.8)
project(rm_deskew_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 开启 O3 优化，对高频循环去畸变至关重要
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 必须开启 C++17 以支持 std::execution 并行策略
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(deskew_node src/deskew_node.cpp)

target_include_directories(deskew_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 链接 TBB 并行计算库
target_link_libraries(deskew_node
  TBB::tbb
)

ament_target_dependencies(deskew_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

install(TARGETS deskew_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()