cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fPIC")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(basic REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudPose.msg
  msg/CloudPose2.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

rosidl_get_typesupport_target(
  c_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_c"
)

include_directories(
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  livox_ros_driver2
  basic
  rosidl_default_runtime
)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  pthread
  ${cpp_typesupport_target}
  ${c_typesupport_target}
)

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  ${cpp_typesupport_target}
  ${c_typesupport_target}
)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  ${cpp_typesupport_target}
  ${c_typesupport_target}
)

install(TARGETS
  lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  super_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Workaround for pyext linking: the Python C extension .so modules generated
# by ament_cmake_python link against lib<package>__rosidl_typesupport_c.so via a
# bare filename. On systems where the linker (ld.lld or certain ld.bfd versions)
# does not resolve bare filenames from the working directory, this causes:
#   "undefined symbol: rosidl_typesupport_c__get_message_type_support_handle__*"
# Injecting -L<build_dir> via CMAKE_{SHARED,MODULE}_LINKER_FLAGS ensures the
# linker can find the generated typesupport .so regardless of linker flavour.
# Variables are used (not link_directories) because ament creates its pyext
# targets in a subdirectory scope that does not inherit directory properties.
string(APPEND CMAKE_SHARED_LINKER_FLAGS " -L${CMAKE_CURRENT_BINARY_DIR}")
string(APPEND CMAKE_MODULE_LINKER_FLAGS " -L${CMAKE_CURRENT_BINARY_DIR}")

# Workaround: ament_cmake_python uses cmake -E create_symlink which cannot
# overwrite an existing directory. If a stale directory exists at the symlink
# destination (e.g. from an interrupted build), repeated builds will fail with:
#   "failed to create symbolic link ... Is a directory"
# This function is called at the end of the current directory scope (after
# ament_package() has created its targets) to conditionally wire up a pre-clean
# step so the build is idempotent across ROS 2 distributions.
function(_super_lio_fix_python_symlink build_dir project_name)
  set(_symlink_dir "${build_dir}/ament_cmake_python/${project_name}/${project_name}")
  if(TARGET ament_cmake_python_symlink_${project_name})
    if(NOT TARGET clean_python_symlink_${project_name})
      add_custom_target(clean_python_symlink_${project_name}
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${_symlink_dir}"
        COMMENT "Pre-cleaning stale Python symlink path for ${project_name}"
      )
    endif()
    add_dependencies(ament_cmake_python_symlink_${project_name}
      clean_python_symlink_${project_name})
  endif()
endfunction()

file(REMOVE_RECURSE
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}")

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.19")
  cmake_language(DEFER CALL _super_lio_fix_python_symlink
    "${CMAKE_CURRENT_BINARY_DIR}" "${PROJECT_NAME}")
else()
  _super_lio_fix_python_symlink("${CMAKE_CURRENT_BINARY_DIR}" "${PROJECT_NAME}")
endif()

ament_package()
