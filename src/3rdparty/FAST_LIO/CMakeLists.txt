cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(fast_lio_compile_definitions
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    list(APPEND fast_lio_compile_definitions MP_EN MP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    list(APPEND fast_lio_compile_definitions MP_EN MP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    list(APPEND fast_lio_compile_definitions MP_PROC_NUM=1)
  endif()
else()
  list(APPEND fast_lio_compile_definitions MP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
find_package(Threads REQUIRED)

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(
  fastlio_mapping
  src/laserMapping.cpp
  src/incremental_map.cpp
  src/map_update_policy.cpp
  src/mapping_output.cpp
  src/sensor_buffer.cpp
  src/scan_registration.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
)
target_compile_definitions(fastlio_mapping PRIVATE ${fast_lio_compile_definitions})
target_compile_options(fastlio_mapping PRIVATE
  -fexceptions
)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  Eigen3::Eigen
  Threads::Threads
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
