cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalizationStatus.msg" "msg/MapCloudUpdate.msg"
  "srv/SavePcdMap.srv" "srv/LoadPcdMap.srv" "srv/SetLocalizationPose.srv"
  DEPENDENCIES geometry_msgs map_msgs nav_msgs sensor_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
