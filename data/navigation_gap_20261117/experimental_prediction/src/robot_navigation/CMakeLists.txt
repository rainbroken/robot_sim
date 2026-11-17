cmake_minimum_required(VERSION 3.8)
project(robot_navigation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
foreach(dep dwb_core dwb_plugins dwb_critics nav2_util nav2_core nav2_costmap_2d nav_2d_msgs pluginlib)
  find_package(${dep} REQUIRED)
endforeach()
add_library(robot_navigation_dwb_plugins SHARED
  src/bpx_trajectory_generator.cpp src/cached_obstacle_footprint_critic.cpp
  src/forward_progress_critic.cpp src/goal_approach_motion_critic.cpp
  src/lateral_velocity_critic.cpp src/parallel_dwb_local_planner.cpp
  src/predictive_obstacle_critic.cpp
  src/rotate_to_path_critic.cpp src/stable_rotate_to_goal_critic.cpp)
target_compile_features(robot_navigation_dwb_plugins PUBLIC cxx_std_17)
target_include_directories(robot_navigation_dwb_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_navigation_dwb_plugins
  dwb_core dwb_plugins dwb_critics geometry_msgs nav2_util nav_2d_msgs pluginlib rclcpp)
target_link_libraries(robot_navigation_dwb_plugins OpenMP::OpenMP_CXX)
pluginlib_export_plugin_description_file(dwb_core dog_nav_dwb_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core dog_nav_controller_plugins.xml)
install(TARGETS robot_navigation_dwb_plugins LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
add_executable(nav_goal src/nav_goal.cpp)
target_compile_features(nav_goal PUBLIC cxx_std_17)
ament_target_dependencies(nav_goal rclcpp rclcpp_action nav2_msgs geometry_msgs)
install(TARGETS nav_goal DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz behavior_trees DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(test_navigation_scenarios test/test_navigation_scenarios.cpp
    ENV ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1)
  if(TARGET test_navigation_scenarios)
    target_include_directories(test_navigation_scenarios PRIVATE include)
    target_compile_definitions(test_navigation_scenarios PRIVATE
      NAV_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2.yaml")
    target_link_libraries(test_navigation_scenarios robot_navigation_dwb_plugins yaml-cpp)
    ament_target_dependencies(test_navigation_scenarios nav2_costmap_2d nav2_util dwb_core)
  endif()
  ament_add_gtest(test_bpx_trajectory_generator
    test/test_bpx_trajectory_generator.cpp test/test_forward_progress_critic.cpp
    test/test_goal_approach_motion_critic.cpp test/test_lateral_velocity_critic.cpp
    test/test_rotate_to_path_critic.cpp test/test_stable_rotate_to_goal_critic.cpp)
  if(TARGET test_bpx_trajectory_generator)
    target_include_directories(test_bpx_trajectory_generator PRIVATE include)
    target_link_libraries(test_bpx_trajectory_generator robot_navigation_dwb_plugins)
    ament_target_dependencies(test_bpx_trajectory_generator dwb_core pluginlib)
  endif()
  ament_add_gtest(test_parallel_dwb_local_planner test/test_parallel_dwb_local_planner.cpp
    ENV ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1)
  if(TARGET test_parallel_dwb_local_planner)
    target_include_directories(test_parallel_dwb_local_planner PRIVATE include)
    target_link_libraries(test_parallel_dwb_local_planner robot_navigation_dwb_plugins OpenMP::OpenMP_CXX)
    ament_target_dependencies(test_parallel_dwb_local_planner dwb_core nav2_util)
  endif()
  ament_add_gtest(test_switchback_pruning test/test_switchback_pruning.cpp
    ENV ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1)
  if(TARGET test_switchback_pruning)
    ament_target_dependencies(test_switchback_pruning dwb_core nav2_util nav2_costmap_2d)
  endif()
endif()
ament_package()
