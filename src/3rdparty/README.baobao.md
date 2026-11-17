# Imported SLAM implementation

Reference: `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS`, imported 2026-11-17.

* `FAST_LIO`: reference modified FAST-LIO2 (source/include and runtime assets).
* `livox_ros_driver2`: original driver and messages; no hardware driver is
  launched by the baobao simulation. `package_ROS2.xml` selected as package.xml.
* `small_gicp`: built by the core, excluded from separate colcon discovery.

No reference robot dimensions, sensor extrinsics, quadruped odometry, supervisor,
CPU affinity, camera calibration or startup scripts are used. All active baobao
parameters and launch wiring are in `src/robot_slam`.

Local FAST-LIO addition: lidar_type 5 explicitly supports instantaneous Gazebo
PointCloud2 snapshots. All point offsets are zero; no invented rotating sweep
times are used. Original CustomMsg modes remain available.

The imported SLAM core is integrated into `../robot_slam/src` and
`../robot_slam/include/dog_slam` (internal C++ namespace retained). The five
message/service definitions are in `../robot_interfaces`, package `robot_interfaces`.
