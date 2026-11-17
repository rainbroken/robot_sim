# Baobao navigation adapted from JUNIOR dog_nav

Reference: `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS/src/dog_nav`.
The original reference is not modified. The imported C++ files retain their
`dog_nav` namespace for traceability and build into this package's
`librobot_navigation_dwb_plugins.so`.

The local controller is `dog_nav::ParallelDwbLocalPlanner` with the reference
BPX trajectory generator, cached footprint collision scoring, forward progress,
goal approach gating, path rotation, stable goal rotation and lateral critics.
The global planner is SmacPlanner2D. Both single-pose and multi-pose behavior
trees use clear/spin/wait recovery without an automatic backup action.

## Robot-specific adaptation

* Keep baobao's `base_footprint`, `/odom`, footprint and lidar-to-scan pipeline.
* Differential drive: `vy = 0`, one lateral sample, no reverse commands.
* Maximum translation 0.30 m/s and maximum controller yaw rate 0.60 rad/s.
* Keep `bpx_startup_threshold: 0.21`. The reference compares
  `sqrt(vx*vx + vy*vy + wz*wz)` with this threshold and also permits a zero
  command. This is the reference firmware convention, not a physical speed
  norm or a claim that every nonzero linear component is at least 0.21 m/s.
* Publish directly to `/cmd_vel` so velocity smoothing does not undo the startup
  samples. The existing wall-time command/localization watchdog still controls
  `/cmd_vel_safe`. Gazebo's differential drive does not model the BPX firmware
  dead zone; simulation validates sampling/output, not physical BPX startup.
* Do not import the quadruped's nav_control offset, sideways motion, 0.9 m/s
  limit, stair-region policy, hardware command mux, audio or route management.
* Keep the current scan-based obstacle layers. The reference's dog_sensors
  point-cloud filter and STVL pipeline are not part of this adaptation.

Nav2 servers, Smac and standard DWB infrastructure still come from the installed
ROS 2 Humble packages. The custom controller, generator and critics are built
from source in this workspace. Replacing the installed server binaries alone
would not reproduce the reference controller.

## Start

Saved-map localization and navigation (the gate waits for valid localization):

```bash
source scripts/env.sh
ros2 launch robot_slam simulation.launch.py mode:=navigation map_dir:=/absolute/map/directory
```

For supervised waypoint map acquisition, first start mapping and wait for
`map -> base_footprint` plus a populated `/map`. Then start navigation against
the live map in the same ROS domain; do not run `mapping_route` simultaneously:

```bash
ros2 launch robot_navigation navigation.launch.py use_live_map:=true
```

Live-map mode starts no map server and requires the external mapper to publish
the map and its transform. It is waypoint navigation against observed cells,
not an autonomous exploration algorithm. Stop this navigation instance before
switching to saved-map localization/navigation.

Build with `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.
For the imported regression tests, add `-DBUILD_TESTING=ON`, then run
`colcon test --packages-select robot_navigation` in a sourced workspace.

Validation logs and the current run report are under `data/dog_nav_*`.
