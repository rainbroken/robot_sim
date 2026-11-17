# Baobao navigation adapted from JUNIOR dog_nav

Reference: `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS/src/dog_nav`.
The original reference is not modified. The imported C++ files retain their
`dog_nav` namespace for traceability and build into this package's
`librobot_navigation_dwb_plugins.so`.

The local controller is `dog_nav::ParallelDwbLocalPlanner` with a configurable
velocity iterator, cached footprint collision scoring, forward progress,
goal approach gating, path rotation, stable goal rotation and lateral critics.
The global planner is SmacPlanner2D. Both single-pose and multi-pose behavior
trees use clear/spin/wait recovery without an automatic backup action.

## Robot-specific adaptation

* Keep baobao's `base_footprint`, `/odom`, footprint and lidar-to-scan pipeline.
* Differential drive: `vy = 0`, one lateral sample, no reverse commands.
* Maximum translation 0.30 m/s and maximum controller yaw rate 0.60 rad/s.
* `bpx_enforce_startup_threshold: false` for this differential simulation:
  allow low independent linear/angular speeds and retain an explicit stop.
  The reference quadruped's mixed-unit 0.21 startup rule is not a measured
  property of this chassis. The generator retains the legacy firmware mode
  (its default is still `true`) for separately calibrated BPX deployments.
* Publish to `/cmd_vel`; the existing wall-time command/localization watchdog
  still controls `/cmd_vel_safe`. Actual hardware dead zones and braking
  distances need measurement before deploying this simulation profile.
* The obstacle critic keeps native oriented-footprint collision rejection,
  additionally checks center cells, then ranks the mean center inflation cost.
  This distinguishes centered passes when footprint edges both saturate at 253.
  The 0.48 x 0.50 m footprint and 0.02 m padding are unchanged. Local resolution
  is 0.025 m to reduce wall-cell aliasing; global static maps retain their own
  native resolution. This increases local grid cells fourfold versus 0.05 m.
* Mandatory in-place path alignment begins at 1.20 rad; smaller detours can use
  collision-checked arcs. Replanning runs at 2 Hz in both behavior trees.
* Final approach penalizes steering changes inside 0.30 m, starts final-pose
  handling at 0.15 m and scores yaw at 0.6 s while checking collision over the
  full 1.7 s trajectory. The goal checker latches the XY tolerance once reached.
  This is tolerance-based navigation, not precision docking.
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

Standing-person gap tests and terminal convergence results are in
[`../../docs/NAVIGATION_GAP_VALIDATION.md`](../../docs/NAVIGATION_GAP_VALIDATION.md),
with raw records in `data/navigation_gap_20261117/`. These are ideal plant /
synthetic-scan tests using real Nav2 servers, not Gazebo or hardware acceptance.
Returning moving pedestrians remain a failed safety case; experimental motion
prediction is archived under that data directory and is not loaded or built.

Run controller regressions with `colcon test --packages-select robot_navigation`.
Run the isolated full chain after sourcing `scripts/env.sh`:

```bash
python3 scripts/test_navigation_synthetic.py --wall .85 --output /tmp/nav_gap65.json
```

The script uses a private ROS domain (217 by default), refuses an occupied
domain, waits for lifecycle activation, records the exact parameter file, and
shuts down its own Nav2 processes. It does not connect to a hardware ROS graph.
Override the navigation YAML with the `params_file` launch argument when testing
a separate profile. Earlier import records remain under `data/dog_nav_*`.
