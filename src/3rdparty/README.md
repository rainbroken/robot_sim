# Third-party sources

mid360_simulation is pinned to upstream commit
241cfbff81f851e4c15c1c7dba039ca29824b264 and locally patched.

See ../../docs/SOURCES.md and upstream LICENSE files. Application launch and
parameters live in sibling robot_* packages.

`navigation2` was cloned from `https://github.com/ros-navigation/navigation2.git`,
branch `humble`, commit `3c3db59d6969d8ecee8e68468693d006397f4a0c`
(upstream commit date 2026-06-03). Imported 2026-11-17. Its Apache-2.0 license is
in the checkout. System ROS 2 Humble Nav2 packages are retained. Per the request
to fetch sources only, `navigation2/COLCON_IGNORE` keeps this checkout out of
the default build; current runtime still uses the installed Nav2 framework.
To intentionally build the source overlay later, remove that marker and run
`bash scripts/build.sh --cmake-clean-cache` from the workspace root, then source
`scripts/env.sh` in a fresh terminal. Cleaning CMake caches is necessary for
existing consumers to stop retaining `/opt/ros/humble` Nav2 package paths.

Custom JUNIOR DWB controller/generator/critic sources are built in
`../robot_navigation`; their local plugin library overrides controller behavior
without copying the Nav2 framework into this third-party directory.
