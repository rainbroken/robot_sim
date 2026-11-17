# World provenance

## Family house

`family_house.world`, `family_house.layout.json` and
`../config/family_house_mapping_route.yaml` are generated locally from
`../src/generate_family_house.cpp`. The generator contains the complete geometry,
colors, collision shapes and route; no downloaded models or textures are used.
Regenerating intentionally replaces these three generated files only.
Floor decorations/windows are visual-only; all physical walls and furniture parts
have matching box collisions. Door openings are open and fixed (no door dynamics).
The roof is intentionally omitted for inspection. Opaque window panels sit on
solid walls; no unmodeled glass opening is used for LiDAR.

## Office

Source: `/home/rain/working/navigation_interview_candidate/src/nav_interview_bringup/worlds/interview_office.sdf`

Source SHA-256: `b2e2c10ee78e4eab6332c46ee15db5c50acb51b9d01d67eb88cef1e9c1edf412`

All 12 static models, their poses, collision geometry and materials are preserved.
Fortress system plugins and the interview_bot include are removed. SDF is set to
1.6 and the physics step to 0.002 s for Gazebo Classic 11 and the small wheels.
A local GUI camera pose frames the office from above the south wall.
The robot is spawned separately at world (0, 0, 0.003), not the original
interview_bot start (-4.8, -4.6). The source project is not modified.
This is a scenario reuse, not a run of the source project's formal benchmark.
