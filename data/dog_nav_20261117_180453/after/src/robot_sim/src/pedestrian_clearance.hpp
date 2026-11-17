#pragma once

namespace robot_sim
{
// Preserve the exclusion distance for approaching actors. If the robot has
// approached a stationary actor, permit steps that increase their separation;
// otherwise every small escape step remains inside the exclusion circle.
inline bool robotBlocksPedestrian(
  double current_distance, double next_distance, double clearance = 0.70)
{
  return next_distance < clearance && next_distance <= current_distance + 1.0e-6;
}
}  // namespace robot_sim
