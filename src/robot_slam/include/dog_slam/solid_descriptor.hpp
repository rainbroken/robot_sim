#pragma once

#include <Eigen/Geometry>

#include "dog_slam/loop_closure_types.hpp"

namespace dog_slam
{

class SolidDescriptorEngine
{
public:
  struct Config
  {
    int range_bins = 40;
    int azimuth_bins = 60;
    int elevation_bins = 32;
    double min_elevation_rad = -0.65;
    double max_elevation_rad = 1.10;
    double min_point_range = 0.8;
    double max_point_range = 55.0;
  };

  explicit SolidDescriptorEngine(Config config);

  SolidDescriptor make(
    const Cloud & cloud, const Eigen::Isometry3d & world_from_base) const;
  double similarity(const SolidDescriptor & lhs, const SolidDescriptor & rhs) const;
  double yaw(const SolidDescriptor & query, const SolidDescriptor & candidate) const;

private:
  Config config_;
};

}  // namespace dog_slam
