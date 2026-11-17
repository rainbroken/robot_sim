#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

namespace dog_slam
{

class PoseGraph
{
public:
  struct Config
  {
    double odom_rotation_sigma = 0.015;
    double odom_translation_sigma = 0.05;
    double loop_rotation_sigma = 0.04;
    double loop_translation_sigma = 0.12;
    double loop_cauchy_scale = 1.0;
    double max_loop_rmse = 0.32;
  };

  explicit PoseGraph(Config config);
  ~PoseGraph();
  PoseGraph(const PoseGraph &) = delete;
  PoseGraph & operator=(const PoseGraph &) = delete;

  std::vector<Eigen::Isometry3d> addKeyframe(const Eigen::Isometry3d & local_pose);
  std::vector<Eigen::Isometry3d> addLoop(
    std::size_t target, std::size_t current,
    const Eigen::Isometry3d & target_from_current, double rmse, double overlap);
  std::size_t size() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_slam
