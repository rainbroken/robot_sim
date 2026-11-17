#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "dog_slam/loop_closure_types.hpp"

namespace dog_slam
{

class LoopRegistration
{
public:
  struct Config
  {
    double voxel_size = 0.30;
    double max_correspondence = 1.20;
    double inlier_distance = 0.45;
    double robust_kernel_scale = 1.0;
    int max_iterations = 32;
    int num_threads = 2;
    int min_inliers = 220;
    double min_overlap = 0.30;
    double max_rmse = 0.32;
    double max_translation = 6.0;
    double max_z = 1.2;
    double max_roll_pitch_rad = 0.35;
    double min_information_eigenvalue = 1.0e-5;
    double max_condition_number = 1.0e8;
  };

  explicit LoopRegistration(Config config);

  RegistrationResult align(
    const Cloud::ConstPtr & target, const Cloud::ConstPtr & source,
    const std::vector<Eigen::Isometry3d> & hypotheses) const;

private:
  Config config_;
};

}  // namespace dog_slam
