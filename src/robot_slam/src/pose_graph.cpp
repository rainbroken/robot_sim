#include "dog_slam/pose_graph.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace dog_slam
{
namespace
{
gtsam::Pose3 eigenToGtsam(const Eigen::Isometry3d & pose)
{
  return gtsam::Pose3(gtsam::Rot3(pose.linear()), gtsam::Point3(pose.translation()));
}

Eigen::Isometry3d gtsamToEigen(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.linear() = pose.rotation().matrix();
  output.translation() = pose.translation();
  return output;
}
}  // namespace

class PoseGraph::Impl
{
public:
  explicit Impl(Config config)
  : config_(std::move(config))
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(parameters);
  }

  gtsam::SharedNoiseModel odometryNoise() const
  {
    gtsam::Vector6 sigmas;
    sigmas << config_.odom_rotation_sigma, config_.odom_rotation_sigma,
      config_.odom_rotation_sigma, config_.odom_translation_sigma,
      config_.odom_translation_sigma, config_.odom_translation_sigma;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  }

  gtsam::SharedNoiseModel loopNoise(double rmse, double overlap) const
  {
    const double quality_scale = std::clamp(
      (1.0 + rmse / std::max(0.05, config_.max_loop_rmse)) /
      std::max(0.25, overlap), 1.0, 5.0);
    gtsam::Vector6 sigmas;
    sigmas << config_.loop_rotation_sigma * quality_scale,
      config_.loop_rotation_sigma * quality_scale,
      config_.loop_rotation_sigma * quality_scale,
      config_.loop_translation_sigma * quality_scale,
      config_.loop_translation_sigma * quality_scale,
      config_.loop_translation_sigma * quality_scale;
    auto base = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    auto estimator = gtsam::noiseModel::mEstimator::Cauchy::Create(
      std::max(0.1, config_.loop_cauchy_scale));
    return gtsam::noiseModel::Robust::Create(estimator, base);
  }

  std::vector<Eigen::Isometry3d> calculate() const
  {
    const gtsam::Values estimate = isam_->calculateEstimate();
    std::vector<Eigen::Isometry3d> optimized(local_poses_.size());
    for (std::size_t index = 0; index < local_poses_.size(); ++index) {
      const gtsam::Symbol key('x', index);
      optimized[index] = estimate.exists(key) ?
        gtsamToEigen(estimate.at<gtsam::Pose3>(key)) : local_poses_[index];
    }
    return optimized;
  }

  Config config_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::vector<Eigen::Isometry3d> local_poses_;
  Eigen::Isometry3d last_optimized_pose_ = Eigen::Isometry3d::Identity();
  bool have_optimized_pose_ = false;
};

PoseGraph::PoseGraph(Config config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

PoseGraph::~PoseGraph() = default;

std::vector<Eigen::Isometry3d> PoseGraph::addKeyframe(
  const Eigen::Isometry3d & local_pose)
{
  const std::size_t index = impl_->local_poses_.size();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  if (index == 0) {
    gtsam::Vector6 prior_sigmas;
    prior_sigmas << 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-3;
    graph.add(
      gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0), eigenToGtsam(local_pose),
        gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
    initial.insert(gtsam::Symbol('x', 0), eigenToGtsam(local_pose));
  } else {
    const Eigen::Isometry3d odometry = impl_->local_poses_.back().inverse() * local_pose;
    graph.add(
      gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', index - 1), gtsam::Symbol('x', index),
        eigenToGtsam(odometry), impl_->odometryNoise()));
    initial.insert(
      gtsam::Symbol('x', index),
      eigenToGtsam(
        (impl_->have_optimized_pose_ ? impl_->last_optimized_pose_ : impl_->local_poses_.back()) *
        odometry));
  }
  impl_->local_poses_.push_back(local_pose);
  impl_->isam_->update(graph, initial);
  impl_->isam_->update();
  auto optimized = impl_->calculate();
  impl_->last_optimized_pose_ = optimized.back();
  impl_->have_optimized_pose_ = true;
  return optimized;
}

std::vector<Eigen::Isometry3d> PoseGraph::addLoop(
  std::size_t target, std::size_t current,
  const Eigen::Isometry3d & target_from_current, double rmse, double overlap)
{
  if (target >= impl_->local_poses_.size() || current >= impl_->local_poses_.size()) {
    throw std::out_of_range("loop factor keyframe index is outside the pose graph");
  }
  gtsam::NonlinearFactorGraph graph;
  graph.add(
    gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', target), gtsam::Symbol('x', current),
      eigenToGtsam(target_from_current), impl_->loopNoise(rmse, overlap)));
  impl_->isam_->update(graph, gtsam::Values());
  for (int iteration = 0; iteration < 4; ++iteration) {
    impl_->isam_->update();
  }
  auto optimized = impl_->calculate();
  impl_->last_optimized_pose_ = optimized.back();
  impl_->have_optimized_pose_ = true;
  return optimized;
}

std::size_t PoseGraph::size() const
{
  return impl_->local_poses_.size();
}

}  // namespace dog_slam
