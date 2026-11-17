#include "dog_slam/loop_registration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace dog_slam
{

LoopRegistration::LoopRegistration(Config config)
: config_(std::move(config))
{
}

RegistrationResult LoopRegistration::align(
  const Cloud::ConstPtr & target, const Cloud::ConstPtr & source,
  const std::vector<Eigen::Isometry3d> & hypotheses) const
{
  RegistrationResult best;
  if (!target || !source ||
    target->size() < static_cast<std::size_t>(config_.min_inliers) ||
    source->size() < static_cast<std::size_t>(config_.min_inliers))
  {
    return best;
  }

  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector3d> source_points;
  target_points.reserve(target->size());
  source_points.reserve(source->size());
  for (const auto & point : target->points) {
    target_points.emplace_back(point.x, point.y, point.z);
  }
  for (const auto & point : source->points) {
    source_points.emplace_back(point.x, point.y, point.z);
  }
  auto [target_preprocessed, target_tree] = small_gicp::preprocess_points(
    target_points, config_.voxel_size, 10, std::max(1, config_.num_threads));
  auto [source_preprocessed, source_tree] = small_gicp::preprocess_points(
    source_points, config_.voxel_size, 10, std::max(1, config_.num_threads));
  (void)source_tree;

  pcl::KdTreeFLANN<pcl::PointXYZI> evaluation_tree;
  evaluation_tree.setInputCloud(target);
  for (const auto & initial : hypotheses) {
    using RobustGicp = small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>;
    small_gicp::Registration<RobustGicp, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = std::max(1, config_.num_threads);
    registration.rejector.max_dist_sq =
      config_.max_correspondence * config_.max_correspondence;
    registration.optimizer.max_iterations = std::max(5, config_.max_iterations);
    registration.criteria.rotation_eps = 0.05 * M_PI / 180.0;
    registration.criteria.translation_eps = 5.0e-4;
    registration.point_factor.robust_kernel.c = std::max(0.05, config_.robust_kernel_scale);
    const auto result = registration.align(
      *target_preprocessed, *source_preprocessed, *target_tree, initial);
    if (!result.T_target_source.matrix().allFinite()) {
      if (best.rejection_reason.empty()) {
        best.rejection_reason = "nonfinite_transform";
      }
      continue;
    }

    std::size_t inliers = 0;
    double squared_error = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    for (const auto & point : source->points) {
      const Eigen::Vector3d transformed = result.T_target_source *
        Eigen::Vector3d(point.x, point.y, point.z);
      pcl::PointXYZI query;
      query.x = static_cast<float>(transformed.x());
      query.y = static_cast<float>(transformed.y());
      query.z = static_cast<float>(transformed.z());
      if (evaluation_tree.nearestKSearch(query, 1, indices, squared_distances) == 1 &&
        squared_distances.front() <= config_.inlier_distance * config_.inlier_distance)
      {
        ++inliers;
        squared_error += squared_distances.front();
      }
    }
    const double overlap = static_cast<double>(inliers) /
      std::max<std::size_t>(1, source->size());
    const double rmse = inliers > 0 ? std::sqrt(squared_error / inliers) :
      std::numeric_limits<double>::infinity();
    const Eigen::Matrix<double, 6, 6> normalized_information =
      result.H / std::max<std::size_t>(1, result.num_inliers);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
      normalized_information);
    double min_eigenvalue = 0.0;
    double condition_number = std::numeric_limits<double>::infinity();
    if (solver.info() == Eigen::Success) {
      min_eigenvalue = std::max(0.0, solver.eigenvalues().minCoeff());
      const double maximum = std::max(0.0, solver.eigenvalues().maxCoeff());
      if (min_eigenvalue > 1.0e-12) {
        condition_number = maximum / min_eigenvalue;
      }
    }
    const Eigen::Vector3d rotated_up =
      result.T_target_source.linear() * Eigen::Vector3d::UnitZ();
    const double tilt_rad = std::acos(std::clamp(rotated_up.z(), -1.0, 1.0));
    std::string rejection_reason;
    const auto reject = [&rejection_reason](const char * reason) {
        if (!rejection_reason.empty()) {
          rejection_reason += ",";
        }
        rejection_reason += reason;
      };
    if (!result.converged) {
      reject("not_converged");
    }
    if (inliers < static_cast<std::size_t>(config_.min_inliers)) {
      reject("inliers");
    }
    if (overlap < config_.min_overlap) {
      reject("overlap");
    }
    if (rmse > config_.max_rmse) {
      reject("rmse");
    }
    if (result.T_target_source.translation().head<2>().norm() > config_.max_translation) {
      reject("translation");
    }
    if (std::abs(result.T_target_source.translation().z()) > config_.max_z) {
      reject("z");
    }
    if (tilt_rad > config_.max_roll_pitch_rad) {
      reject("roll_pitch");
    }
    if (min_eigenvalue < config_.min_information_eigenvalue) {
      reject("min_eigenvalue");
    }
    if (condition_number > config_.max_condition_number) {
      reject("condition_number");
    }
    const bool accepted = rejection_reason.empty() &&
      inliers >= static_cast<std::size_t>(config_.min_inliers) &&
      overlap >= config_.min_overlap && rmse <= config_.max_rmse &&
      result.T_target_source.translation().head<2>().norm() <= config_.max_translation &&
      std::abs(result.T_target_source.translation().z()) <= config_.max_z &&
      tilt_rad <= config_.max_roll_pitch_rad &&
      min_eigenvalue >= config_.min_information_eigenvalue &&
      condition_number <= config_.max_condition_number;
    if ((accepted && !best.accepted) ||
      (accepted == best.accepted && rmse < best.rmse))
    {
      best.accepted = accepted;
      best.converged = result.converged;
      best.iterations = result.iterations;
      best.target_from_source = result.T_target_source;
      best.num_inliers = inliers;
      best.overlap = overlap;
      best.rmse = rmse;
      best.min_eigenvalue = min_eigenvalue;
      best.condition_number = condition_number;
      best.rejection_reason = rejection_reason;
    }
  }
  return best;
}

}  // namespace dog_slam
