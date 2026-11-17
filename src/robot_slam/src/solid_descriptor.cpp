#include "dog_slam/solid_descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Core>

namespace dog_slam
{
namespace
{
double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawOf(const Eigen::Matrix3d & rotation)
{
  const Eigen::Vector3d forward = rotation * Eigen::Vector3d::UnitX();
  return std::atan2(forward.y(), forward.x());
}
}  // namespace

SolidDescriptorEngine::SolidDescriptorEngine(Config config)
: config_(std::move(config))
{
}

SolidDescriptor SolidDescriptorEngine::make(
  const Cloud & cloud, const Eigen::Isometry3d & world_from_base) const
{
  SolidDescriptor descriptor;
  descriptor.range = Eigen::VectorXd::Zero(std::max(4, config_.range_bins));
  descriptor.azimuth = Eigen::VectorXd::Zero(std::max(8, config_.azimuth_bins));
  const int elevation_bins = std::max(4, config_.elevation_bins);
  Eigen::MatrixXd range_matrix = Eigen::MatrixXd::Zero(
    descriptor.range.size(), elevation_bins);
  Eigen::MatrixXd azimuth_matrix = Eigen::MatrixXd::Zero(
    descriptor.azimuth.size(), elevation_bins);
  const double pose_yaw = yawOf(world_from_base.linear());
  const Eigen::Matrix3d gravity_from_base =
    Eigen::AngleAxisd(-pose_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    world_from_base.linear();
  const double elevation_span = std::max(
    0.1, config_.max_elevation_rad - config_.min_elevation_rad);

  for (const auto & point : cloud.points) {
    const Eigen::Vector3d p = gravity_from_base * Eigen::Vector3d(point.x, point.y, point.z);
    const double radius = std::hypot(p.x(), p.y());
    if (radius < config_.min_point_range || radius > config_.max_point_range) {
      continue;
    }
    const double elevation = std::atan2(p.z(), radius);
    if (elevation < config_.min_elevation_rad || elevation > config_.max_elevation_rad) {
      continue;
    }
    double azimuth = std::atan2(p.y(), p.x());
    if (azimuth < 0.0) {
      azimuth += 2.0 * M_PI;
    }
    const int range_index = std::clamp(
      static_cast<int>(radius / config_.max_point_range * descriptor.range.size()), 0,
      static_cast<int>(descriptor.range.size()) - 1);
    const int azimuth_index = std::clamp(
      static_cast<int>(azimuth / (2.0 * M_PI) * descriptor.azimuth.size()), 0,
      static_cast<int>(descriptor.azimuth.size()) - 1);
    const int elevation_index = std::clamp(
      static_cast<int>(
        (elevation - config_.min_elevation_rad) / elevation_span * elevation_bins),
      0, elevation_bins - 1);
    range_matrix(range_index, elevation_index) += 1.0;
    azimuth_matrix(azimuth_index, elevation_index) += 1.0;
  }

  Eigen::VectorXd weights = range_matrix.colwise().sum().transpose();
  if (weights.size() > 0) {
    const double minimum = weights.minCoeff();
    const double span = weights.maxCoeff() - minimum;
    if (span > 1.0e-9) {
      weights = (weights.array() - minimum) / span;
    } else if (weights.maxCoeff() > 0.0) {
      weights.setOnes();
    }
  }
  descriptor.range = range_matrix * weights;
  descriptor.azimuth = azimuth_matrix * weights;
  if (descriptor.range.norm() > 1.0e-9) {
    descriptor.range.normalize();
  }
  if (descriptor.azimuth.norm() > 1.0e-9) {
    descriptor.azimuth.normalize();
  }
  return descriptor;
}

double SolidDescriptorEngine::similarity(
  const SolidDescriptor & lhs, const SolidDescriptor & rhs) const
{
  if (lhs.range.size() != rhs.range.size() || lhs.range.norm() < 1.0e-9 ||
    rhs.range.norm() < 1.0e-9)
  {
    return 0.0;
  }
  return std::clamp(lhs.range.dot(rhs.range), -1.0, 1.0);
}

double SolidDescriptorEngine::yaw(
  const SolidDescriptor & query, const SolidDescriptor & candidate) const
{
  if (query.azimuth.size() != candidate.azimuth.size() || query.azimuth.size() == 0) {
    return 0.0;
  }
  int best_shift = 0;
  double best_score = -std::numeric_limits<double>::infinity();
  const int bins = query.azimuth.size();
  for (int shift = 0; shift < bins; ++shift) {
    double score = 0.0;
    for (int index = 0; index < bins; ++index) {
      score += query.azimuth(index) * candidate.azimuth((index + shift) % bins);
    }
    if (score > best_score) {
      best_score = score;
      best_shift = shift;
    }
  }
  return wrapAngle(best_shift * (2.0 * M_PI / bins));
}

}  // namespace dog_slam
