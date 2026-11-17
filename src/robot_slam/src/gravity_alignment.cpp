#include "dog_slam/gravity_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dog_slam
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;

bool finite(const Eigen::Vector3d & vector)
{
  return vector.array().isFinite().all();
}

}  // namespace

GravityAlignment::GravityAlignment(GravityAlignmentConfig config)
: config_(config)
{
  if (config_.minimum_stationary_duration_sec < 0.0 || config_.minimum_samples == 0 ||
    config_.expected_acceleration_norm <= 0.0 || config_.acceleration_norm_tolerance < 0.0 ||
    config_.max_gyro_norm_rad_s < 0.0 || config_.max_gravity_direction_stddev_deg < 0.0 ||
    config_.minimum_mean_stability_duration_sec < 0.0 ||
    config_.max_mean_direction_change_deg < 0.0)
  {
    throw std::invalid_argument("invalid gravity alignment configuration");
  }
}

bool GravityAlignment::addSample(
  double stamp_sec, const Eigen::Quaterniond & world_from_imu,
  const Eigen::Vector3d & acceleration_imu, const Eigen::Vector3d & angular_velocity_imu)
{
  if (locked_stats_) {
    return true;
  }
  if (!std::isfinite(stamp_sec) || !finite(acceleration_imu) || !finite(angular_velocity_imu) ||
    !std::isfinite(world_from_imu.norm()) || world_from_imu.norm() < 1e-9)
  {
    reset("non-finite IMU or odometry value");
    return false;
  }

  const double acceleration_norm = acceleration_imu.norm();
  const double gyro_norm = angular_velocity_imu.norm();
  if (std::abs(acceleration_norm - config_.expected_acceleration_norm) >
    config_.acceleration_norm_tolerance)
  {
    reset("acceleration norm outside stationary range");
    return false;
  }
  if (gyro_norm > config_.max_gyro_norm_rad_s) {
    reset("angular velocity outside stationary range");
    return false;
  }
  if (!samples_.empty() && stamp_sec <= samples_.back().stamp_sec) {
    reset("non-monotonic IMU timestamp");
    return false;
  }

  const Eigen::Vector3d up_world =
    world_from_imu.normalized() * (acceleration_imu / acceleration_norm);
  samples_.push_back({stamp_sec, acceleration_norm, gyro_norm, up_world.normalized()});
  last_rejection_reason_.clear();

  // Keep a bounded rolling window. An early noisy FAST-LIO estimate must age
  // out naturally instead of forcing every subsequent stationary window to
  // restart from a single sample.
  const double window_sec = std::max(1.0, config_.minimum_stationary_duration_sec * 2.0);
  const double oldest_stamp = stamp_sec - window_sec;
  const auto first_retained = std::lower_bound(
    samples_.begin(), samples_.end(), oldest_stamp,
    [](const Sample & sample, double cutoff) {return sample.stamp_sec < cutoff;});
  samples_.erase(samples_.begin(), first_retained);

  const double duration = samples_.back().stamp_sec - samples_.front().stamp_sec;
  if (samples_.size() < config_.minimum_samples ||
    duration < config_.minimum_stationary_duration_sec)
  {
    return false;
  }

  const auto candidate = calculateStats();
  if (candidate.gravity_direction_stddev_deg > config_.max_gravity_direction_stddev_deg) {
    stable_mean_reference_.reset();
    std::ostringstream reason;
    reason.precision(3);
    reason << "gravity direction spread " << std::fixed
           << candidate.gravity_direction_stddev_deg << " deg exceeds "
           << config_.max_gravity_direction_stddev_deg << " deg";
    last_rejection_reason_ = reason.str();
    return false;
  }

  if (config_.minimum_mean_stability_duration_sec > 0.0) {
    if (!stable_mean_reference_) {
      stable_mean_reference_ = candidate.up_world;
      stable_mean_since_sec_ = stamp_sec;
      last_rejection_reason_ = "waiting for mean gravity direction to settle";
      return false;
    }

    const double mean_change_deg = std::acos(
      std::clamp(
        stable_mean_reference_->dot(candidate.up_world), -1.0, 1.0)) * kRadiansToDegrees;
    if (mean_change_deg > config_.max_mean_direction_change_deg) {
      stable_mean_reference_ = candidate.up_world;
      stable_mean_since_sec_ = stamp_sec;
      std::ostringstream reason;
      reason.precision(3);
      reason << "mean gravity direction changed " << std::fixed << mean_change_deg
             << " deg; stability timer restarted";
      last_rejection_reason_ = reason.str();
      return false;
    }

    const double stable_duration = stamp_sec - stable_mean_since_sec_;
    if (stable_duration < config_.minimum_mean_stability_duration_sec) {
      std::ostringstream reason;
      reason.precision(2);
      reason << "mean gravity direction stable for " << std::fixed << stable_duration
             << " of " << config_.minimum_mean_stability_duration_sec << " s";
      last_rejection_reason_ = reason.str();
      return false;
    }
  }

  locked_stats_ = candidate;
  last_rejection_reason_.clear();
  return true;
}

void GravityAlignment::reset(const std::string & reason)
{
  samples_.clear();
  stable_mean_reference_.reset();
  stable_mean_since_sec_ = 0.0;
  locked_stats_.reset();
  last_rejection_reason_ = reason;
}

bool GravityAlignment::ready() const
{
  return locked_stats_.has_value();
}

const std::string & GravityAlignment::lastRejectionReason() const
{
  return last_rejection_reason_;
}

GravityAlignmentStats GravityAlignment::stats() const
{
  return locked_stats_ ? *locked_stats_ : calculateStats();
}

Eigen::Isometry3d GravityAlignment::mapFromWorld(
  const Eigen::Isometry3d & world_from_base) const
{
  if (!locked_stats_) {
    throw std::logic_error("gravity alignment has not locked");
  }

  const Eigen::Quaterniond gravity_rotation = Eigen::Quaterniond::FromTwoVectors(
    locked_stats_->up_world, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d forward_level =
    gravity_rotation * world_from_base.linear() * Eigen::Vector3d::UnitX();
  const Eigen::Vector2d forward_xy(forward_level.x(), forward_level.y());
  if (forward_xy.norm() < 1e-6) {
    throw std::runtime_error("initial base forward direction is vertical");
  }
  const double yaw = std::atan2(forward_xy.y(), forward_xy.x());
  const Eigen::Matrix3d map_from_world_rotation =
    Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    gravity_rotation.toRotationMatrix();

  Eigen::Isometry3d map_from_world = Eigen::Isometry3d::Identity();
  map_from_world.linear() = map_from_world_rotation;
  map_from_world.translation() = -map_from_world_rotation * world_from_base.translation();
  return map_from_world;
}

GravityAlignmentStats GravityAlignment::calculateStats() const
{
  GravityAlignmentStats result;
  result.sample_count = samples_.size();
  if (samples_.empty()) {
    return result;
  }

  Eigen::Vector3d up_sum = Eigen::Vector3d::Zero();
  double acceleration_sum = 0.0;
  double gyro_squared_sum = 0.0;
  for (const auto & sample : samples_) {
    up_sum += sample.up_world;
    acceleration_sum += sample.acceleration_norm;
    gyro_squared_sum += sample.gyro_norm * sample.gyro_norm;
  }
  result.duration_sec = samples_.back().stamp_sec - samples_.front().stamp_sec;
  result.mean_acceleration_norm = acceleration_sum / samples_.size();
  result.gyro_rms_rad_s = std::sqrt(gyro_squared_sum / samples_.size());
  if (up_sum.norm() < 1e-9) {
    result.gravity_direction_stddev_deg = std::numeric_limits<double>::infinity();
    return result;
  }
  result.up_world = up_sum.normalized();

  double direction_squared_sum = 0.0;
  for (const auto & sample : samples_) {
    const double cosine = std::clamp(sample.up_world.dot(result.up_world), -1.0, 1.0);
    const double angle = std::acos(cosine);
    direction_squared_sum += angle * angle;
  }
  result.gravity_direction_stddev_deg =
    std::sqrt(direction_squared_sum / samples_.size()) * kRadiansToDegrees;
  return result;
}

}  // namespace dog_slam
