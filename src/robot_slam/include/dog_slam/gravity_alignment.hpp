#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace dog_slam
{

struct GravityAlignmentConfig
{
  double minimum_stationary_duration_sec{3.0};
  std::size_t minimum_samples{120};
  double expected_acceleration_norm{1.0};
  double acceleration_norm_tolerance{0.15};
  double max_gyro_norm_rad_s{0.05};
  double max_gravity_direction_stddev_deg{1.0};
  double minimum_mean_stability_duration_sec{2.0};
  double max_mean_direction_change_deg{1.0};
};

struct GravityAlignmentStats
{
  std::size_t sample_count{0};
  double duration_sec{0.0};
  double mean_acceleration_norm{0.0};
  double gyro_rms_rad_s{0.0};
  double gravity_direction_stddev_deg{0.0};
  Eigen::Vector3d up_world{Eigen::Vector3d::UnitZ()};
};

class GravityAlignment
{
public:
  explicit GravityAlignment(GravityAlignmentConfig config);

  bool addSample(
    double stamp_sec, const Eigen::Quaterniond & world_from_imu,
    const Eigen::Vector3d & acceleration_imu, const Eigen::Vector3d & angular_velocity_imu);
  void reset(const std::string & reason = "");

  bool ready() const;
  const std::string & lastRejectionReason() const;
  GravityAlignmentStats stats() const;

  Eigen::Isometry3d mapFromWorld(const Eigen::Isometry3d & world_from_base) const;

private:
  struct Sample
  {
    double stamp_sec;
    double acceleration_norm;
    double gyro_norm;
    Eigen::Vector3d up_world;
  };

  GravityAlignmentStats calculateStats() const;

  GravityAlignmentConfig config_;
  std::vector<Sample> samples_;
  std::optional<Eigen::Vector3d> stable_mean_reference_;
  double stable_mean_since_sec_{0.0};
  std::optional<GravityAlignmentStats> locked_stats_;
  std::string last_rejection_reason_;
};

}  // namespace dog_slam
