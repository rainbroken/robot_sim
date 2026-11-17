#include "map_update_policy.hpp"

#include "so3_math.h"

void MapUpdatePolicy::setConfig(const Config & config)
{
    config_ = config;
}

bool MapUpdatePolicy::shouldUpdate(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
    double time_sec, bool ekf_initialized) const
{
    if (!config_.enabled || !ekf_initialized || !have_last_pose_) {
        return true;
    }

    const double translation = (position - last_position_).norm();
    const Eigen::Matrix3d relative_rotation = last_rotation_.transpose() * rotation;
    const double rotation_distance = Log(relative_rotation).norm();
    const double elapsed = time_sec - last_time_sec_;

    return translation >= config_.min_translation ||
           rotation_distance >= config_.min_rotation ||
           elapsed < 0.0 || elapsed >= config_.max_interval_sec;
}

void MapUpdatePolicy::record(
    const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation, double time_sec)
{
    last_position_ = position;
    last_rotation_ = rotation;
    last_time_sec_ = time_sec;
    have_last_pose_ = true;
}

void MapUpdatePolicy::reset()
{
    have_last_pose_ = false;
    last_position_.setZero();
    last_rotation_.setIdentity();
    last_time_sec_ = 0.0;
}
