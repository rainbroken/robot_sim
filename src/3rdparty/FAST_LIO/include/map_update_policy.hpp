#pragma once

#include <Eigen/Core>

class MapUpdatePolicy
{
public:
    struct Config
    {
        bool enabled = true;
        double min_translation = 0.03;
        double min_rotation = 0.02;
        double max_interval_sec = 0.5;
    };

    void setConfig(const Config & config);
    bool shouldUpdate(
        const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
        double time_sec, bool ekf_initialized) const;
    void record(
        const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
        double time_sec);
    void reset();

private:
    Config config_;
    Eigen::Vector3d last_position_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d last_rotation_{Eigen::Matrix3d::Identity()};
    double last_time_sec_{0.0};
    bool have_last_pose_{false};
};
