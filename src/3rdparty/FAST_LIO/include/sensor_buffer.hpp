#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../src/IMU_Processing.hpp"
#include "../src/preprocess.h"

class SensorBuffer
{
public:
    static constexpr std::size_t kMaxPendingLidarScans = 2;
    static constexpr std::size_t kMaxPendingImuSamples = 400;

    explicit SensorBuffer(Preprocess & preprocess) : preprocess_(preprocess) {}

    void configure(bool time_sync_enabled, double lidar_to_imu_offset);
    void pushStandard(sensor_msgs::msg::PointCloud2::UniquePtr message);
    void pushLivox(livox_ros_driver2::msg::CustomMsg::UniquePtr message);
    void pushImu(sensor_msgs::msg::Imu::UniquePtr message);

    bool syncPackages(MeasureGroup & measures, double & lidar_end_time);
    void waitForData(const std::atomic_bool & stop_requested);
    void notifyStop();

    int scanCount() const;
    double lastPreprocessDuration() const;

private:
    void trimLidarBufferLocked();

    Preprocess & preprocess_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

    bool time_sync_enabled_ = false;
    double lidar_to_imu_offset_ = 0.0;
    double last_timestamp_lidar_ = 0.0;
    double last_timestamp_imu_ = -1.0;
    double timediff_lidar_wrt_imu_ = 0.0;
    bool timediff_set_ = false;
    bool is_first_lidar_ = true;
    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int scan_count_ = 0;
    double last_preprocess_duration_ = 0.0;
};
