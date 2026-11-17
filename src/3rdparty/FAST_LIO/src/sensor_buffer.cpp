#include "sensor_buffer.hpp"

#include <cmath>
#include <iostream>
#include <omp.h>

void SensorBuffer::configure(bool time_sync_enabled, double lidar_to_imu_offset)
{
    std::lock_guard<std::mutex> lock(mutex_);
    time_sync_enabled_ = time_sync_enabled;
    lidar_to_imu_offset_ = lidar_to_imu_offset;
}

void SensorBuffer::trimLidarBufferLocked()
{
    while (lidar_buffer_.size() > kMaxPendingLidarScans)
    {
        const std::size_t drop_index = lidar_pushed_ ? 1 : 0;
        lidar_buffer_.erase(lidar_buffer_.begin() + drop_index);
        time_buffer_.erase(time_buffer_.begin() + drop_index);
    }
}

void SensorBuffer::pushStandard(sensor_msgs::msg::PointCloud2::UniquePtr message)
{
    const double current_time = get_time_sec(message->header.stamp);
    const double preprocess_start = omp_get_wtime();
    PointCloudXYZI::Ptr points(new PointCloudXYZI());
    preprocess_.process(message, points);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++scan_count_;
        if (!is_first_lidar_ && current_time < last_timestamp_lidar_)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer_.clear();
            time_buffer_.clear();
            lidar_pushed_ = false;
        }
        is_first_lidar_ = false;
        lidar_buffer_.push_back(points);
        time_buffer_.push_back(current_time);
        trimLidarBufferLocked();
        last_timestamp_lidar_ = current_time;
        last_preprocess_duration_ = omp_get_wtime() - preprocess_start;
    }
    condition_.notify_all();
}

void SensorBuffer::pushLivox(livox_ros_driver2::msg::CustomMsg::UniquePtr message)
{
    const double current_time = get_time_sec(message->header.stamp);
    const double preprocess_start = omp_get_wtime();
    PointCloudXYZI::Ptr points(new PointCloudXYZI());

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++scan_count_;
        if (!is_first_lidar_ && current_time < last_timestamp_lidar_)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer_.clear();
            time_buffer_.clear();
            lidar_pushed_ = false;
        }
        is_first_lidar_ = false;
        last_timestamp_lidar_ = current_time;

        if (!time_sync_enabled_ && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 &&
            !imu_buffer_.empty() && !lidar_buffer_.empty())
        {
            std::printf(
                "IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
                last_timestamp_imu_, last_timestamp_lidar_);
        }
        if (time_sync_enabled_ && !timediff_set_ &&
            std::abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1.0 && !imu_buffer_.empty())
        {
            timediff_set_ = true;
            timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
            std::printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu_);
        }
    }

    preprocess_.process(message, points);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lidar_buffer_.push_back(points);
        time_buffer_.push_back(current_time);
        trimLidarBufferLocked();
        last_preprocess_duration_ = omp_get_wtime() - preprocess_start;
    }
    condition_.notify_all();
}

void SensorBuffer::pushImu(sensor_msgs::msg::Imu::UniquePtr message)
{
    auto copied = std::make_shared<sensor_msgs::msg::Imu>(*message);
    copied->header.stamp = get_ros_time(
        get_time_sec(message->header.stamp) - lidar_to_imu_offset_);
    if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_enabled_)
    {
        copied->header.stamp = rclcpp::Time(
            timediff_lidar_wrt_imu_ + get_time_sec(message->header.stamp));
    }
    const double timestamp = get_time_sec(copied->header.stamp);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_imu_)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            imu_buffer_.clear();
        }
        last_timestamp_imu_ = timestamp;
        imu_buffer_.push_back(copied);
        while (imu_buffer_.size() > kMaxPendingImuSamples)
        {
            imu_buffer_.pop_front();
        }
    }
    condition_.notify_all();
}

bool SensorBuffer::syncPackages(MeasureGroup & measures, double & lidar_end_time)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (lidar_buffer_.empty() || imu_buffer_.empty())
    {
        return false;
    }
    if (!lidar_pushed_)
    {
        measures.lidar = lidar_buffer_.front();
        measures.lidar_beg_time = time_buffer_.front();
        if (measures.lidar->points.size() <= 1)
        {
            lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime_;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (measures.lidar->points.back().curvature / 1000.0 < 0.5 * lidar_mean_scantime_)
        {
            lidar_end_time = measures.lidar_beg_time + lidar_mean_scantime_;
        }
        else
        {
            ++scan_num_;
            const double scan_time = measures.lidar->points.back().curvature / 1000.0;
            lidar_end_time = measures.lidar_beg_time + scan_time;
            lidar_mean_scantime_ += (scan_time - lidar_mean_scantime_) / scan_num_;
        }
        measures.lidar_end_time = lidar_end_time;
        lidar_pushed_ = true;
    }
    if (last_timestamp_imu_ < lidar_end_time)
    {
        return false;
    }
    measures.imu.clear();
    while (!imu_buffer_.empty() && get_time_sec(imu_buffer_.front()->header.stamp) < lidar_end_time)
    {
        const double imu_time = get_time_sec(imu_buffer_.front()->header.stamp);
        if (imu_time > lidar_end_time)
        {
            break;
        }
        measures.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void SensorBuffer::waitForData(const std::atomic_bool & stop_requested)
{
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &stop_requested]() {
        return stop_requested.load() || !rclcpp::ok() ||
               (!lidar_buffer_.empty() && !imu_buffer_.empty());
    });
}

void SensorBuffer::notifyStop()
{
    condition_.notify_all();
}

int SensorBuffer::scanCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return scan_count_;
}

double SensorBuffer::lastPreprocessDuration() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_preprocess_duration_;
}
