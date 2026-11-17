#include "mapping_output.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
constexpr int kPublishFramePeriod = 20;
}

void MappingOutput::configure(const Config & config)
{
    config_ = config;
}

void MappingOutput::transformToWorld(
    const PointType & input, PointType & output, const state_ikfom & state)
{
    const V3D body(input.x, input.y, input.z);
    const V3D world =
        state.rot * (state.offset_R_L_I * body + state.offset_T_L_I) + state.pos;
    output.x = world.x();
    output.y = world.y();
    output.z = world.z();
    output.intensity = input.intensity;
}

void MappingOutput::transformLidarToImu(
    const PointType & input, PointType & output, const state_ikfom & state)
{
    const V3D lidar(input.x, input.y, input.z);
    const V3D imu = state.offset_R_L_I * lidar + state.offset_T_L_I;
    output.x = imu.x();
    output.y = imu.y();
    output.z = imu.z();
    output.intensity = input.intensity;
}

void MappingOutput::publishFrameWorld(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI::Ptr & undistorted,
    const PointCloudXYZI::Ptr & downsampled_body,
    const state_ikfom & state, double stamp_sec, int & publish_count)
{
    const PointCloudXYZI::Ptr source =
        config_.dense_publish_enabled ? undistorted : downsampled_body;
    PointCloudXYZI::Ptr world(new PointCloudXYZI(source->size(), 1));
    for (std::size_t i = 0; i < source->size(); ++i)
    {
        transformToWorld(source->points[i], world->points[i], state);
    }

    if (config_.scan_publish_enabled)
    {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*world, message);
        message.header.stamp = get_ros_time(stamp_sec);
        message.header.frame_id = config_.world_frame;
        publisher->publish(message);
        publish_count -= kPublishFramePeriod;
    }

    if (config_.pcd_save_enabled)
    {
        // Preserve the original behavior: saved scans always use full density.
        PointCloudXYZI::Ptr full_world(new PointCloudXYZI(undistorted->size(), 1));
        for (std::size_t i = 0; i < undistorted->size(); ++i)
        {
            transformToWorld(undistorted->points[i], full_world->points[i], state);
        }
        *accumulated_scans_ += *full_world;
    }
}

void MappingOutput::publishFrameBody(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI::Ptr & undistorted,
    const state_ikfom & state, double stamp_sec, int & publish_count)
{
    PointCloudXYZI::Ptr imu_body(new PointCloudXYZI(undistorted->size(), 1));
    for (std::size_t i = 0; i < undistorted->size(); ++i)
    {
        transformLidarToImu(undistorted->points[i], imu_body->points[i], state);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*imu_body, message);
    message.header.stamp = get_ros_time(stamp_sec);
    message.header.frame_id = config_.body_frame;
    publisher->publish(message);
    publish_count -= kPublishFramePeriod;
}

void MappingOutput::publishEffectWorld(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI::Ptr & effect_points, int effect_point_count,
    const state_ikfom & state, double stamp_sec)
{
    PointCloudXYZI::Ptr world(new PointCloudXYZI(effect_point_count, 1));
    for (int i = 0; i < effect_point_count; ++i)
    {
        transformToWorld(effect_points->points[i], world->points[i], state);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*world, message);
    message.header.stamp = get_ros_time(stamp_sec);
    message.header.frame_id = config_.world_frame;
    publisher->publish(message);
}

void MappingOutput::publishMap(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const PointCloudXYZI::Ptr & undistorted,
    const PointCloudXYZI::Ptr & downsampled_body,
    const state_ikfom & state, double stamp_sec)
{
    const PointCloudXYZI::Ptr source =
        config_.dense_publish_enabled ? undistorted : downsampled_body;
    PointCloudXYZI::Ptr world(new PointCloudXYZI(source->size(), 1));
    for (std::size_t i = 0; i < source->size(); ++i)
    {
        transformToWorld(source->points[i], world->points[i], state);
    }
    *published_map_ += *world;

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*published_map_, message);
    message.header.stamp = get_ros_time(stamp_sec);
    message.header.frame_id = config_.world_frame;
    publisher->publish(message);
}

void MappingOutput::setPose(
    geometry_msgs::msg::Pose & pose, const state_ikfom & state,
    const geometry_msgs::msg::Quaternion & orientation)
{
    pose.position.x = state.pos(0);
    pose.position.y = state.pos(1);
    pose.position.z = state.pos(2);
    pose.orientation = orientation;
}

void MappingOutput::publishOdometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher,
    std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_broadcaster,
    const Filter & filter, const state_ikfom & state,
    const geometry_msgs::msg::Quaternion & orientation, double stamp_sec)
{
    odometry_.header.frame_id = config_.world_frame;
    odometry_.child_frame_id = config_.body_frame;
    odometry_.header.stamp = get_ros_time(stamp_sec);
    setPose(odometry_.pose.pose, state, orientation);
    const auto covariance = filter.get_P();
    for (int i = 0; i < 6; ++i)
    {
        const int k = i < 3 ? i + 3 : i - 3;
        for (int j = 0; j < 6; ++j)
        {
            odometry_.pose.covariance[i * 6 + j] = covariance(k, (j + 3) % 6);
        }
    }
    publisher->publish(odometry_);

    if (!config_.odom_tf_enabled)
    {
        return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry_.header;
    transform.child_frame_id = config_.body_frame;
    transform.transform.translation.x = odometry_.pose.pose.position.x;
    transform.transform.translation.y = odometry_.pose.pose.position.y;
    transform.transform.translation.z = odometry_.pose.pose.position.z;
    transform.transform.rotation = odometry_.pose.pose.orientation;
    tf_broadcaster->sendTransform(transform);
}

void MappingOutput::publishPath(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    nav_msgs::msg::Path & path, const state_ikfom & state,
    const geometry_msgs::msg::Quaternion & orientation, double stamp_sec)
{
    setPose(body_pose_.pose, state, orientation);
    body_pose_.header.stamp = get_ros_time(stamp_sec);
    body_pose_.header.frame_id = config_.world_frame;
    if (++path_publish_counter_ % 10 == 0)
    {
        path.poses.push_back(body_pose_);
        publisher->publish(path);
    }
}

bool MappingOutput::saveMap() const
{
    return pcl::PCDWriter().writeBinary(config_.map_file_path, *published_map_) == 0;
}

bool MappingOutput::saveAccumulatedScans() const
{
    if (accumulated_scans_->empty())
    {
        return false;
    }
    return pcl::PCDWriter().writeBinary(config_.map_file_path, *accumulated_scans_) == 0;
}
