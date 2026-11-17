#pragma once

#include <memory>
#include <string>
#include <deque>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common_lib.h"
#include "use-ikfom.hpp"

class MappingOutput
{
public:
    using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

    struct Config
    {
        bool scan_publish_enabled = false;
        bool dense_publish_enabled = false;
        bool odom_tf_enabled = true;
        bool pcd_save_enabled = false;
        std::string world_frame = "camera_init";
        std::string body_frame = "body";
        std::string map_file_path;
    };

    void configure(const Config & config);

    void publishFrameWorld(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI::Ptr & undistorted,
        const PointCloudXYZI::Ptr & downsampled_body,
        const state_ikfom & state, double stamp_sec, int & publish_count);
    void publishFrameBody(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI::Ptr & undistorted,
        const state_ikfom & state, double stamp_sec, int & publish_count);
    void publishEffectWorld(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI::Ptr & effect_points, int effect_point_count,
        const state_ikfom & state, double stamp_sec);
    void publishMap(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
        const PointCloudXYZI::Ptr & undistorted,
        const PointCloudXYZI::Ptr & downsampled_body,
        const state_ikfom & state, double stamp_sec);
    void publishOdometry(
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher,
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_broadcaster,
        const Filter & filter, const state_ikfom & state,
        const geometry_msgs::msg::Quaternion & orientation, double stamp_sec);
    void publishPath(
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
        nav_msgs::msg::Path & path, const state_ikfom & state,
        const geometry_msgs::msg::Quaternion & orientation, double stamp_sec);

    bool saveMap() const;
    bool saveAccumulatedScans() const;

private:
    static void transformToWorld(
        const PointType & input, PointType & output, const state_ikfom & state);
    static void transformLidarToImu(
        const PointType & input, PointType & output, const state_ikfom & state);
    static void setPose(
        geometry_msgs::msg::Pose & pose, const state_ikfom & state,
        const geometry_msgs::msg::Quaternion & orientation);

    Config config_;
    PointCloudXYZI::Ptr published_map_{new PointCloudXYZI()};
    PointCloudXYZI::Ptr accumulated_scans_{new PointCloudXYZI()};
    nav_msgs::msg::Odometry odometry_;
    geometry_msgs::msg::PoseStamped body_pose_;
    int path_publish_counter_ = 0;
};
