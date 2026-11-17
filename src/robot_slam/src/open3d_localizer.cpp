#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <open3d/Open3D.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "robot_interfaces/msg/localization_status.hpp"
#include "robot_interfaces/srv/load_pcd_map.hpp"
#include "robot_interfaces/srv/set_localization_pose.hpp"
#include "dog_slam/localization_pose_cache.hpp"

namespace
{
using PclCloud = pcl::PointCloud<pcl::PointXYZI>;
using Open3dCloud = open3d::geometry::PointCloud;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

struct MapTileKey
{
  std::int32_t x;
  std::int32_t y;

  bool operator==(const MapTileKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct MapTileKeyHash
{
  std::size_t operator()(const MapTileKey & key) const
  {
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

using MapTileMap = std::unordered_map<MapTileKey, std::shared_ptr<Open3dCloud>, MapTileKeyHash>;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  rotation = rotation.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : rotation.normalized();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  rotation = rotation.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : rotation.normalized();
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.linear() = rotation.toRotationMatrix();
  output.translation() = Eigen::Vector3d(
    transform.translation.x, transform.translation.y, transform.translation.z);
  return output;
}

geometry_msgs::msg::Transform eigenToTransform(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform message;
  const Eigen::Quaterniond rotation(transform.linear());
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  message.rotation.x = rotation.x();
  message.rotation.y = rotation.y();
  message.rotation.z = rotation.z();
  message.rotation.w = rotation.w();
  return message;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose message;
  const Eigen::Quaterniond rotation(transform.linear());
  message.position.x = transform.translation().x();
  message.position.y = transform.translation().y();
  message.position.z = transform.translation().z();
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

bool transformIsSane(const Eigen::Isometry3d & transform, double max_translation)
{
  return transform.matrix().allFinite() &&
         transform.translation().norm() <= std::max(1.0, max_translation);
}

Eigen::Isometry3d interpolate(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double alpha)
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.translation() = (1.0 - alpha) * from.translation() + alpha * to.translation();
  const Eigen::Quaterniond from_rotation(from.linear());
  const Eigen::Quaterniond to_rotation(to.linear());
  output.linear() = from_rotation.slerp(alpha, to_rotation).normalized().toRotationMatrix();
  return output;
}

Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw)
{
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Isometry3d planarPose(const Eigen::Isometry3d & pose)
{
  const Eigen::Vector3d forward = pose.linear() * Eigen::Vector3d::UnitX();
  const double yaw = std::atan2(forward.y(), forward.x());
  Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
  planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  planar.translation().x() = pose.translation().x();
  planar.translation().y() = pose.translation().y();
  return planar;
}

Eigen::Isometry3d stepToward(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to,
  double max_translation_step, double max_rotation_step)
{
  const Eigen::Isometry3d delta = from.inverse() * to;
  const double translation_distance = delta.translation().norm();
  const double rotation_distance = rotationAngle(delta.linear());
  double alpha = 1.0;
  if (max_translation_step > 0.0 && translation_distance > max_translation_step) {
    alpha = std::min(alpha, max_translation_step / translation_distance);
  }
  if (max_rotation_step > 0.0 && rotation_distance > max_rotation_step) {
    alpha = std::min(alpha, max_rotation_step / rotation_distance);
  }
  return interpolate(from, to, std::clamp(alpha, 0.0, 1.0));
}
}  // namespace

class Open3dLocalizer final : public rclcpp::Node
{
public:
  Open3dLocalizer()
  : Node("open3d_localizer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    fastlio_odom_topic_ = declare_parameter<std::string>(
      "fastlio_odom_topic", "/fastlio/base_odom");
    dog_odom_topic_ = declare_parameter<std::string>("dog_odom_topic", "/odom");
    initial_map_id_ = declare_parameter<std::string>("initial_map_id", "");
    initial_map_path_ = declare_parameter<std::string>("initial_map_path", "");
    initial_visualization_map_path_ = declare_parameter<std::string>(
      "initial_visualization_map_path", "");
    layer_base_z_ = declare_parameter<double>("layer_base_z", 0.0);
    layer_z_tolerance_ = declare_parameter<double>("layer_z_tolerance", 0.0);
    auto_initialize_from_origin_ = declare_parameter<bool>(
      "auto_initialize_from_origin", true);
    persist_last_pose_ = declare_parameter<bool>("persist_last_pose", true);
    restore_last_pose_ = declare_parameter<bool>("restore_last_pose", true);
    pose_cache_path_ = declare_parameter<std::string>(
      "pose_cache_path", "~/.local/share/dog/last_localization_pose.txt");
    if (pose_cache_path_.rfind("~/", 0) == 0) {
      const char * home = std::getenv("HOME");
      if (home == nullptr) {
        throw std::invalid_argument("HOME is required to expand pose_cache_path");
      }
      pose_cache_path_ = std::string(home) + pose_cache_path_.substr(1);
    }
    auto_initial_x_ = declare_parameter<double>("auto_initial_x", 0.0);
    auto_initial_y_ = declare_parameter<double>("auto_initial_y", 0.0);
    auto_initial_z_ = declare_parameter<double>("auto_initial_z", 0.0);
    auto_initial_roll_ = declare_parameter<double>("auto_initial_roll", 0.0);
    auto_initial_pitch_ = declare_parameter<double>("auto_initial_pitch", 0.0);
    auto_initial_yaw_ = declare_parameter<double>("auto_initial_yaw", 0.0);
    planar_navigation_tf_ = declare_parameter<bool>("planar_navigation_tf", true);
    window_duration_sec_ = declare_parameter<double>("window_duration_sec", 1.2);
    dog_odom_buffer_duration_sec_ = declare_parameter<double>(
      "dog_odom_buffer_duration_sec", 5.0);
    max_dog_odom_interpolation_gap_sec_ = declare_parameter<double>(
      "max_dog_odom_interpolation_gap_sec", 0.20);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 0.50);
    registration_rate_hz_ = declare_parameter<double>("registration_rate_hz", 1.0);
    tracking_fine_only_ = declare_parameter<bool>("tracking_fine_only", true);
    tf_rate_hz_ = declare_parameter<double>("tf_rate_hz", 30.0);
    coarse_voxel_size_ = declare_parameter<double>("coarse_voxel_size", 0.45);
    fine_voxel_size_ = declare_parameter<double>("fine_voxel_size", 0.20);
    coarse_max_correspondence_ = declare_parameter<double>(
      "coarse_max_correspondence", 1.5);
    fine_max_correspondence_ = declare_parameter<double>(
      "fine_max_correspondence", 0.60);
    normal_radius_ = declare_parameter<double>("normal_radius", 0.50);
    max_iterations_coarse_ = declare_parameter<int>("max_iterations_coarse", 30);
    max_iterations_fine_ = declare_parameter<int>("max_iterations_fine", 20);
    min_source_points_ = declare_parameter<int>("min_source_points", 250);
    min_inliers_ = declare_parameter<int>("min_inliers", 150);
    min_fitness_ = declare_parameter<double>("min_fitness", 0.30);
    max_inlier_rmse_ = declare_parameter<double>("max_inlier_rmse", 0.45);
    max_tracking_translation_correction_ = declare_parameter<double>(
      "max_tracking_translation_correction", 0.60);
    max_tracking_rotation_correction_rad_ = declare_parameter<double>(
      "max_tracking_rotation_correction_rad", 0.25);
    max_localizing_translation_correction_ = declare_parameter<double>(
      "max_localizing_translation_correction", 2.5);
    max_localizing_rotation_correction_rad_ = declare_parameter<double>(
      "max_localizing_rotation_correction_rad", 0.80);
    local_map_radius_xy_ = declare_parameter<double>("local_map_radius_xy", 7.0);
    local_map_half_height_ = declare_parameter<double>("local_map_half_height", 3.0);
    map_tile_size_ = std::max(1.0, declare_parameter<double>("map_tile_size", 4.0));
    max_condition_number_ = declare_parameter<double>("max_condition_number", 1.0e6);
    min_information_eigenvalue_ = declare_parameter<double>(
      "min_information_eigenvalue", 1.0e-4);
    robust_kernel_scale_ = declare_parameter<double>("robust_kernel_scale", 0.35);
    immediate_tracking_translation_ = declare_parameter<double>(
      "immediate_tracking_translation", 0.10);
    immediate_tracking_rotation_rad_ = declare_parameter<double>(
      "immediate_tracking_rotation_rad", 0.05);
    correction_confirmation_translation_ = declare_parameter<double>(
      "correction_confirmation_translation", 0.08);
    correction_confirmation_rotation_rad_ = declare_parameter<double>(
      "correction_confirmation_rotation_rad", 0.04);
    large_correction_confirmations_ = declare_parameter<int>(
      "large_correction_confirmations", 3);
    override_initial_base_z_ = declare_parameter<bool>("override_initial_base_z", false);
    initial_base_z_ = declare_parameter<double>("initial_base_z", 0.0);
    successes_to_track_ = declare_parameter<int>("successes_to_track", 2);
    failures_to_lose_ = declare_parameter<int>("failures_to_lose", 5);
    tracking_smoothing_alpha_ = declare_parameter<double>("tracking_smoothing_alpha", 0.35);
    max_tf_translation_rate_mps_ = declare_parameter<double>(
      "max_tf_translation_rate_mps", 0.25);
    max_tf_rotation_rate_radps_ = declare_parameter<double>(
      "max_tf_rotation_rate_radps", 0.20);
    max_reasonable_translation_ = declare_parameter<double>(
      "max_reasonable_translation", 1000.0);
    max_fastlio_pose_jump_translation_ = declare_parameter<double>(
      "max_fastlio_pose_jump_translation", 2.0);
    max_fastlio_pose_jump_rotation_rad_ = declare_parameter<double>(
      "max_fastlio_pose_jump_rotation_rad", 0.50);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &Open3dLocalizer::updateDynamicParameters, this,
        std::placeholders::_1));

    auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    status_publisher_ = create_publisher<robot_interfaces::msg::LocalizationStatus>(
      "/localization/status", transient_qos);
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/cloud_map", transient_qos);
    aligned_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/aligned_cloud", rclcpp::QoS(1));
    visualization_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_3d", rclcpp::SensorDataQoS().keep_last(1));

    cloud_subscriber_.subscribe(this, cloud_topic_, rmw_qos_profile_sensor_data);
    fastlio_odom_subscriber_.subscribe(
      this, fastlio_odom_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(30), cloud_subscriber_, fastlio_odom_subscriber_);
    synchronizer_->registerCallback(
      std::bind(
        &Open3dLocalizer::fastlioCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    fastlio_pose_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      fastlio_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Open3dLocalizer::fastlioPoseCallback, this, std::placeholders::_1));
    dog_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      dog_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Open3dLocalizer::dogOdomCallback, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      std::bind(&Open3dLocalizer::initialPoseCallback, this, std::placeholders::_1));
    load_map_service_ = create_service<robot_interfaces::srv::LoadPcdMap>(
      "/localization/load_pcd_map",
      std::bind(
        &Open3dLocalizer::loadMapCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    initial_pose_service_ = create_service<robot_interfaces::srv::SetLocalizationPose>(
      "/localization/set_initial_pose",
      std::bind(
        &Open3dLocalizer::setInitialPoseCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    registration_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(0.1, registration_rate_hz_))),
      std::bind(&Open3dLocalizer::queueRegistration, this));
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate_hz_))),
      std::bind(&Open3dLocalizer::publishTransform, this));
    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Open3dLocalizer::publishStatus, this));
    pose_cache_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Open3dLocalizer::persistLastPose, this));

    worker_ = std::thread(&Open3dLocalizer::registrationWorker, this);
    if (!initial_map_path_.empty()) {
      std::string message;
      std::uint64_t point_count = 0;
      if (!loadMap(
          initial_map_id_, initial_map_path_, initial_visualization_map_path_,
          message, point_count))
      {
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      }
    }
    publishStatus();
  }

  ~Open3dLocalizer() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_worker_ = true;
    }
    work_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    persistLastPose();
  }

private:
  void persistLastPose()
  {
    if (!persist_last_pose_ || pose_cache_path_.empty()) {
      return;
    }
    std::optional<dog_slam::CachedLocalizationPose> pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = last_trusted_pose_;
    }
    if (!pose) {
      return;
    }
    try {
      dog_slam::writeLocalizationPose(pose_cache_path_, *pose);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "Cannot save last localization pose: %s",
        error.what());
    }
  }

  rcl_interfaces::msg::SetParametersResult updateDynamicParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    double next_min_fitness = min_fitness_;
    double next_max_inlier_rmse = max_inlier_rmse_;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    try {
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "min_fitness") {
          next_min_fitness = parameter.as_double();
        } else if (parameter.get_name() == "max_inlier_rmse") {
          next_max_inlier_rmse = parameter.as_double();
        }
      }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & error) {
      result.reason = error.what();
      return result;
    }
    if (next_min_fitness < 0.10 || next_min_fitness > 0.90) {
      result.reason = "min_fitness must be between 0.10 and 0.90";
      return result;
    }
    if (next_max_inlier_rmse < 0.10 || next_max_inlier_rmse > 1.00) {
      result.reason = "max_inlier_rmse must be between 0.10 and 1.00";
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      min_fitness_ = next_min_fitness;
      max_inlier_rmse_ = next_max_inlier_rmse;
    }
    result.successful = true;
    return result;
  }

  struct WindowFrame
  {
    std::int64_t stamp_nanoseconds;
    std::size_t point_count = 0;
  };

  struct DogOdomSample
  {
    std::int64_t stamp_nanoseconds;
    Eigen::Isometry3d odom_from_base = Eigen::Isometry3d::Identity();
  };

  struct RegistrationWork
  {
    std::shared_ptr<const std::vector<Eigen::Vector3d>> source_points;
    std::size_t source_begin = 0;
    std::size_t source_end = 0;
    Eigen::Isometry3d initial_map_from_fastlio = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d fastlio_from_base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d dog_odom_from_base = Eigen::Isometry3d::Identity();
    std::shared_ptr<const MapTileMap> coarse_map_tiles;
    std::shared_ptr<const MapTileMap> fine_map_tiles;
    bool was_tracking = false;
    bool was_lost = false;
    std::uint64_t generation = 0;
    std::uint64_t pose_epoch = 0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct RegistrationMetrics
  {
    double condition_number = std::numeric_limits<double>::infinity();
    double minimum_eigenvalue = 0.0;
  };

  std::shared_ptr<MapTileMap> buildMapTiles(const Open3dCloud & cloud) const
  {
    auto tiles = std::make_shared<MapTileMap>();
    const bool has_normals = cloud.normals_.size() == cloud.points_.size();
    for (std::size_t index = 0; index < cloud.points_.size(); ++index) {
      const auto & point = cloud.points_[index];
      const MapTileKey key{
        static_cast<std::int32_t>(std::floor(point.x() / map_tile_size_)),
        static_cast<std::int32_t>(std::floor(point.y() / map_tile_size_))};
      auto & tile = (*tiles)[key];
      if (!tile) {
        tile = std::make_shared<Open3dCloud>();
      }
      tile->points_.push_back(point);
      if (has_normals) {
        tile->normals_.push_back(cloud.normals_[index]);
      }
    }
    return tiles;
  }

  std::shared_ptr<Open3dCloud> localMapFromTiles(
    const MapTileMap & tiles, const Eigen::Vector3d & center,
    const Eigen::Vector3d & extent) const
  {
    auto local = std::make_shared<Open3dCloud>();
    const auto min_x = static_cast<std::int32_t>(
      std::floor((center.x() - extent.x()) / map_tile_size_));
    const auto max_x = static_cast<std::int32_t>(
      std::floor((center.x() + extent.x()) / map_tile_size_));
    const auto min_y = static_cast<std::int32_t>(
      std::floor((center.y() - extent.y()) / map_tile_size_));
    const auto max_y = static_cast<std::int32_t>(
      std::floor((center.y() + extent.y()) / map_tile_size_));
    for (std::int32_t tile_y = min_y; tile_y <= max_y; ++tile_y) {
      for (std::int32_t tile_x = min_x; tile_x <= max_x; ++tile_x) {
        const auto iterator = tiles.find(MapTileKey{tile_x, tile_y});
        if (iterator == tiles.end() || !iterator->second) {
          continue;
        }
        const auto & tile = *iterator->second;
        const bool has_normals = tile.normals_.size() == tile.points_.size();
        for (std::size_t index = 0; index < tile.points_.size(); ++index) {
          const auto & point = tile.points_[index];
          if ((point.array() < (center - extent).array()).any() ||
            (point.array() > (center + extent).array()).any())
          {
            continue;
          }
          local->points_.push_back(point);
          if (has_normals) {
            local->normals_.push_back(tile.normals_[index]);
          }
        }
      }
    }
    return local;
  }

  void fastlioCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_message,
    const nav_msgs::msg::Odometry::ConstSharedPtr & fastlio_message)
  {
    const Eigen::Isometry3d fastlio_from_child = poseToEigen(fastlio_message->pose.pose);
    Eigen::Isometry3d child_from_cloud = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d child_from_base = Eigen::Isometry3d::Identity();
    try {
      if (fastlio_message->child_frame_id != cloud_message->header.frame_id) {
        child_from_cloud = transformToEigen(
          tf_buffer_.lookupTransform(
            fastlio_message->child_frame_id, cloud_message->header.frame_id,
            tf2::TimePointZero).transform);
      }
      child_from_base = transformToEigen(
        tf_buffer_.lookupTransform(
          fastlio_message->child_frame_id, base_frame_, tf2::TimePointZero).transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot resolve FAST-LIO/body frames: %s",
        error.what());
      return;
    }
    const Eigen::Isometry3d fastlio_from_cloud = fastlio_from_child * child_from_cloud;
    const Eigen::Isometry3d fastlio_from_base = fastlio_from_child * child_from_base;

    PclCloud cloud;
    pcl::fromROSMsg(*cloud_message, cloud);
    WindowFrame frame;
    frame.stamp_nanoseconds = rclcpp::Time(cloud_message->header.stamp).nanoseconds();
    std::vector<Eigen::Vector3d> frame_points;
    frame_points.reserve(cloud.size());
    for (const auto & point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        frame_points.emplace_back(
          fastlio_from_cloud * Eigen::Vector3d(point.x, point.y, point.z));
      }
    }

    const auto window_nanoseconds = static_cast<std::int64_t>(window_duration_sec_ * 1e9);
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time synchronized_stamp(fastlio_message->header.stamp);
    if (minimum_cloud_stamp_after_jump_.nanoseconds() != 0 &&
      synchronized_stamp < minimum_cloud_stamp_after_jump_)
    {
      return;
    }
    if (pending_fastlio_rebase_) {
      map_from_fastlio_ = map_from_fastlio_ * pending_fastlio_rebase_transform_;
      pending_fastlio_rebase_ = false;
      pending_fastlio_rebase_transform_ = Eigen::Isometry3d::Identity();
    }
    latest_fastlio_from_base_ = fastlio_from_base;
    latest_fastlio_stamp_ = synchronized_stamp;
    latest_fastlio_received_at_ = std::chrono::steady_clock::now();
    have_fastlio_ = true;
    ++cloud_generation_;
    if (!window_points_.unique()) {
      window_points_ = std::make_shared<std::vector<Eigen::Vector3d>>(
        window_points_->begin() + static_cast<std::ptrdiff_t>(window_points_begin_),
        window_points_->end());
      window_points_begin_ = 0;
    }
    window_points_->insert(window_points_->end(), frame_points.begin(), frame_points.end());
    frame.point_count = frame_points.size();
    window_.push_back(std::move(frame));
    const auto oldest_allowed = window_.back().stamp_nanoseconds - window_nanoseconds;
    while (!window_.empty() && window_.front().stamp_nanoseconds < oldest_allowed) {
      window_points_begin_ += window_.front().point_count;
      window_.pop_front();
    }
    if (window_points_begin_ > 0 &&
      (window_points_begin_ >= window_points_->size() / 2U || window_points_begin_ > 100000U))
    {
      window_points_->erase(
        window_points_->begin(),
        window_points_->begin() + static_cast<std::ptrdiff_t>(window_points_begin_));
      window_points_begin_ = 0;
    }
  }

  void fastlioPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & message)
  {
    Eigen::Isometry3d child_from_base = Eigen::Isometry3d::Identity();
    try {
      child_from_base = transformToEigen(
        tf_buffer_.lookupTransform(
          message->child_frame_id, base_frame_, tf2::TimePointZero).transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot resolve FAST-LIO/body frames: %s",
        error.what());
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    const Eigen::Isometry3d fastlio_from_base =
      poseToEigen(message->pose.pose) * child_from_base;
    bool pose_jump = false;
    double jump_translation = 0.0;
    double jump_rotation = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_continuity_pose_ && stamp <= latest_continuity_stamp_) {
        return;
      }
      if (have_continuity_pose_) {
        const Eigen::Isometry3d delta =
          latest_continuity_fastlio_from_base_.inverse() * fastlio_from_base;
        jump_translation = delta.translation().norm();
        jump_rotation = rotationAngle(delta.linear());
        pose_jump =
          jump_translation > std::max(0.1, max_fastlio_pose_jump_translation_) ||
          jump_rotation > std::max(0.05, max_fastlio_pose_jump_rotation_rad_);
        if (pose_jump) {
          const Eigen::Isometry3d rebase =
            latest_continuity_fastlio_from_base_ * fastlio_from_base.inverse();
          pending_fastlio_rebase_transform_ = pending_fastlio_rebase_ ?
            pending_fastlio_rebase_transform_ * rebase : rebase;
          pending_fastlio_rebase_ = true;
          window_.clear();
          window_points_ = std::make_shared<std::vector<Eigen::Vector3d>>();
          window_points_begin_ = 0;
          pending_candidate_valid_ = false;
          pending_candidate_confirmations_ = 0;
          consecutive_failures_ = 0;
          minimum_cloud_stamp_after_jump_ = stamp;
          ++pose_epoch_;
          status_message_ = "FAST-LIO pose discontinuity rebased; rebuilding ICP window";
        }
      }
      latest_continuity_fastlio_from_base_ = fastlio_from_base;
      latest_continuity_stamp_ = stamp;
      have_continuity_pose_ = true;
    }
    if (pose_jump) {
      RCLCPP_ERROR(
        get_logger(),
        "FAST-LIO pose discontinuity detected (translation=%.3f m, rotation=%.3f rad); "
        "preserving the last map pose and resetting the ICP window",
        jump_translation, jump_rotation);
    }
  }

  void dogOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & message)
  {
    const auto stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    DogOdomSample sample;
    sample.stamp_nanoseconds = stamp_nanoseconds;
    sample.odom_from_base = poseToEigen(message->pose.pose);

    std::lock_guard<std::mutex> lock(mutex_);
    const auto position = std::lower_bound(
      dog_odom_buffer_.begin(), dog_odom_buffer_.end(), stamp_nanoseconds,
      [](const DogOdomSample & existing, std::int64_t stamp) {
        return existing.stamp_nanoseconds < stamp;
      });
    if (position != dog_odom_buffer_.end() &&
      position->stamp_nanoseconds == stamp_nanoseconds)
    {
      *position = std::move(sample);
    } else {
      dog_odom_buffer_.insert(position, std::move(sample));
    }

    const auto buffer_duration_nanoseconds = static_cast<std::int64_t>(
      std::max(0.1, dog_odom_buffer_duration_sec_) * 1e9);
    const auto oldest_allowed = dog_odom_buffer_.back().stamp_nanoseconds -
      buffer_duration_nanoseconds;
    while (!dog_odom_buffer_.empty() &&
      dog_odom_buffer_.front().stamp_nanoseconds < oldest_allowed)
    {
      dog_odom_buffer_.pop_front();
    }
  }

  bool interpolateDogOdom(
    std::int64_t stamp_nanoseconds, Eigen::Isometry3d & odom_from_base) const
  {
    if (dog_odom_buffer_.empty()) {
      return false;
    }
    const auto upper = std::lower_bound(
      dog_odom_buffer_.begin(), dog_odom_buffer_.end(), stamp_nanoseconds,
      [](const DogOdomSample & sample, std::int64_t stamp) {
        return sample.stamp_nanoseconds < stamp;
      });
    if (upper != dog_odom_buffer_.end() && upper->stamp_nanoseconds == stamp_nanoseconds) {
      odom_from_base = upper->odom_from_base;
      return true;
    }
    if (upper == dog_odom_buffer_.begin() || upper == dog_odom_buffer_.end()) {
      return false;
    }

    const auto lower = std::prev(upper);
    const auto gap_nanoseconds = upper->stamp_nanoseconds - lower->stamp_nanoseconds;
    const auto max_gap_nanoseconds = static_cast<std::int64_t>(
      std::max(0.0, max_dog_odom_interpolation_gap_sec_) * 1e9);
    if (gap_nanoseconds <= 0 || gap_nanoseconds > max_gap_nanoseconds) {
      return false;
    }
    const double alpha = static_cast<double>(stamp_nanoseconds - lower->stamp_nanoseconds) /
      static_cast<double>(gap_nanoseconds);
    odom_from_base = interpolate(lower->odom_from_base, upper->odom_from_base, alpha);
    return true;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & message)
  {
    std::string result_message;
    const bool accepted = acceptInitialPose(*message, result_message);
    if (!accepted) {
      RCLCPP_WARN(get_logger(), "%s", result_message.c_str());
    }
    publishStatus();
    if (accepted) {
      publishTransform();
    }
  }

  void setInitialPoseCallback(
    const std::shared_ptr<robot_interfaces::srv::SetLocalizationPose::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetLocalizationPose::Response> response)
  {
    response->success = acceptInitialPose(request->pose, response->message);
    publishStatus();
    if (response->success) {
      publishTransform();
    }
  }

  bool acceptInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message, std::string & result_message)
  {
    if (!message.header.frame_id.empty() && message.header.frame_id != map_frame_) {
      result_message = "initial pose rejected: frame must be " + map_frame_;
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!map_loaded_) {
        status_message_ = "initial pose rejected: no localization map is loaded";
        result_message = status_message_;
        return false;
      }
      if (!have_fastlio_) {
        status_message_ = "initial pose rejected: waiting for FAST-LIO data";
        result_message = status_message_;
        return false;
      }
      const double receive_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_fastlio_received_at_).count();
      if (receive_age > max_cloud_age_sec_) {
        status_message_ = "initial pose rejected: FAST-LIO data stream is stale";
        result_message = status_message_;
        return false;
      }
      const double timestamp_age = (now() - latest_fastlio_stamp_).seconds();
      if (std::abs(timestamp_age) > max_cloud_age_sec_) {
        status_message_ = "initial pose rejected: FAST-LIO timestamp is not synchronized";
        result_message = status_message_;
        return false;
      }
      if (dog_odom_buffer_.empty()) {
        status_message_ = "initial pose rejected: waiting for navigation odometry";
        result_message = status_message_;
        return false;
      }
      Eigen::Isometry3d dog_odom_from_base = Eigen::Isometry3d::Identity();
      if (!interpolateDogOdom(
          latest_fastlio_stamp_.nanoseconds(), dog_odom_from_base))
      {
        status_message_ = "initial pose rejected: navigation odometry is not time-aligned";
        result_message = status_message_;
        return false;
      }
      Eigen::Isometry3d map_from_base = poseToEigen(message.pose.pose);
      if (!transformIsSane(map_from_base, max_reasonable_translation_)) {
        status_message_ = "initial pose rejected: position is outside reasonable map range";
        result_message = status_message_;
        return false;
      }
      if (override_initial_base_z_) {
        map_from_base.translation().z() = initial_base_z_;
      }
      map_from_fastlio_ = map_from_base * latest_fastlio_from_base_.inverse();
      pose_valid_ = false;
      const Eigen::Isometry3d localized_map_from_base =
        map_from_fastlio_ * latest_fastlio_from_base_;
      map_from_odom_ = planar_navigation_tf_ ?
        planarPose(localized_map_from_base) * planarPose(dog_odom_from_base).inverse() :
        localized_map_from_base * dog_odom_from_base.inverse();
      if (!transformIsSane(map_from_odom_, max_reasonable_translation_)) {
        status_message_ = "initial pose rejected: computed map-to-odom transform is invalid";
        result_message = status_message_;
        return false;
      }
      output_map_from_odom_ = map_from_odom_;
      output_transform_initialized_ = true;
      last_tf_update_time_ = now();
      state_ = robot_interfaces::msg::LocalizationStatus::LOCALIZING;
      consecutive_successes_ = 0;
      consecutive_failures_ = 0;
      pending_candidate_valid_ = false;
      pending_candidate_confirmations_ = 0;
      automatic_initialization_attempted_ = true;
      initial_pose_received_ = true;
      ++pose_epoch_;
      status_message_ = "initial pose accepted; waiting for ICP confirmation";
      result_message = status_message_;
    }
    return true;
  }

  void loadMapCallback(
    const std::shared_ptr<robot_interfaces::srv::LoadPcdMap::Request> request,
    std::shared_ptr<robot_interfaces::srv::LoadPcdMap::Response> response)
  {
    response->success = loadMap(
      request->map_id, request->pcd_path, request->visualization_pcd_path,
      response->message, response->point_count);
  }

  bool loadMap(
    const std::string & map_id, const std::string & path,
    const std::string & visualization_path, std::string & message,
    std::uint64_t & point_count)
  {
    if (path.empty() || !std::filesystem::is_regular_file(path)) {
      message = "PCD map does not exist: " + path;
      return false;
    }
    PclCloud pcl_cloud;
    if (pcl::io::loadPCDFile(path, pcl_cloud) < 0) {
      message = "failed to read PCD map: " + path;
      return false;
    }
    auto map = std::make_shared<Open3dCloud>();
    map->points_.reserve(pcl_cloud.size());
    for (const auto & point : pcl_cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        map->points_.emplace_back(point.x, point.y, point.z);
      }
    }
    if (map->points_.size() < static_cast<std::size_t>(min_source_points_)) {
      message = "PCD map has too few finite points";
      return false;
    }
    auto coarse_map = map->VoxelDownSample(coarse_voxel_size_);
    auto fine_map = map->VoxelDownSample(fine_voxel_size_);
    fine_map->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_radius_, 30));
    auto coarse_map_tiles = buildMapTiles(*coarse_map);
    auto fine_map_tiles = buildMapTiles(*fine_map);
    const auto map_path = std::filesystem::weakly_canonical(path).string();
    // Flush the previous map's last trusted pose before selecting a new map.
    persistLastPose();
    std::optional<dog_slam::CachedLocalizationPose> saved_pose;
    if (restore_last_pose_ && !pose_cache_path_.empty()) {
      try {
        if (std::filesystem::exists(pose_cache_path_)) {
          auto cached = dog_slam::readLocalizationPose(pose_cache_path_);
          if (cached.matches(map_id, map_path, map_frame_, base_frame_) &&
            transformIsSane(cached.map_from_base, max_reasonable_translation_))
          {
            saved_pose = std::move(cached);
          }
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "Ignoring localization pose cache: %s", error.what());
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      coarse_map_tiles_ = std::move(coarse_map_tiles);
      fine_map_tiles_ = std::move(fine_map_tiles);
      map_id_ = map_id;
      loaded_map_path_ = map_path;
      startup_pose_ = std::move(saved_pose);
      last_trusted_pose_.reset();
      map_loaded_ = true;
      pose_valid_ = false;
      have_last_valid_transform_ = false;
      output_transform_initialized_ = false;
      state_ = robot_interfaces::msg::LocalizationStatus::UNINITIALIZED;
      consecutive_successes_ = 0;
      consecutive_failures_ = 0;
      pending_candidate_valid_ = false;
      pending_candidate_confirmations_ = 0;
      automatic_initialization_attempted_ = false;
      initial_pose_received_ = false;
      ++pose_epoch_;
      last_queued_generation_ = cloud_generation_;
      status_message_ = startup_pose_ ?
        "map loaded; waiting for sensors to restore last pose" :
        "map loaded; waiting for /initialpose";
      point_count = fine_map->points_.size();
    }
    sensor_msgs::msg::PointCloud2 map_message;
    const std::string display_path =
      !visualization_path.empty() && std::filesystem::is_regular_file(visualization_path) ?
      visualization_path : path;
    pcl::PCLPointCloud2 display_cloud;
    if (pcl::io::loadPCDFile(display_path, display_cloud) >= 0) {
      pcl_conversions::fromPCL(display_cloud, map_message);
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to load visualization map %s; using localization map",
        display_path.c_str());
      pcl::toROSMsg(pcl_cloud, map_message);
    }
    map_message.header.stamp = now();
    map_message.header.frame_id = map_frame_;
    map_publisher_->publish(map_message);
    message = "map loaded";
    RCLCPP_INFO(
      get_logger(), "Loaded Open3D map %s with %lu fine points", map_id.c_str(),
      static_cast<unsigned long>(point_count));
    publishStatus();
    return true;
  }

  void queueRegistration()
  {
    tryAutomaticInitialization();
    RegistrationWork work;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (work_pending_ || worker_busy_ || !map_loaded_ || !have_fastlio_ ||
        dog_odom_buffer_.empty() ||
        state_ == robot_interfaces::msg::LocalizationStatus::UNINITIALIZED ||
        window_.empty() || cloud_generation_ == last_queued_generation_)
      {
        return;
      }
      const double receive_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_fastlio_received_at_).count();
      if (receive_age > max_cloud_age_sec_) {
        status_message_ = "FAST-LIO data stream is stale; registration skipped";
        return;
      }
      const double timestamp_age = (now() - latest_fastlio_stamp_).seconds();
      if (std::abs(timestamp_age) > max_cloud_age_sec_) {
        status_message_ = "FAST-LIO timestamp is not synchronized; registration skipped";
        return;
      }
      const std::size_t point_count = window_points_->size() - window_points_begin_;
      if (point_count < static_cast<std::size_t>(min_source_points_)) {
        status_message_ = "waiting for enough FAST-LIO points";
        return;
      }
      work.source_points = window_points_;
      work.source_begin = window_points_begin_;
      work.source_end = window_points_->size();
      work.initial_map_from_fastlio = map_from_fastlio_;
      work.fastlio_from_base = latest_fastlio_from_base_;
      if (!interpolateDogOdom(
          latest_fastlio_stamp_.nanoseconds(), work.dog_odom_from_base))
      {
        status_message_ = "waiting for time-aligned dog odometry";
        return;
      }
      work.coarse_map_tiles = coarse_map_tiles_;
      work.fine_map_tiles = fine_map_tiles_;
      work.was_tracking = state_ == robot_interfaces::msg::LocalizationStatus::TRACKING;
      work.was_lost = state_ == robot_interfaces::msg::LocalizationStatus::LOST;
      work.generation = cloud_generation_;
      work.pose_epoch = pose_epoch_;
      work.stamp = latest_fastlio_stamp_;
      pending_work_ = std::move(work);
      work_pending_ = true;
      last_queued_generation_ = cloud_generation_;
    }
    work_condition_.notify_one();
  }

  void tryAutomaticInitialization()
  {
    bool initialized = false;
    bool restored = false;
    Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if ((!startup_pose_ && !auto_initialize_from_origin_) ||
        automatic_initialization_attempted_ ||
        !map_loaded_ || !have_fastlio_ || dog_odom_buffer_.empty() ||
        state_ != robot_interfaces::msg::LocalizationStatus::UNINITIALIZED)
      {
        return;
      }
      const double receive_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_fastlio_received_at_).count();
      if (receive_age > max_cloud_age_sec_ ||
        std::abs((now() - latest_fastlio_stamp_).seconds()) > max_cloud_age_sec_)
      {
        status_message_ = "automatic initialization waiting for fresh FAST-LIO data";
        return;
      }

      Eigen::Isometry3d dog_odom_from_base = Eigen::Isometry3d::Identity();
      if (!interpolateDogOdom(
          latest_fastlio_stamp_.nanoseconds(), dog_odom_from_base))
      {
        status_message_ = "automatic initialization waiting for aligned dog odometry";
        return;
      }

      Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
      map_from_base.linear() = rpyToRotation(
        auto_initial_roll_, auto_initial_pitch_, auto_initial_yaw_);
      map_from_base.translation() = Eigen::Vector3d(
        auto_initial_x_, auto_initial_y_,
        layer_z_tolerance_ > 0.0 ? layer_base_z_ : auto_initial_z_);
      restored = startup_pose_.has_value();
      if (restored) {
        map_from_base = startup_pose_->map_from_base;
      }
      initial_pose = map_from_base;
      map_from_fastlio_ = map_from_base * latest_fastlio_from_base_.inverse();
      const Eigen::Isometry3d localized_map_from_base =
        map_from_fastlio_ * latest_fastlio_from_base_;
      map_from_odom_ = planar_navigation_tf_ ?
        planarPose(localized_map_from_base) * planarPose(dog_odom_from_base).inverse() :
        localized_map_from_base * dog_odom_from_base.inverse();
      output_map_from_odom_ = map_from_odom_;
      output_transform_initialized_ = true;
      last_tf_update_time_ = now();
      pose_valid_ = false;
      state_ = robot_interfaces::msg::LocalizationStatus::LOCALIZING;
      consecutive_successes_ = 0;
      consecutive_failures_ = 0;
      pending_candidate_valid_ = false;
      pending_candidate_confirmations_ = 0;
      automatic_initialization_attempted_ = true;
      initial_pose_received_ = true;
      ++pose_epoch_;
      status_message_ = restored ?
        "automatic localization from last saved pose; confirming with ICP" :
        "automatic localization from map origin; confirming with ICP";
      initialized = true;
    }
    if (initialized) {
      publishStatus();
      publishTransform();
      RCLCPP_INFO(
        get_logger(),
        "Automatic localization started from %s: x=%.2f y=%.2f z=%.2f yaw=%.2f",
        restored ? "last saved pose" : "map origin", initial_pose.translation().x(),
        initial_pose.translation().y(), initial_pose.translation().z(),
        std::atan2(initial_pose.linear()(1, 0), initial_pose.linear()(0, 0)));
    }
  }

  RegistrationMetrics calculateRegistrationMetrics(
    const Open3dCloud & source, const Open3dCloud & target,
    const open3d::pipelines::registration::RegistrationResult & result,
    const Eigen::Vector3d & source_origin) const
  {
    RegistrationMetrics metrics;
    if (!target.HasNormals() || result.correspondence_set_.empty()) {
      return metrics;
    }

    const Eigen::Matrix3d rotation = result.transformation_.block<3, 3>(0, 0);
    const Eigen::Vector3d translation = result.transformation_.block<3, 1>(0, 3);
    const Eigen::Vector3d transformed_origin = rotation * source_origin + translation;
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto & correspondence : result.correspondence_set_) {
      const int source_index = correspondence(0);
      const int target_index = correspondence(1);
      if (source_index < 0 || target_index < 0 ||
        source_index >= static_cast<int>(source.points_.size()) ||
        target_index >= static_cast<int>(target.points_.size()))
      {
        continue;
      }
      const Eigen::Vector3d point = rotation * source.points_[source_index] + translation;
      const Eigen::Vector3d normal = target.normals_[target_index].normalized();
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() = (point - transformed_origin).cross(normal);
      jacobian.tail<3>() = normal;
      information.noalias() += jacobian * jacobian.transpose();
    }

    information /= std::max<std::size_t>(1, result.correspondence_set_.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
    if (solver.info() != Eigen::Success) {
      return metrics;
    }
    const auto eigenvalues = solver.eigenvalues();
    metrics.minimum_eigenvalue = std::max(0.0, eigenvalues.minCoeff());
    const double maximum_eigenvalue = std::max(0.0, eigenvalues.maxCoeff());
    if (metrics.minimum_eigenvalue > 1e-12) {
      metrics.condition_number = maximum_eigenvalue / metrics.minimum_eigenvalue;
    }
    return metrics;
  }

  bool registrationPassesQualityGates(
    const RegistrationWork & work,
    const open3d::pipelines::registration::RegistrationResult & result,
    const RegistrationMetrics & metrics)
  {
    const Eigen::Isometry3d candidate(result.transformation_);
    const Eigen::Isometry3d correction = work.initial_map_from_fastlio.inverse() * candidate;
    const double max_translation = work.was_tracking ?
      max_tracking_translation_correction_ : max_localizing_translation_correction_;
    const double max_rotation = work.was_tracking ?
      max_tracking_rotation_correction_rad_ : max_localizing_rotation_correction_rad_;
    double min_fitness;
    double max_inlier_rmse;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      min_fitness = min_fitness_;
      max_inlier_rmse = max_inlier_rmse_;
    }
    return result.transformation_.allFinite() &&
           transformIsSane(candidate, max_reasonable_translation_) &&
           result.correspondence_set_.size() >= static_cast<std::size_t>(min_inliers_) &&
           result.fitness_ >= min_fitness && result.inlier_rmse_ <= max_inlier_rmse &&
           correction.translation().norm() <= max_translation &&
           rotationAngle(correction.linear()) <= max_rotation &&
           metrics.minimum_eigenvalue >= min_information_eigenvalue_ &&
           metrics.condition_number <= max_condition_number_;
  }

  void recordRegistrationFailure(const RegistrationWork & work, const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (work.pose_epoch != pose_epoch_) {
        return;
      }
      consecutive_successes_ = 0;
      ++consecutive_failures_;
      pending_candidate_valid_ = false;
      pending_candidate_confirmations_ = 0;
      if (consecutive_failures_ >= static_cast<std::uint32_t>(failures_to_lose_)) {
        state_ = robot_interfaces::msg::LocalizationStatus::LOST;
        pose_valid_ = false;
        status_message_ = reason + "; localization lost, holding last map->odom";
      } else {
        status_message_ = reason;
      }
    }
    RCLCPP_WARN(
      get_logger(), "ICP skipped generation=%lu reason=%s",
      static_cast<unsigned long>(work.generation), reason.c_str());
    publishStatus();
  }

  void registrationWorker()
  {
    while (true) {
      RegistrationWork work;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        work_condition_.wait(lock, [this]() {return stop_worker_ || work_pending_;});
        if (stop_worker_) {
          return;
        }
        work = std::move(pending_work_);
        work_pending_ = false;
        worker_busy_ = true;
      }
      try {
        if (!work.source_points || work.source_begin > work.source_end ||
          work.source_end > work.source_points->size() ||
          !work.coarse_map_tiles || !work.fine_map_tiles)
        {
          recordRegistrationFailure(work, "registration cache is unavailable");
          std::lock_guard<std::mutex> lock(mutex_);
          worker_busy_ = false;
          continue;
        }
        Open3dCloud source;
        source.points_.assign(
          work.source_points->begin() + static_cast<std::ptrdiff_t>(work.source_begin),
          work.source_points->begin() + static_cast<std::ptrdiff_t>(work.source_end));
        const Eigen::Vector3d crop_extent(
          std::max(1.0, local_map_radius_xy_), std::max(1.0, local_map_radius_xy_),
          std::max(1.0, local_map_half_height_));
        const open3d::geometry::AxisAlignedBoundingBox source_bounds(
          work.fastlio_from_base.translation() - crop_extent,
          work.fastlio_from_base.translation() + crop_extent);
        auto local_source = source.Crop(source_bounds);
        auto coarse_source = local_source->VoxelDownSample(coarse_voxel_size_);
        auto fine_source = local_source->VoxelDownSample(fine_voxel_size_);
        if (fine_source->points_.size() < static_cast<std::size_t>(min_source_points_)) {
          recordRegistrationFailure(work, "local source cloud has too few points");
          std::lock_guard<std::mutex> lock(mutex_);
          worker_busy_ = false;
          continue;
        }

        const Eigen::Vector3d predicted_base =
          (work.initial_map_from_fastlio * work.fastlio_from_base).translation();
        auto local_coarse_map = localMapFromTiles(
          *work.coarse_map_tiles, predicted_base, crop_extent);
        auto local_fine_map = localMapFromTiles(
          *work.fine_map_tiles, predicted_base, crop_extent);
        if (local_coarse_map->points_.size() < static_cast<std::size_t>(min_inliers_) ||
          local_fine_map->points_.size() < static_cast<std::size_t>(min_inliers_))
        {
          recordRegistrationFailure(work, "local map has too few points");
          std::lock_guard<std::mutex> lock(mutex_);
          worker_busy_ = false;
          continue;
        }

        const auto robust_kernel =
          std::make_shared<open3d::pipelines::registration::TukeyLoss>(
          std::max(0.01, robust_kernel_scale_));
        const auto run_fine_registration = [&](const Eigen::Matrix4d & initial_transform) {
            return open3d::pipelines::registration::RegistrationICP(
              *fine_source, *local_fine_map, fine_max_correspondence_, initial_transform,
              open3d::pipelines::registration::TransformationEstimationPointToPlane(robust_kernel),
              open3d::pipelines::registration::ICPConvergenceCriteria(
                1e-6, 1e-6, max_iterations_fine_));
          };
        const auto run_coarse_registration = [&]() {
            return open3d::pipelines::registration::RegistrationICP(
              *coarse_source, *local_coarse_map, coarse_max_correspondence_,
              work.initial_map_from_fastlio.matrix(),
              open3d::pipelines::registration::TransformationEstimationPointToPoint(),
              open3d::pipelines::registration::ICPConvergenceCriteria(
                1e-6, 1e-6, max_iterations_coarse_));
          };
        const bool try_fine_only = tracking_fine_only_ && work.was_tracking;
        auto fine_result = try_fine_only ?
          run_fine_registration(work.initial_map_from_fastlio.matrix()) :
          run_fine_registration(run_coarse_registration().transformation_);
        auto metrics = calculateRegistrationMetrics(
          *fine_source, *local_fine_map, fine_result, work.fastlio_from_base.translation());

        if (try_fine_only && !registrationPassesQualityGates(work, fine_result, metrics)) {
          const auto coarse_result = run_coarse_registration();
          fine_result = run_fine_registration(coarse_result.transformation_);
          metrics = calculateRegistrationMetrics(
            *fine_source, *local_fine_map, fine_result, work.fastlio_from_base.translation());
        }
        processRegistrationResult(work, *fine_source, fine_result, metrics);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Open3D registration failed: %s", error.what());
        recordRegistrationFailure(work, std::string("Open3D exception: ") + error.what());
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_busy_ = false;
      }
    }
  }

  void processRegistrationResult(
    const RegistrationWork & work, const Open3dCloud & source,
    const open3d::pipelines::registration::RegistrationResult & result,
    const RegistrationMetrics & metrics)
  {
    const Eigen::Isometry3d candidate(result.transformation_);
    const Eigen::Isometry3d correction = work.initial_map_from_fastlio.inverse() * candidate;
    const double translation_correction = correction.translation().norm();
    const double rotation_correction = rotationAngle(correction.linear());
    const double max_translation = work.was_tracking ?
      max_tracking_translation_correction_ : max_localizing_translation_correction_;
    const double max_rotation = work.was_tracking ?
      max_tracking_rotation_correction_rad_ : max_localizing_rotation_correction_rad_;
    const auto num_inliers = result.correspondence_set_.size();
    double min_fitness;
    double max_inlier_rmse;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      min_fitness = min_fitness_;
      max_inlier_rmse = max_inlier_rmse_;
    }
    const bool quality_accepted = result.transformation_.allFinite() &&
      transformIsSane(candidate, max_reasonable_translation_) &&
      num_inliers >= static_cast<std::size_t>(min_inliers_) &&
      result.fitness_ >= min_fitness && result.inlier_rmse_ <= max_inlier_rmse &&
      translation_correction <= max_translation && rotation_correction <= max_rotation &&
      metrics.minimum_eigenvalue >= min_information_eigenvalue_ &&
      metrics.condition_number <= max_condition_number_;

    std::string failed_gates;
    const auto add_failed_gate = [&failed_gates](const char * gate) {
        if (!failed_gates.empty()) {
          failed_gates += ",";
        }
        failed_gates += gate;
      };
    if (!result.transformation_.allFinite()) {
      add_failed_gate("nonfinite_transform");
    }
    if (!transformIsSane(candidate, max_reasonable_translation_)) {
      add_failed_gate("unreasonable_transform");
    }
    if (num_inliers < static_cast<std::size_t>(min_inliers_)) {
      add_failed_gate("inliers");
    }
    if (result.fitness_ < min_fitness) {
      add_failed_gate("fitness");
    }
    if (result.inlier_rmse_ > max_inlier_rmse) {
      add_failed_gate("rmse");
    }
    if (translation_correction > max_translation) {
      add_failed_gate("translation");
    }
    if (rotation_correction > max_rotation) {
      add_failed_gate("rotation");
    }
    if (metrics.minimum_eigenvalue < min_information_eigenvalue_) {
      add_failed_gate("min_eigenvalue");
    }
    if (metrics.condition_number > max_condition_number_) {
      add_failed_gate("condition_number");
    }
    if (failed_gates.empty()) {
      failed_gates = "none";
    }

    if (quality_accepted) {
      RCLCPP_DEBUG(
        get_logger(),
        "ICP ACCEPT generation=%lu phase=%s fitness=%.4f(min=%.4f) "
        "rmse=%.4f(max=%.4f) inliers=%lu(min=%d) "
        "correction=%.3fm/%.3frad(max=%.3fm/%.3frad) "
        "min_eigen=%.3e(min=%.3e) condition=%.3e(max=%.3e) failed_gates=%s",
        static_cast<unsigned long>(work.generation),
        work.was_tracking ? "TRACKING" : (work.was_lost ? "LOST" : "LOCALIZING"),
        result.fitness_, min_fitness, result.inlier_rmse_, max_inlier_rmse,
        static_cast<unsigned long>(num_inliers), min_inliers_,
        translation_correction, rotation_correction, max_translation, max_rotation,
        metrics.minimum_eigenvalue, min_information_eigenvalue_,
        metrics.condition_number, max_condition_number_, failed_gates.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "ICP REJECT generation=%lu phase=%s fitness=%.4f(min=%.4f) "
        "rmse=%.4f(max=%.4f) inliers=%lu(min=%d) "
        "correction=%.3fm/%.3frad(max=%.3fm/%.3frad) "
        "min_eigen=%.3e(min=%.3e) condition=%.3e(max=%.3e) failed_gates=%s",
        static_cast<unsigned long>(work.generation),
        work.was_tracking ? "TRACKING" : (work.was_lost ? "LOST" : "LOCALIZING"),
        result.fitness_, min_fitness, result.inlier_rmse_, max_inlier_rmse,
        static_cast<unsigned long>(num_inliers), min_inliers_,
        translation_correction, rotation_correction, max_translation, max_rotation,
        metrics.minimum_eigenvalue, min_information_eigenvalue_,
        metrics.condition_number, max_condition_number_, failed_gates.c_str());
    }

    Eigen::Isometry3d published_map_from_fastlio = candidate;
    bool pose_updated = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (work.pose_epoch != pose_epoch_) {
        return;
      }
      registration_error_ = result.inlier_rmse_;
      inlier_ratio_ = result.fitness_;
      num_inliers_ = num_inliers;
      condition_number_ = metrics.condition_number;
      if (quality_accepted) {
        consecutive_failures_ = 0;
        const bool needs_confirmation = !work.was_tracking ||
          translation_correction > immediate_tracking_translation_ ||
          rotation_correction > immediate_tracking_rotation_rad_;
        const std::uint32_t confirmations_required =
          static_cast<std::uint32_t>(
          std::max(1, work.was_tracking ? large_correction_confirmations_ : successes_to_track_));

        if (needs_confirmation) {
          bool same_candidate = false;
          if (pending_candidate_valid_) {
            const Eigen::Isometry3d candidate_delta = pending_candidate_.inverse() * candidate;
            same_candidate =
              candidate_delta.translation().norm() <= correction_confirmation_translation_ &&
              rotationAngle(candidate_delta.linear()) <= correction_confirmation_rotation_rad_;
          }
          if (!same_candidate) {
            pending_candidate_ = candidate;
            pending_candidate_valid_ = true;
            pending_candidate_confirmations_ = 1;
          } else {
            pending_candidate_ = interpolate(pending_candidate_, candidate, 0.5);
            ++pending_candidate_confirmations_;
          }
          consecutive_successes_ = pending_candidate_confirmations_;
          if (pending_candidate_confirmations_ < confirmations_required) {
            status_message_ = work.was_tracking ?
              "large ICP correction pending multi-frame confirmation" :
              "ICP candidate accepted; confirming with fresh cloud generations";
          } else {
            const Eigen::Isometry3d confirmed_candidate = pending_candidate_;
            map_from_fastlio_ = work.was_tracking && pose_valid_ ?
              interpolate(
              map_from_fastlio_, confirmed_candidate,
              std::clamp(tracking_smoothing_alpha_, 0.0, 1.0)) : confirmed_candidate;
            pending_candidate_valid_ = false;
            pending_candidate_confirmations_ = 0;
            pose_updated = true;
          }
        } else {
          pending_candidate_valid_ = false;
          pending_candidate_confirmations_ = 0;
          ++consecutive_successes_;
          map_from_fastlio_ = interpolate(
            map_from_fastlio_, candidate,
            std::clamp(tracking_smoothing_alpha_, 0.0, 1.0));
          pose_updated = true;
        }

        if (pose_updated) {
          published_map_from_fastlio = map_from_fastlio_;
          const Eigen::Isometry3d localized_map_from_base =
            map_from_fastlio_ * work.fastlio_from_base;
          map_from_odom_ = planar_navigation_tf_ ?
            planarPose(localized_map_from_base) * planarPose(work.dog_odom_from_base).inverse() :
            localized_map_from_base * work.dog_odom_from_base.inverse();
          state_ = robot_interfaces::msg::LocalizationStatus::TRACKING;
          pose_valid_ = true;
          last_valid_map_from_odom_ = map_from_odom_;
          have_last_valid_transform_ = true;
          if (!output_transform_initialized_) {
            output_map_from_odom_ = map_from_odom_;
            output_transform_initialized_ = true;
            last_tf_update_time_ = now();
          }
          status_message_ = "Open3D localization tracking; observation accepted";
        }
      } else {
        consecutive_successes_ = 0;
        ++consecutive_failures_;
        pending_candidate_valid_ = false;
        pending_candidate_confirmations_ = 0;
        if (consecutive_failures_ >= static_cast<std::uint32_t>(failures_to_lose_)) {
          state_ = robot_interfaces::msg::LocalizationStatus::LOST;
          pose_valid_ = false;
          status_message_ = "ICP quality gate failed repeatedly; holding last map->odom";
        } else {
          status_message_ = "ICP rejected: fitness/RMSE/innovation/observability gate";
        }
      }
    }
    if (pose_updated) {
      publishAlignedCloud(source, published_map_from_fastlio, work.stamp);
      // Make the first valid TF available before consumers receive TRACKING.
      publishTransform();
    }
    publishStatus();
  }

  void publishAlignedCloud(
    const Open3dCloud & source, const Eigen::Isometry3d & map_from_fastlio,
    const rclcpp::Time & stamp)
  {
    if (aligned_cloud_publisher_->get_subscription_count() == 0) {
      return;
    }
    PclCloud aligned;
    aligned.reserve(source.points_.size());
    for (const auto & point : source.points_) {
      const Eigen::Vector3d transformed = map_from_fastlio * point;
      pcl::PointXYZI output;
      output.x = transformed.x();
      output.y = transformed.y();
      output.z = transformed.z();
      output.intensity = 0.0F;
      aligned.push_back(output);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    aligned_cloud_publisher_->publish(message);
  }

  void publishTransform()
  {
    Eigen::Isometry3d transform;
    Eigen::Isometry3d visualization_pose = Eigen::Isometry3d::Identity();
    bool publish = false;
    bool publish_visualization_pose = false;
    const rclcpp::Time publish_time = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pose_valid_) {
        const rclcpp::Time update_time = publish_time;
        if (!output_transform_initialized_) {
          output_map_from_odom_ = map_from_odom_;
          output_transform_initialized_ = true;
        } else {
          const double elapsed = std::clamp(
            (update_time - last_tf_update_time_).seconds(), 0.0, 0.2);
          output_map_from_odom_ = stepToward(
            output_map_from_odom_, map_from_odom_,
            std::max(0.0, max_tf_translation_rate_mps_) * elapsed,
            std::max(0.0, max_tf_rotation_rate_radps_) * elapsed);
        }
        last_tf_update_time_ = update_time;
        transform = output_map_from_odom_;
        publish = true;
        if (have_fastlio_ &&
          (publish_time - latest_fastlio_stamp_).seconds() <=
          std::max(0.5, max_cloud_age_sec_))
        {
          visualization_pose = map_from_fastlio_ * latest_fastlio_from_base_;
          publish_visualization_pose = true;
          const double receive_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - latest_fastlio_received_at_).count();
          if (map_loaded_ && state_ == robot_interfaces::msg::LocalizationStatus::TRACKING &&
            !map_id_.empty() && receive_age <= max_cloud_age_sec_ &&
            std::abs((publish_time - latest_fastlio_stamp_).seconds()) <= max_cloud_age_sec_ &&
            transformIsSane(visualization_pose, max_reasonable_translation_))
          {
            dog_slam::CachedLocalizationPose cached;
            cached.map_id = map_id_;
            cached.map_path = loaded_map_path_;
            cached.map_frame = map_frame_;
            cached.base_frame = base_frame_;
            cached.stamp_nanoseconds = latest_fastlio_stamp_.nanoseconds();
            cached.map_from_base = visualization_pose;
            last_trusted_pose_ = std::move(cached);
          }
        }
      } else if ((state_ == robot_interfaces::msg::LocalizationStatus::LOST ||
        state_ == robot_interfaces::msg::LocalizationStatus::LOCALIZING) &&
        output_transform_initialized_)
      {
        transform = output_map_from_odom_;
        publish = true;
      }
    }
    if (!publish) {
      return;
    }
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = publish_time;
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = eigenToTransform(transform);
    tf_broadcaster_->sendTransform(message);
    if (publish_visualization_pose) {
      geometry_msgs::msg::PoseStamped pose_message;
      pose_message.header.stamp = publish_time;
      pose_message.header.frame_id = map_frame_;
      pose_message.pose = eigenToPose(visualization_pose);
      visualization_pose_publisher_->publish(pose_message);
    }
  }

  std::string stateLabel(std::uint8_t state) const
  {
    switch (state) {
      case robot_interfaces::msg::LocalizationStatus::LOCALIZING:
        return "LOCALIZING";
      case robot_interfaces::msg::LocalizationStatus::TRACKING:
        return "TRACKING";
      case robot_interfaces::msg::LocalizationStatus::LOST:
        return "LOST";
      default:
        return "UNINITIALIZED";
    }
  }

  void publishStatus()
  {
    robot_interfaces::msg::LocalizationStatus status;
    const auto status_time = now();
    status.header.stamp = status_time;
    status.header.frame_id = map_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.state = state_;
      status.state_label = stateLabel(state_);
      status.map_id = map_id_;
      status.map_loaded = map_loaded_;
      status.pose_valid = pose_valid_;
      const double receive_age = have_fastlio_ ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_fastlio_received_at_).count() : -1.0;
      const double timestamp_age = have_fastlio_ ?
        (status_time - latest_fastlio_stamp_).seconds() : -1.0;
      status.lidar_data_age_sec = receive_age;
      status.lidar_data_fresh = have_fastlio_ && receive_age <= max_cloud_age_sec_;
      status.lidar_timestamp_age_sec = timestamp_age;
      status.lidar_timestamp_valid = have_fastlio_ &&
        std::abs(timestamp_age) <= max_cloud_age_sec_;
      status.initial_pose_received = initial_pose_received_;
      status.registration_error = registration_error_;
      status.inlier_ratio = inlier_ratio_;
      status.condition_number = condition_number_;
      status.num_inliers = num_inliers_;
      status.consecutive_successes = consecutive_successes_;
      status.consecutive_failures = consecutive_failures_;
      status.message = status_message_;
    }
    status_publisher_->publish(status);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string fastlio_odom_topic_;
  std::string dog_odom_topic_;
  std::string initial_map_id_;
  std::string initial_map_path_;
  std::string initial_visualization_map_path_;
  double layer_base_z_{0.0};
  double layer_z_tolerance_{0.0};
  bool auto_initialize_from_origin_;
  bool persist_last_pose_;
  bool restore_last_pose_;
  std::string pose_cache_path_;
  std::string loaded_map_path_;
  std::optional<dog_slam::CachedLocalizationPose> startup_pose_;
  std::optional<dog_slam::CachedLocalizationPose> last_trusted_pose_;
  double auto_initial_x_;
  double auto_initial_y_;
  double auto_initial_z_;
  double auto_initial_roll_;
  double auto_initial_pitch_;
  double auto_initial_yaw_;
  bool planar_navigation_tf_;
  double window_duration_sec_;
  double dog_odom_buffer_duration_sec_;
  double max_dog_odom_interpolation_gap_sec_;
  double max_cloud_age_sec_;
  double registration_rate_hz_;
  bool tracking_fine_only_;
  double tf_rate_hz_;
  double coarse_voxel_size_;
  double fine_voxel_size_;
  double coarse_max_correspondence_;
  double fine_max_correspondence_;
  double normal_radius_;
  int max_iterations_coarse_;
  int max_iterations_fine_;
  int min_source_points_;
  int min_inliers_;
  double min_fitness_;
  double max_inlier_rmse_;
  double max_tracking_translation_correction_;
  double max_tracking_rotation_correction_rad_;
  double max_localizing_translation_correction_;
  double max_localizing_rotation_correction_rad_;
  double local_map_radius_xy_;
  double local_map_half_height_;
  double map_tile_size_;
  double max_condition_number_;
  double min_information_eigenvalue_;
  double robust_kernel_scale_;
  double immediate_tracking_translation_;
  double immediate_tracking_rotation_rad_;
  double correction_confirmation_translation_;
  double correction_confirmation_rotation_rad_;
  int large_correction_confirmations_;
  bool override_initial_base_z_;
  double initial_base_z_;
  int successes_to_track_;
  int failures_to_lose_;
  double tracking_smoothing_alpha_;
  double max_tf_translation_rate_mps_;
  double max_tf_rotation_rate_radps_;
  double max_reasonable_translation_;
  double max_fastlio_pose_jump_translation_;
  double max_fastlio_pose_jump_rotation_rad_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  std::mutex mutex_;
  std::condition_variable work_condition_;
  std::thread worker_;
  bool stop_worker_{false};
  bool work_pending_{false};
  bool worker_busy_{false};
  RegistrationWork pending_work_;
  std::deque<WindowFrame> window_;
  std::shared_ptr<std::vector<Eigen::Vector3d>> window_points_ =
    std::make_shared<std::vector<Eigen::Vector3d>>();
  std::size_t window_points_begin_{0};
  std::deque<DogOdomSample> dog_odom_buffer_;
  std::shared_ptr<const MapTileMap> coarse_map_tiles_;
  std::shared_ptr<const MapTileMap> fine_map_tiles_;
  Eigen::Isometry3d latest_fastlio_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d latest_continuity_fastlio_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pending_fastlio_rebase_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_from_fastlio_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_from_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d output_map_from_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_valid_map_from_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pending_candidate_ = Eigen::Isometry3d::Identity();
  rclcpp::Time latest_fastlio_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_continuity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time minimum_cloud_stamp_after_jump_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point latest_fastlio_received_at_{};
  rclcpp::Time last_tf_update_time_{0, 0, RCL_ROS_TIME};
  bool have_fastlio_{false};
  bool have_continuity_pose_{false};
  bool pending_fastlio_rebase_{false};
  bool map_loaded_{false};
  bool pose_valid_{false};
  bool have_last_valid_transform_{false};
  bool output_transform_initialized_{false};
  bool pending_candidate_valid_{false};
  bool automatic_initialization_attempted_{false};
  bool initial_pose_received_{false};
  std::uint64_t cloud_generation_{0};
  std::uint64_t last_queued_generation_{0};
  std::uint64_t pose_epoch_{0};
  std::uint8_t state_{robot_interfaces::msg::LocalizationStatus::UNINITIALIZED};
  std::string map_id_;
  std::string status_message_{"waiting for map"};
  double registration_error_{std::numeric_limits<double>::infinity()};
  double inlier_ratio_{0.0};
  double condition_number_{std::numeric_limits<double>::infinity()};
  std::size_t num_inliers_{0};
  std::uint32_t pending_candidate_confirmations_{0};
  std::uint32_t consecutive_successes_{0};
  std::uint32_t consecutive_failures_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> fastlio_odom_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dog_odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Service<robot_interfaces::srv::LoadPcdMap>::SharedPtr load_map_service_;
  rclcpp::Service<robot_interfaces::srv::SetLocalizationPose>::SharedPtr
    initial_pose_service_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    visualization_pose_publisher_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr pose_cache_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Open3dLocalizer>());
  rclcpp::shutdown();
  return 0;
}
