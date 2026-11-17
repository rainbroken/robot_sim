#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_map_server/map_io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "robot_interfaces/msg/map_cloud_update.hpp"
#include "robot_interfaces/srv/save_pcd_map.hpp"
#include "dog_slam/descriptor_index.hpp"
#include "dog_slam/colored_map_builder.hpp"
#include "dog_slam/loop_closure_types.hpp"
#include "dog_slam/loop_registration.hpp"
#include "dog_slam/map_rebuilder.hpp"
#include "dog_slam/point_cloud_utils.hpp"
#include "dog_slam/pose_graph.hpp"
#include "dog_slam/solid_descriptor.hpp"

namespace dog_slam
{
namespace
{
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

class SaveInProgressGuard
{
public:
  explicit SaveInProgressGuard(std::atomic<bool> & flag)
  : flag_(flag), acquired_(!flag_.exchange(true))
  {
  }

  ~SaveInProgressGuard()
  {
    if (acquired_) {
      flag_.store(false);
    }
  }

  bool acquired() const {return acquired_;}

private:
  std::atomic<bool> & flag_;
  bool acquired_;
};

bool occupancyIsEmpty(const std::optional<nav_msgs::msg::OccupancyGrid> & occupancy)
{
  if (!occupancy || occupancy->info.width == 0 || occupancy->info.height == 0) {
    return true;
  }
  for (const auto value : occupancy->data) {
    if (value >= 0) {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::OccupancyGrid placeholderOccupancy(
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  double resolution)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(std::max(0.01, resolution));
  grid.info.width = 1;
  grid.info.height = 1;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(1, -1);
  return grid;
}

bool writeOccupancyFiles(
  const std::filesystem::path & directory,
  const std::optional<nav_msgs::msg::OccupancyGrid> & occupancy,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  double resolution)
{
  nav_msgs::msg::OccupancyGrid grid = occupancy ?
    *occupancy : placeholderOccupancy(frame_id, stamp, resolution);
  nav2_map_server::SaveParameters save_parameters;
  save_parameters.map_file_name = (directory / "occupancy").string();
  save_parameters.image_format = "pgm";
  // PGM unknown is 205: (255-205)/255 ~= 0.196078. A threshold
  // of 0.25 loads unknown cells as FREE on the next map_server run.
  save_parameters.free_thresh = 0.196;
  save_parameters.occupied_thresh = 0.65;
  save_parameters.mode = nav2_map_server::MapMode::Trinary;
  return nav2_map_server::saveMapToFile(grid, save_parameters);
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  rotation = rotation.norm() < 1.0e-9 ? Eigen::Quaterniond::Identity() : rotation.normalized();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond rotation(transform.linear());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

double rotationAngle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}
}  // namespace

class LoopClosureMapper final : public rclcpp::Node
{
public:
  LoopClosureMapper()
  : Node("loop_closure_mapper")
  {
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/fastlio/leveled_cloud");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio/base_odom");
    colored_cloud_topic_ = declare_parameter<std::string>(
      "colored_cloud_topic", "/fastlio/leveled_colored_points");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/mapping/loop_closure_diagnostics");
    diagnostics_period_sec_ = declare_parameter<double>("diagnostics_period_sec", 1.0);
    input_stale_timeout_sec_ = declare_parameter<double>("input_stale_timeout_sec", 3.0);
    keyframe_translation_ = declare_parameter<double>("keyframe_translation", 0.50);
    keyframe_rotation_rad_ = declare_parameter<double>("keyframe_rotation_rad", 0.17);
    keyframe_voxel_size_ = declare_parameter<double>("keyframe_voxel_size", 0.20);
    map_keyframe_voxel_size_ = declare_parameter<double>("map_keyframe_voxel_size", 0.10);
    colored_keyframe_voxel_size_ = declare_parameter<double>(
      "colored_keyframe_voxel_size", 0.06);
    colored_map_voxel_size_ = declare_parameter<double>("colored_map_voxel_size", 0.08);
    color_association_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("color_association_tolerance_ms", 5.0) * 0.001);
    color_cache_duration_sec_ = std::max(
      0.1, declare_parameter<double>("color_cache_duration_sec", 2.0));
    min_point_range_ = declare_parameter<double>("min_point_range", 0.8);
    max_point_range_ = declare_parameter<double>("max_point_range", 55.0);
    min_point_z_ = declare_parameter<double>("min_point_z", -2.0);
    max_point_z_ = declare_parameter<double>("max_point_z", 18.0);
    self_min_x_ = declare_parameter<double>("self_min_x", -0.48);
    self_max_x_ = declare_parameter<double>("self_max_x", 0.48);
    self_min_y_ = declare_parameter<double>("self_min_y", -0.35);
    self_max_y_ = declare_parameter<double>("self_max_y", 0.35);
    self_min_z_ = declare_parameter<double>("self_min_z", -0.40);
    self_max_z_ = declare_parameter<double>("self_max_z", 0.55);

    SolidDescriptorEngine::Config solid_config;
    solid_config.range_bins = declare_parameter<int>("solid_range_bins", 40);
    solid_config.azimuth_bins = declare_parameter<int>("solid_azimuth_bins", 60);
    solid_config.elevation_bins = declare_parameter<int>("solid_elevation_bins", 32);
    solid_config.min_elevation_rad = declare_parameter<double>(
      "solid_min_elevation_rad", -0.65);
    solid_config.max_elevation_rad = declare_parameter<double>(
      "solid_max_elevation_rad", 1.10);
    solid_config.min_point_range = min_point_range_;
    solid_config.max_point_range = max_point_range_;
    descriptor_min_score_ = declare_parameter<double>("descriptor_min_score", 0.72);
    descriptor_top_k_ = declare_parameter<int>("descriptor_top_k", 4);
    loop_min_index_separation_ = declare_parameter<int>("loop_min_index_separation", 30);
    loop_min_time_separation_sec_ = declare_parameter<double>(
      "loop_min_time_separation_sec", 30.0);
    loop_submap_half_width_ = declare_parameter<int>("loop_submap_half_width", 4);
    loop_confirmations_required_ = declare_parameter<int>("loop_confirmations_required", 2);
    loop_confirmation_index_tolerance_ = declare_parameter<int>(
      "loop_confirmation_index_tolerance", 5);
    loop_confirmation_translation_ = declare_parameter<double>(
      "loop_confirmation_translation", 1.0);
    loop_confirmation_rotation_rad_ = declare_parameter<double>(
      "loop_confirmation_rotation_rad", 0.20);
    max_global_correction_translation_ = declare_parameter<double>(
      "max_global_correction_translation", 3.0);
    max_global_correction_rotation_rad_ = declare_parameter<double>(
      "max_global_correction_rotation_rad", 0.35);

    LoopRegistration::Config registration_config;
    registration_config.voxel_size = declare_parameter<double>("gicp_voxel_size", 0.30);
    registration_config.max_correspondence = declare_parameter<double>(
      "gicp_max_correspondence", 1.20);
    registration_config.inlier_distance = declare_parameter<double>(
      "gicp_inlier_distance", 0.45);
    registration_config.robust_kernel_scale = declare_parameter<double>(
      "gicp_robust_kernel_scale", 1.0);
    registration_config.max_iterations = declare_parameter<int>("gicp_max_iterations", 32);
    registration_config.num_threads = declare_parameter<int>("gicp_num_threads", 2);
    registration_config.min_inliers = declare_parameter<int>("min_loop_inliers", 220);
    registration_config.min_overlap = declare_parameter<double>("min_loop_overlap", 0.30);
    registration_config.max_rmse = declare_parameter<double>("max_loop_rmse", 0.32);
    registration_config.max_translation = declare_parameter<double>(
      "max_loop_translation", 6.0);
    registration_config.max_z = declare_parameter<double>("max_loop_z", 1.2);
    registration_config.max_roll_pitch_rad = declare_parameter<double>(
      "max_loop_roll_pitch_rad", 0.35);
    registration_config.min_information_eigenvalue = declare_parameter<double>(
      "min_loop_information_eigenvalue", 1.0e-5);
    registration_config.max_condition_number = declare_parameter<double>(
      "max_loop_condition_number", 1.0e8);
    gicp_voxel_size_ = registration_config.voxel_size;
    min_loop_inliers_ = registration_config.min_inliers;

    PoseGraph::Config graph_config;
    graph_config.odom_rotation_sigma = declare_parameter<double>(
      "odom_rotation_sigma", 0.015);
    graph_config.odom_translation_sigma = declare_parameter<double>(
      "odom_translation_sigma", 0.05);
    graph_config.loop_rotation_sigma = declare_parameter<double>(
      "loop_rotation_sigma", 0.04);
    graph_config.loop_translation_sigma = declare_parameter<double>(
      "loop_translation_sigma", 0.12);
    graph_config.loop_cauchy_scale = declare_parameter<double>("loop_cauchy_scale", 1.0);
    graph_config.max_loop_rmse = registration_config.max_rmse;
    publish_every_keyframes_ = declare_parameter<int>("publish_every_keyframes", 8);

    MapRebuilder::Config map_config;
    map_config.full_map_voxel_size = declare_parameter<double>("full_map_voxel_size", 0.08);
    map_config.localization_voxel_size = declare_parameter<double>(
      "localization_voxel_size", 0.20);
    map_config.localization_min_observations = declare_parameter<int>(
      "localization_min_observations", 2);
    map_config.dynamic_filter_enabled = declare_parameter<bool>(
      "dynamic_filter_enabled", false);
    map_config.dynamic_filter_voxel_size = declare_parameter<double>(
      "dynamic_filter_voxel_size", 0.20);
    map_config.dynamic_filter_max_range = declare_parameter<double>(
      "dynamic_filter_max_range", 25.0);
    map_config.dynamic_filter_endpoint_margin_base = declare_parameter<double>(
      "dynamic_filter_endpoint_margin_base", 0.15);
    map_config.dynamic_filter_endpoint_margin_per_meter = declare_parameter<double>(
      "dynamic_filter_endpoint_margin_per_meter", 0.005);
    map_config.dynamic_filter_endpoint_margin_max = declare_parameter<double>(
      "dynamic_filter_endpoint_margin_max", 0.35);
    map_config.dynamic_filter_min_hit_keyframes = declare_parameter<int>(
      "dynamic_filter_min_hit_keyframes", 2);
    map_config.dynamic_filter_observer_voxel_size = declare_parameter<double>(
      "dynamic_filter_observer_voxel_size", 0.50);
    map_config.dynamic_filter_min_observer_positions = declare_parameter<int>(
      "dynamic_filter_min_observer_positions", 2);
    map_config.dynamic_filter_prob_hit = declare_parameter<double>(
      "dynamic_filter_prob_hit", 0.70);
    map_config.dynamic_filter_prob_miss = declare_parameter<double>(
      "dynamic_filter_prob_miss", 0.40);
    map_config.dynamic_filter_clamping_min = declare_parameter<double>(
      "dynamic_filter_clamping_min", 0.12);
    map_config.dynamic_filter_clamping_max = declare_parameter<double>(
      "dynamic_filter_clamping_max", 0.97);
    map_config.dynamic_filter_occupancy_threshold = declare_parameter<double>(
      "dynamic_filter_occupancy_threshold", 0.50);
    map_config.lidar_origin_x = declare_parameter<double>("lidar_origin_x", 0.25);
    map_config.lidar_origin_y = declare_parameter<double>("lidar_origin_y", 0.0);
    map_config.lidar_origin_z = declare_parameter<double>("lidar_origin_z", 0.12);
    map_config.outlier_filter_enabled = declare_parameter<bool>(
      "outlier_filter_enabled", false);
    map_config.outlier_filter_radius = declare_parameter<double>(
      "outlier_filter_radius", 0.25);
    map_config.outlier_filter_min_neighbors = declare_parameter<int>(
      "outlier_filter_min_neighbors", 2);
    map_config.occupancy_resolution = declare_parameter<double>("occupancy_resolution", 0.05);
    map_config.obstacle_min_height = declare_parameter<double>("obstacle_min_height", 0.08);
    map_config.obstacle_max_height = declare_parameter<double>("obstacle_max_height", 1.20);
    map_config.ground_distance_threshold = declare_parameter<double>(
      "ground_distance_threshold", 0.06);
    map_config.fallback_ground_below_sensor = declare_parameter<double>(
      "fallback_ground_below_sensor", 0.35);
    map_config.occupancy_max_range = declare_parameter<double>("occupancy_max_range", 15.0);

    solid_ = std::make_unique<SolidDescriptorEngine>(solid_config);
    descriptor_index_ = std::make_unique<DescriptorIndex>();
    registration_ = std::make_unique<LoopRegistration>(registration_config);
    pose_graph_ = std::make_unique<PoseGraph>(graph_config);
    map_rebuilder_ = std::make_unique<MapRebuilder>(map_config);

    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mapping/cloud_map", rclcpp::QoS(1).reliable().transient_local());
    map_reset_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mapping/cloud_map_reset", rclcpp::QoS(1).reliable().transient_local());
    map_update_publisher_ = create_publisher<robot_interfaces::msg::MapCloudUpdate>(
      "/mapping/cloud_map_update", rclcpp::QoS(40).reliable().transient_local());
    occupancy_resolution_ = map_config.occupancy_resolution;
    occupancy_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/slam_map", rclcpp::QoS(1).reliable().transient_local());
    occupancy_update_publisher_ = create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      "/slam_map_updates", rclcpp::QoS(20).reliable());
    optimized_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mapping/optimized_odom", rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    save_service_ = create_service<robot_interfaces::srv::SavePcdMap>(
      "/mapping/save_pcd_map",
      std::bind(
        &LoopClosureMapper::saveCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    cloud_subscriber_.subscribe(this, cloud_topic_, rmw_qos_profile_sensor_data);
    odom_subscriber_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(40), cloud_subscriber_, odom_subscriber_);
    synchronizer_->registerCallback(
      std::bind(
        &LoopClosureMapper::sensorCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    colored_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      colored_cloud_topic_, rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&LoopClosureMapper::coloredCloudCallback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, diagnostics_period_sec_)),
      std::bind(&LoopClosureMapper::publishDiagnostics, this));

    worker_ = std::thread(&LoopClosureMapper::workerLoop, this);
    RCLCPP_INFO(
      get_logger(),
      "Loop mapper ready: SOLiD=%dx%dx%d, small_gicp threads=%d, confirmations=%d",
      solid_config.range_bins, solid_config.azimuth_bins, solid_config.elevation_bins,
      registration_config.num_threads, loop_confirmations_required_);
  }

  ~LoopClosureMapper() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_worker_ = true;
    }
    queue_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct ColorObservation
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    ColoredCloud::Ptr world_cloud;
  };

  struct PendingLoop
  {
    bool valid = false;
    std::size_t target_index = 0;
    Eigen::Isometry3d map_from_local = Eigen::Isometry3d::Identity();
    int confirmations = 0;
  };

  void sensorCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_message,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_message)
  {
    ++synchronized_input_count_;
    last_input_steady_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    const Eigen::Isometry3d local_pose = poseToEigen(odom_message->pose.pose);
    publishOptimizedOdometry(*odom_message, local_pose);
    if (save_in_progress_.load(std::memory_order_relaxed)) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (have_last_queued_pose_) {
        const Eigen::Isometry3d delta = last_queued_pose_.inverse() * local_pose;
        if (delta.translation().norm() < keyframe_translation_ &&
          rotationAngle(delta.linear()) < keyframe_rotation_rad_)
        {
          ++motion_skipped_count_;
          return;
        }
      }
    }

    auto world_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*cloud_message, *world_cloud);
    auto body_cloud = std::make_shared<Cloud>();
    body_cloud->reserve(world_cloud->size());
    const Eigen::Isometry3d base_from_world = local_pose.inverse();
    for (const auto & point : world_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Eigen::Vector3d p = base_from_world * Eigen::Vector3d(point.x, point.y, point.z);
      const double range = p.norm();
      if (range < min_point_range_ || range > max_point_range_ ||
        p.z() < min_point_z_ || p.z() > max_point_z_)
      {
        continue;
      }
      if (p.x() >= self_min_x_ && p.x() <= self_max_x_ &&
        p.y() >= self_min_y_ && p.y() <= self_max_y_ &&
        p.z() >= self_min_z_ && p.z() <= self_max_z_)
      {
        continue;
      }
      pcl::PointXYZI output;
      output.x = static_cast<float>(p.x());
      output.y = static_cast<float>(p.y());
      output.z = static_cast<float>(p.z());
      output.intensity = point.intensity;
      body_cloud->push_back(output);
    }
    auto map_body_cloud = voxelized(body_cloud, map_keyframe_voxel_size_);
    auto loop_body_cloud = voxelized(map_body_cloud, keyframe_voxel_size_);
    if (loop_body_cloud->size() < static_cast<std::size_t>(min_loop_inliers_)) {
      ++point_skipped_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Mapping keyframe skipped: only %zu filtered points", loop_body_cloud->size());
      return;
    }

    Keyframe keyframe;
    keyframe.stamp = rclcpp::Time(cloud_message->header.stamp);
    keyframe.local_pose = local_pose;
    keyframe.cloud = loop_body_cloud;
    keyframe.map_cloud = map_body_cloud;
    attachCachedColor(keyframe);
    keyframe.descriptor = solid_->make(*loop_body_cloud, local_pose);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      last_queued_pose_ = local_pose;
      have_last_queued_pose_ = true;
      pending_keyframes_.push_back(std::move(keyframe));
      ++queued_keyframe_count_;
    }
    queue_condition_.notify_one();
  }

  void publishOptimizedOdometry(
    const nav_msgs::msg::Odometry & source, const Eigen::Isometry3d & local_pose)
  {
    Eigen::Isometry3d map_from_local;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      map_from_local = map_from_local_;
    }
    auto output = source;
    output.header.frame_id = map_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose = eigenToPose(map_from_local * local_pose);
    optimized_odom_publisher_->publish(output);
  }

  void workerLoop()
  {
    while (true) {
      Keyframe keyframe;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(
          lock, [this]() {return stop_worker_ || !pending_keyframes_.empty();});
        if (stop_worker_) {
          return;
        }
        keyframe = std::move(pending_keyframes_.front());
        pending_keyframes_.pop_front();
        worker_busy_ = true;
      }
      try {
        attachCachedColor(keyframe);
        const rclcpp::Time keyframe_stamp = keyframe.stamp;
        processKeyframe(std::move(keyframe));
        attachCachedColorToStoredKeyframe(keyframe_stamp);
      } catch (const std::exception & error) {
        ++worker_failure_count_;
        RCLCPP_ERROR(get_logger(), "Loop mapping worker failed on a keyframe: %s", error.what());
      }
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        worker_busy_ = false;
      }
      worker_idle_condition_.notify_all();
    }
  }

  ColoredCloud::Ptr coloredCloudToLocal(
    const ColoredCloud & world_cloud, const Eigen::Isometry3d & local_pose) const
  {
    auto local = std::make_shared<ColoredCloud>();
    pcl::transformPointCloud(
      world_cloud, *local, local_pose.inverse().matrix().cast<float>());
    local->erase(
      std::remove_if(
        local->begin(), local->end(),
        [this](const pcl::PointXYZRGB & point) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            return true;
          }
          const double range = std::sqrt(
            static_cast<double>(point.x) * point.x +
            static_cast<double>(point.y) * point.y +
            static_cast<double>(point.z) * point.z);
          return range < min_point_range_ || range > max_point_range_ ||
                 point.z < min_point_z_ || point.z > max_point_z_ ||
                 (point.x >= self_min_x_ && point.x <= self_max_x_ &&
                 point.y >= self_min_y_ && point.y <= self_max_y_ &&
                 point.z >= self_min_z_ && point.z <= self_max_z_);
        }),
      local->end());
    if (local->empty()) {
      return local;
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(local);
    const float leaf = static_cast<float>(std::max(0.02, colored_keyframe_voxel_size_));
    filter.setLeafSize(leaf, leaf, leaf);
    auto filtered = std::make_shared<ColoredCloud>();
    filter.filter(*filtered);
    return filtered;
  }

  bool stampsMatch(const rclcpp::Time & left, const rclcpp::Time & right) const
  {
    return std::abs((left - right).seconds()) <= color_association_tolerance_sec_;
  }

  std::optional<ColorObservation> takeColorObservation(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(color_mutex_);
    auto best = color_observations_.end();
    double best_delta = std::numeric_limits<double>::infinity();
    for (auto iterator = color_observations_.begin(); iterator != color_observations_.end();
      ++iterator)
    {
      const double delta = std::abs((iterator->stamp - stamp).seconds());
      if (delta <= color_association_tolerance_sec_ && delta < best_delta) {
        best = iterator;
        best_delta = delta;
      }
    }
    if (best == color_observations_.end()) {
      return std::nullopt;
    }
    ColorObservation observation = std::move(*best);
    color_observations_.erase(best);
    return observation;
  }

  void attachCachedColor(Keyframe & keyframe)
  {
    if (keyframe.colored_cloud) {
      return;
    }
    auto observation = takeColorObservation(keyframe.stamp);
    if (observation && observation->world_cloud) {
      keyframe.colored_cloud = coloredCloudToLocal(*observation->world_cloud, keyframe.local_pose);
      if (!keyframe.colored_cloud->empty()) {
        ++colored_keyframe_count_;
      }
    }
  }

  void attachCachedColorToStoredKeyframe(const rclcpp::Time & stamp)
  {
    std::size_t index = 0;
    Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      auto iterator = std::find_if(
        keyframes_.begin(), keyframes_.end(),
        [this, &stamp](const Keyframe & keyframe) {
          return !keyframe.colored_cloud && stampsMatch(keyframe.stamp, stamp);
        });
      if (iterator == keyframes_.end()) {
        return;
      }
      index = static_cast<std::size_t>(std::distance(keyframes_.begin(), iterator));
      local_pose = iterator->local_pose;
    }
    auto observation = takeColorObservation(stamp);
    if (!observation || !observation->world_cloud) {
      return;
    }
    auto local = coloredCloudToLocal(*observation->world_cloud, local_pose);
    if (local->empty()) {
      return;
    }
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (index < keyframes_.size() && !keyframes_[index].colored_cloud &&
      stampsMatch(keyframes_[index].stamp, stamp))
    {
      keyframes_[index].colored_cloud = std::move(local);
      ++colored_keyframe_count_;
    }
  }

  void coloredCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    auto world_cloud = std::make_shared<ColoredCloud>();
    pcl::fromROSMsg(*message, *world_cloud);
    if (world_cloud->empty()) {
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    {
      std::lock_guard<std::mutex> lock(color_mutex_);
      color_observations_.push_back(ColorObservation{stamp, world_cloud});
      while (!color_observations_.empty() &&
        (stamp - color_observations_.front().stamp).seconds() > color_cache_duration_sec_)
      {
        color_observations_.pop_front();
        ++dropped_color_observation_count_;
      }
      while (color_observations_.size() > 100U) {
        color_observations_.pop_front();
        ++dropped_color_observation_count_;
      }
    }
    attachCachedColorToStoredKeyframe(stamp);
  }

  void applyOptimizedPosesLocked(const std::vector<Eigen::Isometry3d> & poses)
  {
    if (poses.size() != keyframes_.size()) {
      throw std::runtime_error("pose graph and keyframe store have different sizes");
    }
    for (std::size_t index = 0; index < poses.size(); ++index) {
      keyframes_[index].optimized_pose = poses[index];
    }
  }

  void processKeyframe(Keyframe keyframe)
  {
    bool loop_added = false;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      keyframe.id = keyframes_.size();
      const auto optimized = pose_graph_->addKeyframe(keyframe.local_pose);
      keyframes_.push_back(std::move(keyframe));
      applyOptimizedPosesLocked(optimized);

      const std::size_t current_index = keyframes_.size() - 1;
      const auto candidates = findCandidatesLocked(current_index);
      descriptor_index_->add(keyframes_[current_index].descriptor.range);
      descriptor_candidate_count_ += candidates.size();
      RegistrationResult best_registration;
      std::size_t best_target = 0;
      double best_descriptor_score = 0.0;
      for (const auto & candidate : candidates) {
        const auto registration = verifyCandidateLocked(current_index, candidate);
        if (!registration.accepted) {
          continue;
        }

        const Eigen::Isometry3d proposed_correction =
          keyframes_[candidate.index].optimized_pose *
          registration.target_from_source *
          keyframes_[current_index].local_pose.inverse();
        const double correction_translation = proposed_correction.translation().norm();
        const double correction_rotation = rotationAngle(proposed_correction.linear());
        if (correction_translation > max_global_correction_translation_ ||
          correction_rotation > max_global_correction_rotation_rad_)
        {
          ++global_consistency_reject_count_;
          RCLCPP_WARN(
            get_logger(),
            "GLOBAL CONSISTENCY REJECT target=%zu current=%zu translation=%.3f/%.3f m "
            "rotation=%.3f/%.3f rad solid=%.3f overlap=%.3f rmse=%.3f",
            candidate.index, current_index,
            correction_translation, max_global_correction_translation_,
            correction_rotation, max_global_correction_rotation_rad_,
            candidate.descriptor_score, registration.overlap, registration.rmse);
          continue;
        }

        if (!best_registration.accepted || registration.rmse < best_registration.rmse) {
          best_registration = registration;
          best_target = candidate.index;
          best_descriptor_score = candidate.descriptor_score;
        }
      }

      if (best_registration.accepted) {
        ++geometric_accept_count_;
        last_registration_ = best_registration;
        const Eigen::Isometry3d correction =
          keyframes_[best_target].optimized_pose *
          best_registration.target_from_source *
          keyframes_[current_index].local_pose.inverse();
        bool consistent = false;
        if (pending_loop_.valid) {
          const Eigen::Isometry3d delta = pending_loop_.map_from_local.inverse() * correction;
          consistent =
            std::abs(
            static_cast<long>(pending_loop_.target_index) -
            static_cast<long>(best_target)) <= loop_confirmation_index_tolerance_ &&
            delta.translation().norm() <= loop_confirmation_translation_ &&
            rotationAngle(delta.linear()) <= loop_confirmation_rotation_rad_;
        }
        if (consistent) {
          ++pending_loop_.confirmations;
          pending_loop_.map_from_local = correction;
          pending_loop_.target_index = best_target;
        } else {
          pending_loop_.valid = true;
          pending_loop_.target_index = best_target;
          pending_loop_.map_from_local = correction;
          pending_loop_.confirmations = 1;
        }

        if (pending_loop_.confirmations >= std::max(1, loop_confirmations_required_)) {
          applyOptimizedPosesLocked(
            pose_graph_->addLoop(
              best_target, current_index, best_registration.target_from_source,
              best_registration.rmse, best_registration.overlap));
          loop_edges_.emplace_back(best_target, current_index);
          pending_loop_ = PendingLoop{};
          loop_added = true;
          RCLCPP_WARN(
            get_logger(),
            "LOOP ACCEPT target=%zu current=%zu solid=%.3f overlap=%.3f rmse=%.3f "
            "inliers=%zu min_eigen=%.3e condition=%.3e",
            best_target, current_index, best_descriptor_score, best_registration.overlap,
            best_registration.rmse, best_registration.num_inliers,
            best_registration.min_eigenvalue, best_registration.condition_number);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Loop candidate pending confirmation %d/%d: target=%zu current=%zu "
            "solid=%.3f overlap=%.3f rmse=%.3f",
            pending_loop_.confirmations, std::max(1, loop_confirmations_required_),
            best_target, current_index, best_descriptor_score, best_registration.overlap,
            best_registration.rmse);
        }
      } else {
        if (!candidates.empty()) {
          geometric_reject_count_ += candidates.size();
        }
        pending_loop_ = PendingLoop{};
      }
      ++processed_keyframe_count_;
      updateCorrectionLocked();
    }

    std::size_t keyframe_count = 0;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      keyframe_count = keyframes_.size();
    }
    const bool periodic_publish = keyframe_count == 1 ||
      (publish_every_keyframes_ > 0 &&
      keyframe_count % static_cast<std::size_t>(publish_every_keyframes_) == 0);
    if (!save_in_progress_.load(std::memory_order_relaxed) &&
      (loop_added || periodic_publish))
    {
      publishRebuiltMap(loop_added);
    }
  }

  void updateCorrectionLocked()
  {
    if (keyframes_.empty()) {
      return;
    }
    const auto & latest = keyframes_.back();
    const Eigen::Isometry3d correction = latest.optimized_pose * latest.local_pose.inverse();
    std::lock_guard<std::mutex> lock(correction_mutex_);
    map_from_local_ = correction;
  }

  std::vector<LoopCandidate> findCandidatesLocked(std::size_t current_index)
  {
    std::vector<LoopCandidate> candidates;
    if (current_index < static_cast<std::size_t>(std::max(1, loop_min_index_separation_))) {
      return candidates;
    }
    if (descriptor_index_->size() != current_index) {
      descriptor_index_->clear();
      for (std::size_t index = 0; index < current_index; ++index) {
        descriptor_index_->add(keyframes_[index].descriptor.range);
      }
    }
    const auto & current = keyframes_[current_index];
    std::size_t eligible_count = current_index;
    while (eligible_count > 0) {
      const std::size_t index = eligible_count - 1;
      const bool index_separated =
        current_index - index >= static_cast<std::size_t>(loop_min_index_separation_);
      const bool time_separated =
        (current.stamp - keyframes_[index].stamp).seconds() >= loop_min_time_separation_sec_;
      if (index_separated && time_separated) {
        break;
      }
      --eligible_count;
    }
    const auto nearest = descriptor_index_->nearest(
      current.descriptor.range, eligible_count,
      static_cast<std::size_t>(std::max(1, descriptor_top_k_)));
    for (const std::size_t index : nearest) {
      const double score = solid_->similarity(
        current.descriptor, keyframes_[index].descriptor);
      if (score < descriptor_min_score_) {
        continue;
      }
      candidates.push_back(
        LoopCandidate{
          index, score, solid_->yaw(current.descriptor, keyframes_[index].descriptor)});
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const LoopCandidate & lhs, const LoopCandidate & rhs) {
        return lhs.descriptor_score > rhs.descriptor_score;
      });
    return candidates;
  }

  Cloud::Ptr buildSubmapLocked(std::size_t center_index) const
  {
    auto submap = std::make_shared<Cloud>();
    const std::size_t begin = center_index > static_cast<std::size_t>(loop_submap_half_width_) ?
      center_index - loop_submap_half_width_ : 0;
    const std::size_t end = std::min(
      keyframes_.size() - 1, center_index + static_cast<std::size_t>(loop_submap_half_width_));
    const Eigen::Isometry3d center_from_map = keyframes_[center_index].optimized_pose.inverse();
    for (std::size_t index = begin; index <= end; ++index) {
      Cloud transformed;
      const Eigen::Matrix4f transform =
        (center_from_map * keyframes_[index].optimized_pose).matrix().cast<float>();
      pcl::transformPointCloud(*keyframes_[index].cloud, transformed, transform);
      *submap += transformed;
    }
    return voxelized(submap, gicp_voxel_size_);
  }

  RegistrationResult verifyCandidateLocked(
    std::size_t current_index, const LoopCandidate & candidate) const
  {
    const auto target = buildSubmapLocked(candidate.index);
    const auto source = voxelized(keyframes_[current_index].cloud, gicp_voxel_size_);
    std::vector<Eigen::Isometry3d> hypotheses;
    hypotheses.push_back(
      keyframes_[candidate.index].optimized_pose.inverse() *
      keyframes_[current_index].optimized_pose);
    hypotheses.push_back(Eigen::Isometry3d::Identity());
    for (const double yaw : {candidate.yaw_rad, -candidate.yaw_rad}) {
      Eigen::Isometry3d hypothesis = Eigen::Isometry3d::Identity();
      hypothesis.linear() = Eigen::AngleAxisd(
        yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      hypotheses.push_back(hypothesis);
    }
    return registration_->align(target, source, hypotheses);
  }

  std::vector<Keyframe> keyframeSnapshot() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return keyframes_;
  }

  std::size_t loopEdgeCount() const
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return loop_edges_.size();
  }

  static diagnostic_msgs::msg::KeyValue diagnosticValue(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = value;
    return output;
  }

  void publishDiagnostics()
  {
    std::size_t keyframes = 0;
    std::size_t loops = 0;
    int pending_confirmations = 0;
    RegistrationResult last_registration;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      keyframes = keyframes_.size();
      loops = loop_edges_.size();
      pending_confirmations = pending_loop_.valid ? pending_loop_.confirmations : 0;
      last_registration = last_registration_;
    }
    std::size_t queue_depth = 0;
    bool worker_busy = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_depth = pending_keyframes_.size();
      worker_busy = worker_busy_;
    }

    const auto now_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto last_input_ns = last_input_steady_ns_.load();
    const double input_age_sec = last_input_ns == 0 ?
      std::numeric_limits<double>::infinity() :
      static_cast<double>(now_steady_ns - last_input_ns) * 1.0e-9;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "dog_slam/loop_closure_mapper";
    status.hardware_id = "mapping_backend";
    if (worker_failure_count_.load() > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "mapping worker failure";
    } else if (last_input_ns != 0 && input_age_sec > input_stale_timeout_sec_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "synchronized cloud/odometry input is stale";
    } else if (queue_depth > 4) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "loop worker queue is accumulating";
    } else if (last_input_ns == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for synchronized cloud/odometry input";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "loop mapping healthy";
    }

    status.values = {
      diagnosticValue("cloud_topic", cloud_topic_),
      diagnosticValue("odom_topic", odom_topic_),
      diagnosticValue("synchronized_inputs", std::to_string(synchronized_input_count_.load())),
      diagnosticValue("input_age_sec", std::to_string(input_age_sec)),
      diagnosticValue("queued_keyframes", std::to_string(queued_keyframe_count_.load())),
      diagnosticValue("processed_keyframes", std::to_string(processed_keyframe_count_.load())),
      diagnosticValue("keyframes", std::to_string(keyframes)),
      diagnosticValue("worker_queue_depth", std::to_string(queue_depth)),
      diagnosticValue("worker_busy", worker_busy ? "true" : "false"),
      diagnosticValue("motion_skipped", std::to_string(motion_skipped_count_.load())),
      diagnosticValue("insufficient_points_skipped", std::to_string(point_skipped_count_.load())),
      diagnosticValue("descriptor_candidates", std::to_string(descriptor_candidate_count_.load())),
      diagnosticValue("geometric_accepts", std::to_string(geometric_accept_count_.load())),
      diagnosticValue("geometric_rejects", std::to_string(geometric_reject_count_.load())),
      diagnosticValue(
        "global_consistency_rejects",
        std::to_string(global_consistency_reject_count_.load())),
      diagnosticValue("pending_confirmations", std::to_string(pending_confirmations)),
      diagnosticValue("accepted_loops", std::to_string(loops)),
      diagnosticValue("last_loop_overlap", std::to_string(last_registration.overlap)),
      diagnosticValue("last_loop_rmse", std::to_string(last_registration.rmse)),
      diagnosticValue("last_loop_inliers", std::to_string(last_registration.num_inliers)),
      diagnosticValue("last_loop_min_eigenvalue", std::to_string(last_registration.min_eigenvalue)),
      diagnosticValue(
        "last_loop_condition_number", std::to_string(last_registration.condition_number)),
      diagnosticValue("worker_failures", std::to_string(worker_failure_count_.load()))};

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  }

  void publishRebuiltMap(bool force_reset)
  {
    const auto keyframes = keyframeSnapshot();
    if (keyframes.empty()) {
      return;
    }
    const auto stamp = now();
    const bool checkpoint_due = map_updates_since_reset_ >= 31;
    const auto rebuilt = map_rebuilder_->rebuildIncremental(
      keyframes, map_frame_, stamp, false, force_reset || checkpoint_due);
    const bool reset = force_reset || checkpoint_due || rebuilt.reset_required;
    RCLCPP_DEBUG(
      get_logger(),
      "Incremental map publish: keyframes=%zu transformed=%zu reused=%zu delta=%zu reset=%s "
      "timing_sec={update=%.3f materialize=%.3f occupancy=%.3f}",
      keyframes.size(), rebuilt.transformed_keyframes, rebuilt.reused_keyframes,
      rebuilt.delta ? rebuilt.delta->size() : 0U, reset ? "true" : "false",
      rebuilt.timings.transform_sec, rebuilt.timings.full_map_sec,
      rebuilt.timings.occupancy_sec);

    const Cloud::Ptr published_cloud = reset ? rebuilt.full_map : rebuilt.delta;
    const bool has_cloud = published_cloud && (reset || !published_cloud->empty());
    if (has_cloud || rebuilt.occupancy || rebuilt.occupancy_update) {
      sensor_msgs::msg::PointCloud2 message;
      if (has_cloud) {
        pcl::toROSMsg(*published_cloud, message);
      }
      message.header.stamp = stamp;
      message.header.frame_id = map_frame_;

      robot_interfaces::msg::MapCloudUpdate update;
      update.header = message.header;
      update.version = ++map_version_;
      update.reset = reset;
      update.has_cloud = has_cloud;
      if (reset) {
        map_base_version_ = update.version;
        map_updates_since_reset_ = 0;
      } else {
        ++map_updates_since_reset_;
      }
      update.base_version = map_base_version_;
      update.cloud = message;
      update.has_occupancy = rebuilt.occupancy.has_value();
      if (rebuilt.occupancy) {
        update.occupancy = *rebuilt.occupancy;
      }
      update.has_occupancy_update = rebuilt.occupancy_update.has_value();
      if (rebuilt.occupancy_update) {
        update.occupancy_update = *rebuilt.occupancy_update;
      }
      map_update_publisher_->publish(update);

      if (reset && has_cloud) {
        map_publisher_->publish(message);
        map_reset_publisher_->publish(message);
      }
    }
    if (rebuilt.occupancy) {
      occupancy_publisher_->publish(*rebuilt.occupancy);
    } else if (rebuilt.occupancy_update) {
      occupancy_update_publisher_->publish(*rebuilt.occupancy_update);
    }
  }

  bool waitForWorker()
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    return worker_idle_condition_.wait_for(
      lock, std::chrono::seconds(30),
      [this]() {return pending_keyframes_.empty() && !worker_busy_;});
  }

  void publishClearedOccupancy(const builtin_interfaces::msg::Time & stamp)
  {
    occupancy_publisher_->publish(
      placeholderOccupancy(map_frame_, stamp, occupancy_resolution_));
  }

  std::optional<nav_msgs::msg::OccupancyGrid> occupancyFromOrigin(
    const std::vector<Keyframe> & keyframes, const builtin_interfaces::msg::Time & stamp) const
  {
    const std::size_t origin = map_rebuilder_->occupancyOrigin();
    if (origin >= keyframes.size()) {
      return std::nullopt;
    }
    const std::vector<Keyframe> slice(keyframes.begin() + static_cast<std::ptrdiff_t>(origin),
      keyframes.end());
    return map_rebuilder_->occupancy(slice, map_frame_, stamp);
  }

  void resetOccupancyEpoch(std::size_t origin, const builtin_interfaces::msg::Time & stamp)
  {
    map_rebuilder_->resetOccupancy(origin);
    publishClearedOccupancy(stamp);
  }

  void saveCallback(
    const std::shared_ptr<robot_interfaces::srv::SavePcdMap::Request> request,
    std::shared_ptr<robot_interfaces::srv::SavePcdMap::Response> response)
  {
    SaveInProgressGuard save_guard(save_in_progress_);
    if (!save_guard.acquired()) {
      response->message = "map save is already in progress";
      return;
    }
    if (request->output_directory.empty()) {
      response->message = "output_directory is empty";
      return;
    }
    if (request->occupancy_only && request->skip_occupancy) {
      response->message = "occupancy_only and skip_occupancy cannot both be true";
      return;
    }
    if (request->occupancy_only && request->layer_snapshot) {
      response->message = "occupancy_only and layer_snapshot cannot both be true";
      return;
    }
    if (!waitForWorker()) {
      response->message = "timed out waiting for loop optimization worker";
      return;
    }
    const auto snapshot_keyframes = keyframeSnapshot();
    if (snapshot_keyframes.empty()) {
      response->message = "no optimized mapping keyframes have been collected";
      return;
    }
    const std::size_t snapshot_end = snapshot_keyframes.size();
    std::vector<Keyframe> keyframes = snapshot_keyframes;
    if (request->layer_snapshot) {
      const std::size_t layer_origin = map_rebuilder_->occupancyOrigin();
      if (layer_origin >= snapshot_end) {
        response->message = "no keyframes have been collected for the current layer";
        return;
      }
      keyframes = std::vector<Keyframe>(
        snapshot_keyframes.begin() + static_cast<std::ptrdiff_t>(layer_origin),
        snapshot_keyframes.end());
    }
    std::error_code error;
    std::filesystem::create_directories(request->output_directory, error);
    if (error) {
      response->message = "failed to create output directory: " + error.message();
      return;
    }
    const builtin_interfaces::msg::Time save_stamp = now();
    auto occupancy = request->layer_snapshot ?
      map_rebuilder_->occupancy(keyframes, map_frame_, save_stamp) :
      occupancyFromOrigin(keyframes, save_stamp);
    response->occupancy_empty = occupancyIsEmpty(occupancy);
    if (request->layer_snapshot) {
      std::vector<double> heights;
      heights.reserve(keyframes.size());
      for (const auto & keyframe : keyframes) {
        const double height = keyframe.optimized_pose.translation().z();
        if (std::isfinite(height)) {
          heights.push_back(height);
        }
      }
      if (heights.empty()) {
        response->message = "current layer has no finite optimized heights";
        return;
      }
      std::sort(heights.begin(), heights.end());
      response->layer_base_z = heights[heights.size() / 2];
      response->layer_z_tolerance = 0.60;
    }

    if (request->occupancy_only) {
      if (!writeOccupancyFiles(
          request->output_directory, occupancy, map_frame_, save_stamp, occupancy_resolution_))
      {
        response->message = "failed to write occupancy map files";
        return;
      }
      if (request->reset_occupancy) {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        resetOccupancyEpoch(snapshot_end, save_stamp);
      }
      response->success = true;
      response->message = "occupancy layer saved";
      return;
    }

    const auto rebuild_started = std::chrono::steady_clock::now();
    auto rebuilt = map_rebuilder_->rebuild(keyframes, map_frame_, save_stamp, true, false);
    const auto rebuild_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - rebuild_started).count();
    const auto & full_map = rebuilt.full_map;
    const auto & localization_map = rebuilt.localization_map;
    if (localization_map->empty()) {
      response->message = "stable localization map is empty; collect more overlapping keyframes";
      return;
    }
    const auto full_path = std::filesystem::path(request->output_directory) / "full_map.pcd";
    const auto raw_full_path =
      std::filesystem::path(request->output_directory) / "full_map_raw.pcd";
    const auto colored_path =
      std::filesystem::path(request->output_directory) / "full_map_colored.pcd";
    const auto localization_path =
      std::filesystem::path(request->output_directory) / "localization.pcd";
    const auto write_started = std::chrono::steady_clock::now();
    const auto colored_map = buildColoredMap(
      *full_map, keyframes, colored_map_voxel_size_, 96, 96, 96);
    const bool filtered_save = rebuilt.raw_full_map != full_map;
    const bool raw_write_failed = filtered_save &&
      pcl::io::savePCDFileBinary(raw_full_path.string(), *rebuilt.raw_full_map) < 0;
    if (raw_write_failed || pcl::io::savePCDFileBinary(full_path.string(), *full_map) < 0 ||
      pcl::io::savePCDFileBinaryCompressed(colored_path.string(), *colored_map.cloud) < 0 ||
      pcl::io::savePCDFileBinary(localization_path.string(), *localization_map) < 0)
    {
      response->message = "failed to write optimized PCD map files";
      return;
    }
    if (!request->skip_occupancy) {
      if (!writeOccupancyFiles(
          request->output_directory, occupancy, map_frame_, save_stamp, occupancy_resolution_))
      {
        response->message = "failed to write optimized occupancy map files";
        return;
      }
    }
    const auto write_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - write_started).count();
    sensor_msgs::msg::PointCloud2 map_message;
    pcl::toROSMsg(*rebuilt.raw_full_map, map_message);
    map_message.header.stamp = save_stamp;
    map_message.header.frame_id = map_frame_;
    map_publisher_->publish(map_message);
    if (!request->skip_occupancy) {
      occupancy_publisher_->publish(
        occupancy ? *occupancy : placeholderOccupancy(map_frame_, save_stamp, occupancy_resolution_));
    }
    if (request->reset_occupancy) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      resetOccupancyEpoch(snapshot_end, save_stamp);
    }
    response->success = true;
    response->message = "loop-optimized PCD maps saved";
    response->full_point_count = full_map->size();
    response->localization_point_count = localization_map->size();
    response->colored_point_count = colored_map.colored_point_count;
    response->colored_keyframe_count = colored_map.colored_keyframe_count;
    RCLCPP_INFO(
      get_logger(),
      "Saved optimized map: keyframes=%zu loops=%zu raw=%zu full=%zu localization=%zu "
      "colored=%zu colored_keyframes=%zu occupancy_empty=%s "
      "timing_sec={transform=%.3f full_voxel=%.3f localization=%.3f occupancy=%.3f "
      "rebuild_total=%.3f storage_write=%.3f}",
      keyframes.size(), loopEdgeCount(), rebuilt.raw_full_map->size(), full_map->size(),
      localization_map->size(),
      colored_map.colored_point_count, colored_map.colored_keyframe_count,
      response->occupancy_empty ? "true" : "false",
      rebuilt.timings.transform_sec, rebuilt.timings.full_map_sec,
      rebuilt.timings.localization_sec, rebuilt.timings.occupancy_sec,
      rebuild_sec, write_sec);
  }

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string colored_cloud_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string diagnostics_topic_;
  double diagnostics_period_sec_;
  double input_stale_timeout_sec_;
  double keyframe_translation_;
  double keyframe_rotation_rad_;
  double keyframe_voxel_size_;
  double map_keyframe_voxel_size_;
  double colored_keyframe_voxel_size_;
  double colored_map_voxel_size_;
  double color_association_tolerance_sec_;
  double color_cache_duration_sec_;
  double min_point_range_;
  double max_point_range_;
  double min_point_z_;
  double max_point_z_;
  double self_min_x_;
  double self_max_x_;
  double self_min_y_;
  double self_max_y_;
  double self_min_z_;
  double self_max_z_;
  double descriptor_min_score_;
  int descriptor_top_k_;
  int loop_min_index_separation_;
  double loop_min_time_separation_sec_;
  int loop_submap_half_width_;
  int loop_confirmations_required_;
  int loop_confirmation_index_tolerance_;
  double loop_confirmation_translation_;
  double loop_confirmation_rotation_rad_;
  double max_global_correction_translation_;
  double max_global_correction_rotation_rad_;
  double gicp_voxel_size_;
  int min_loop_inliers_;
  double occupancy_resolution_{0.05};
  int publish_every_keyframes_;
  std::uint64_t map_version_{0};
  std::uint64_t map_base_version_{0};
  std::size_t map_updates_since_reset_{0};

  std::unique_ptr<SolidDescriptorEngine> solid_;
  std::unique_ptr<DescriptorIndex> descriptor_index_;
  std::unique_ptr<LoopRegistration> registration_;
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<MapRebuilder> map_rebuilder_;

  mutable std::mutex graph_mutex_;
  std::vector<Keyframe> keyframes_;
  std::vector<std::pair<std::size_t, std::size_t>> loop_edges_;
  PendingLoop pending_loop_;
  RegistrationResult last_registration_;

  std::mutex correction_mutex_;
  Eigen::Isometry3d map_from_local_ = Eigen::Isometry3d::Identity();
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::condition_variable worker_idle_condition_;
  std::deque<Keyframe> pending_keyframes_;
  std::thread worker_;
  bool stop_worker_{false};
  bool worker_busy_{false};
  bool have_last_queued_pose_{false};
  Eigen::Isometry3d last_queued_pose_ = Eigen::Isometry3d::Identity();

  std::atomic<std::uint64_t> synchronized_input_count_{0};
  std::atomic<std::uint64_t> queued_keyframe_count_{0};
  std::atomic<std::uint64_t> processed_keyframe_count_{0};
  std::atomic<std::uint64_t> motion_skipped_count_{0};
  std::atomic<std::uint64_t> point_skipped_count_{0};
  std::atomic<std::uint64_t> descriptor_candidate_count_{0};
  std::atomic<std::uint64_t> geometric_accept_count_{0};
  std::atomic<std::uint64_t> geometric_reject_count_{0};
  std::atomic<std::uint64_t> global_consistency_reject_count_{0};
  std::atomic<std::uint64_t> worker_failure_count_{0};
  std::atomic<std::uint64_t> colored_keyframe_count_{0};
  std::atomic<std::uint64_t> dropped_color_observation_count_{0};
  std::atomic<bool> save_in_progress_{false};
  std::atomic<std::int64_t> last_input_steady_ns_{0};

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_reset_publisher_;
  rclcpp::Publisher<robot_interfaces::msg::MapCloudUpdate>::SharedPtr map_update_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_publisher_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
    occupancy_update_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<robot_interfaces::srv::SavePcdMap>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::mutex color_mutex_;
  std::deque<ColorObservation> color_observations_;
};

}  // namespace dog_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_slam::LoopClosureMapper>());
  rclcpp::shutdown();
  return 0;
}
