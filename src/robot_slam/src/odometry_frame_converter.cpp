#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "dog_slam/gravity_alignment.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

Eigen::Isometry3d toEigen(const tf2::Transform & transform)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  const auto & rotation = transform.getRotation();
  result.linear() = Eigen::Quaterniond(
    rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
  const auto & origin = transform.getOrigin();
  result.translation() = Eigen::Vector3d(origin.x(), origin.y(), origin.z());
  return result;
}

tf2::Transform toTf(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond rotation(transform.linear());
  tf2::Transform result;
  result.setOrigin(
    tf2::Vector3(
      transform.translation().x(), transform.translation().y(), transform.translation().z()));
  result.setRotation(
    tf2::Quaternion(
      rotation.x(), rotation.y(), rotation.z(), rotation.w()));
  return result;
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class OdometryFrameConverter final : public rclcpp::Node
{
public:
  OdometryFrameConverter()
  : Node("odometry_frame_converter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/fastlio/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/fastlio/base_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const bool legacy_level_world = declare_parameter<bool>("level_world", false);
    leveling_mode_ = declare_parameter<std::string>(
      "leveling_mode", legacy_level_world ? "initial_base" : "disabled");
    leveling_warmup_sec_ = declare_parameter<double>("leveling_warmup_sec", 0.0);
    cloud_input_topic_ = declare_parameter<std::string>("cloud_input_topic", "");
    cloud_output_topic_ = declare_parameter<std::string>("cloud_output_topic", "");
    body_cloud_output_topic_ = declare_parameter<std::string>("body_cloud_output_topic", "");
    sensor_frame_override_ = declare_parameter<std::string>("sensor_frame_override", "");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    max_imu_odom_age_sec_ = declare_parameter<double>("max_imu_odom_age_sec", 0.1);

    if (leveling_mode_ != "disabled" && leveling_mode_ != "initial_base" &&
      leveling_mode_ != "gravity")
    {
      throw std::invalid_argument(
              "leveling_mode must be one of: disabled, initial_base, gravity");
    }
    if (max_imu_odom_age_sec_ < 0.0) {
      throw std::invalid_argument("max_imu_odom_age_sec must be non-negative");
    }

    if (leveling_mode_ == "gravity") {
      dog_slam::GravityAlignmentConfig config;
      config.minimum_stationary_duration_sec = declare_parameter<double>(
        "minimum_stationary_duration_sec", 3.0);
      const auto minimum_samples = declare_parameter<int64_t>("minimum_samples", 120);
      if (minimum_samples <= 0) {
        throw std::invalid_argument("minimum_samples must be positive");
      }
      config.minimum_samples = static_cast<std::size_t>(minimum_samples);
      config.expected_acceleration_norm = declare_parameter<double>(
        "expected_acceleration_norm", 1.0);
      config.acceleration_norm_tolerance = declare_parameter<double>(
        "acceleration_norm_tolerance", 0.15);
      config.max_gyro_norm_rad_s = declare_parameter<double>(
        "max_gyro_norm_rad_s", 0.05);
      config.max_gravity_direction_stddev_deg = declare_parameter<double>(
        "max_gravity_direction_stddev_deg", 1.0);
      config.minimum_mean_stability_duration_sec = declare_parameter<double>(
        "minimum_mean_stability_duration_sec", 2.0);
      config.max_mean_direction_change_deg = declare_parameter<double>(
        "max_mean_direction_change_deg", 1.0);
      gravity_alignment_ = std::make_unique<dog_slam::GravityAlignment>(config);
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdometryFrameConverter::collectGravity, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "Waiting for stationary IMU samples on %s before publishing mapping data",
        imu_topic_.c_str());
    } else if (leveling_mode_ == "disabled") {
      leveled_from_world_.setIdentity();
      leveling_initialized_ = true;
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdometryFrameConverter::convert, this, std::placeholders::_1));
    if (!cloud_input_topic_.empty() &&
      (!cloud_output_topic_.empty() || !body_cloud_output_topic_.empty()))
    {
      if (!cloud_output_topic_.empty()) {
        cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          cloud_output_topic_, rclcpp::SensorDataQoS());
      }
      if (!body_cloud_output_topic_.empty()) {
        body_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          body_cloud_output_topic_, rclcpp::SensorDataQoS());
      }
      cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdometryFrameConverter::convertCloud, this, std::placeholders::_1));
    }
  }

private:
  sensor_msgs::msg::PointCloud2 transformCloud(
    const sensor_msgs::msg::PointCloud2 & input, const tf2::Transform & transform,
    const std::string & frame_id)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.header.frame_id = frame_id;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(static_cast<std::size_t>(input.width) * input.height);

    sensor_msgs::PointCloud2ConstIterator<float> input_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> input_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> input_z(input, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> input_intensity;
    for (const auto & field : input.fields) {
      if (field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        input_intensity = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
          input, "intensity");
        break;
      }
    }
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> output_intensity(output, "intensity");
    for (; input_x != input_x.end();
      ++input_x, ++input_y, ++input_z, ++output_x, ++output_y, ++output_z,
      ++output_intensity)
    {
      const tf2::Vector3 point = transform * tf2::Vector3(*input_x, *input_y, *input_z);
      *output_x = static_cast<float>(point.x());
      *output_y = static_cast<float>(point.y());
      *output_z = static_cast<float>(point.z());
      *output_intensity = input_intensity ? **input_intensity : 0.0f;
      if (input_intensity) {
        ++(*input_intensity);
      }
    }
    return output;
  }

  void convert(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (message->child_frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring odometry without child_frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped base_to_sensor_message;
    const std::string sensor_frame = sensor_frame_override_.empty() ?
      message->child_frame_id : sensor_frame_override_;
    try {
      base_to_sensor_message = tf_buffer_.lookupTransform(
        base_frame_, sensor_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot convert %s pose to %s: %s",
        message->child_frame_id.c_str(), base_frame_.c_str(), error.what());
      return;
    }

    tf2::Transform world_to_sensor;
    tf2::fromMsg(message->pose.pose, world_to_sensor);
    tf2::Transform base_to_sensor;
    tf2::fromMsg(base_to_sensor_message.transform, base_to_sensor);
    const tf2::Transform world_to_base = world_to_sensor * base_to_sensor.inverse();
    latest_world_to_sensor_ = world_to_sensor;
    latest_world_to_base_ = world_to_base;
    latest_odom_stamp_sec_ = stampSeconds(message->header.stamp);
    latest_sensor_frame_ = sensor_frame;
    latest_world_to_base_valid_ = true;

    if (leveling_mode_ == "initial_base" && !leveling_initialized_) {
      const auto received_at = std::chrono::steady_clock::now();
      if (!leveling_warmup_started_) {
        leveling_warmup_started_ = true;
        leveling_warmup_started_at_ = received_at;
        RCLCPP_INFO(
          get_logger(), "Waiting %.1f s before applying legacy initial-base leveling",
          std::max(0.0, leveling_warmup_sec_));
      }
      const double warmup_elapsed = std::chrono::duration<double>(
        received_at - leveling_warmup_started_at_).count();
      if (warmup_elapsed >= std::max(0.0, leveling_warmup_sec_)) {
        leveled_from_world_ = world_to_base.inverse();
        leveling_initialized_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Using legacy initial-base leveling; initial base roll/pitch becomes map level");
      }
    }
    if (!leveling_initialized_) {
      return;
    }

    const tf2::Transform output_world_to_base = leveled_from_world_ * world_to_base;
    auto output = *message;
    output.child_frame_id = base_frame_;
    const auto origin = output_world_to_base.getOrigin();
    const auto rotation = output_world_to_base.getRotation();
    output.pose.pose.position.x = origin.x();
    output.pose.pose.position.y = origin.y();
    output.pose.pose.position.z = origin.z();
    output.pose.pose.orientation.x = rotation.x();
    output.pose.pose.orientation.y = rotation.y();
    output.pose.pose.orientation.z = rotation.z();
    output.pose.pose.orientation.w = rotation.w();
    publisher_->publish(output);
  }

  void collectGravity(const sensor_msgs::msg::Imu::ConstSharedPtr message)
  {
    if (leveling_initialized_ || !gravity_alignment_) {
      return;
    }
    if (!latest_world_to_base_valid_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for FAST-LIO odometry before collecting IMU");
      return;
    }

    const double imu_stamp_sec = stampSeconds(message->header.stamp);
    const double odom_age = std::abs(imu_stamp_sec - latest_odom_stamp_sec_);
    if (odom_age > max_imu_odom_age_sec_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for synchronized IMU and odometry (current difference %.3f s)", odom_age);
      return;
    }

    tf2::Transform sensor_from_imu;
    try {
      const auto sensor_from_imu_message = tf_buffer_.lookupTransform(
        latest_sensor_frame_, message->header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(sensor_from_imu_message.transform, sensor_from_imu);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot transform IMU frame %s to %s: %s",
        message->header.frame_id.c_str(), latest_sensor_frame_.c_str(), error.what());
      return;
    }

    const tf2::Quaternion world_from_imu_tf =
      latest_world_to_sensor_.getRotation() * sensor_from_imu.getRotation();
    const Eigen::Quaterniond world_from_imu(
      world_from_imu_tf.w(), world_from_imu_tf.x(),
      world_from_imu_tf.y(), world_from_imu_tf.z());
    const Eigen::Vector3d acceleration(
      message->linear_acceleration.x, message->linear_acceleration.y,
      message->linear_acceleration.z);
    const Eigen::Vector3d angular_velocity(
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z);

    if (!gravity_alignment_->addSample(
        imu_stamp_sec, world_from_imu, acceleration, angular_velocity))
    {
      const auto current = gravity_alignment_->stats();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Collecting stationary gravity samples: %zu samples, %.2f s%s%s",
        current.sample_count, current.duration_sec,
        gravity_alignment_->lastRejectionReason().empty() ? "" : "; last reset: ",
        gravity_alignment_->lastRejectionReason().c_str());
      return;
    }

    try {
      leveled_from_world_ = toTf(gravity_alignment_->mapFromWorld(toEigen(latest_world_to_base_)));
    } catch (const std::exception & error) {
      gravity_alignment_->reset(error.what());
      RCLCPP_ERROR(get_logger(), "Cannot lock gravity alignment: %s", error.what());
      return;
    }
    leveling_initialized_ = true;

    const auto alignment_stats = gravity_alignment_->stats();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(leveled_from_world_.getRotation()).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "Gravity alignment locked: %zu samples over %.2f s, accel norm %.4f (input units), gyro RMS %.4f "
      "rad/s, direction spread %.3f deg, up=(%.5f, %.5f, %.5f), correction "
      "rpy=(%.3f, %.3f, %.3f) deg",
      alignment_stats.sample_count, alignment_stats.duration_sec,
      alignment_stats.mean_acceleration_norm, alignment_stats.gyro_rms_rad_s,
      alignment_stats.gravity_direction_stddev_deg,
      alignment_stats.up_world.x(), alignment_stats.up_world.y(), alignment_stats.up_world.z(),
      roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }

  void convertCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (cloud_publisher_ && !leveling_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for gravity alignment before publishing the leveled cloud");
      return;
    }

    if (cloud_publisher_) {
      try {
        cloud_publisher_->publish(
          transformCloud(*message, leveled_from_world_, message->header.frame_id));
      } catch (const std::runtime_error & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Cannot level registered cloud: %s", error.what());
        return;
      }
    }
    if (body_cloud_publisher_ && latest_world_to_base_valid_) {
      try {
        body_cloud_publisher_->publish(
          transformCloud(*message, latest_world_to_base_.inverse(), base_frame_));
      } catch (const std::runtime_error & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Cannot convert registered cloud to %s: %s",
          base_frame_.c_str(), error.what());
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string leveling_mode_;
  std::string cloud_input_topic_;
  std::string cloud_output_topic_;
  std::string body_cloud_output_topic_;
  std::string sensor_frame_override_;
  std::string imu_topic_;
  std::string latest_sensor_frame_;
  double leveling_warmup_sec_{0.0};
  double max_imu_odom_age_sec_{0.1};
  double latest_odom_stamp_sec_{0.0};
  bool leveling_warmup_started_{false};
  bool leveling_initialized_{false};
  bool latest_world_to_base_valid_{false};
  std::chrono::steady_clock::time_point leveling_warmup_started_at_;
  tf2::Transform leveled_from_world_;
  tf2::Transform latest_world_to_sensor_;
  tf2::Transform latest_world_to_base_;
  std::unique_ptr<dog_slam::GravityAlignment> gravity_alignment_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryFrameConverter>());
  rclcpp::shutdown();
  return 0;
}
