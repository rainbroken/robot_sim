#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>;

Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  rotation = rotation.norm() < 1.0e-9 ? Eigen::Quaterniond::Identity() : rotation.normalized();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
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
}  // namespace

class MappingTfBridge final : public rclcpp::Node
{
public:
  MappingTfBridge()
  : Node("mapping_tf_bridge"), broadcaster_(*this)
  {
    fastlio_odom_topic_ = declare_parameter<std::string>(
      "fastlio_odom_topic", "/fastlio/base_odom");
    dog_odom_topic_ = declare_parameter<std::string>("dog_odom_topic", "/odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    planar_navigation_tf_ = declare_parameter<bool>("planar_navigation_tf", true);

    fastlio_subscriber_.subscribe(this, fastlio_odom_topic_, rmw_qos_profile_sensor_data);
    dog_subscriber_.subscribe(this, dog_odom_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(30), fastlio_subscriber_, dog_subscriber_);
    synchronizer_->registerCallback(
      std::bind(&MappingTfBridge::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & fastlio,
    const nav_msgs::msg::Odometry::ConstSharedPtr & dog)
  {
    const Eigen::Isometry3d map_from_base = poseToEigen(fastlio->pose.pose);
    const Eigen::Isometry3d odom_from_base = poseToEigen(dog->pose.pose);
    const Eigen::Isometry3d map_from_odom = planar_navigation_tf_ ?
      planarPose(map_from_base) * planarPose(odom_from_base).inverse() :
      map_from_base * odom_from_base.inverse();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = dog->header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = eigenToTransform(map_from_odom);
    broadcaster_.sendTransform(transform);
  }

  std::string fastlio_odom_topic_;
  std::string dog_odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  bool planar_navigation_tf_;
  tf2_ros::TransformBroadcaster broadcaster_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> fastlio_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> dog_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingTfBridge>());
  rclcpp::shutdown();
  return 0;
}
