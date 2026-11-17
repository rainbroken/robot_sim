// Project a tilted sparse lidar into a ground-aligned virtual LaserScan.
// Unobserved bins are NaN: absence of a non-repetitive ray is NOT free space.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CloudToScan : public rclcpp::Node {
public:
  CloudToScan() : Node("cloud_to_scan"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter<std::string>("target_frame", "scan_link");
    min_height_ = declare_parameter("min_height", 0.08);
    max_height_ = declare_parameter("max_height", 0.85);
    range_min_ = declare_parameter("range_min", 0.25);
    range_max_ = declare_parameter("range_max", 25.0);
    bins_ = declare_parameter("bins", 720);
    if (bins_ < 36 || bins_ > 7200 || min_height_ >= max_height_ ||
        range_min_ <= 0 || range_max_ <= range_min_) throw std::invalid_argument("Invalid projection parameters");
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { project(*cloud); });
  }
private:
  void project(const sensor_msgs::msg::PointCloud2 & cloud) {
    tf2::Transform to_scan, to_base;
    try {
      tf2::fromMsg(buffer_.lookupTransform(frame_, cloud.header.frame_id,
                      rclcpp::Time(cloud.header.stamp)).transform, to_scan);
      tf2::fromMsg(buffer_.lookupTransform("base_footprint", frame_,
                      rclcpp::Time(cloud.header.stamp)).transform, to_base);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for sensor TF: %s", e.what()); return;
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud.header; scan.header.frame_id = frame_;
    constexpr double pi = 3.14159265358979323846;
    scan.angle_min = -pi; scan.angle_increment = 2.0 * pi / bins_;
    scan.angle_max = scan.angle_min + (bins_ - 1) * scan.angle_increment;
    scan.range_min = range_min_; scan.range_max = range_max_;
    scan.scan_time = 0.1; scan.time_increment = 0.0;  // instantaneous ray snapshot
    scan.ranges.assign(bins_, std::numeric_limits<float>::quiet_NaN());
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const auto p = to_scan * tf2::Vector3(*x, *y, *z);
        if (p.z() < min_height_ || p.z() > max_height_) continue;
        const auto base = to_base * p;
        // Neutral arm pose/chassis envelope plus margin. No arm actuation in navigation.
        if (std::abs(base.x()) < 0.26 && std::abs(base.y()) < 0.26) continue;
        const auto range = std::hypot(p.x(), p.y());
        if (range < range_min_ || range > range_max_) continue;
        const auto bin = static_cast<int>(std::floor((std::atan2(p.y(), p.x()) + pi) / scan.angle_increment)) % bins_;
        auto & previous = scan.ranges[bin];
        if (!std::isfinite(previous) || range < previous) previous = static_cast<float>(range);
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Malformed cloud: %s", e.what()); return;
    }
    pub_->publish(scan);
  }
  std::string frame_;
  double min_height_, max_height_, range_min_, range_max_;
  int bins_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToScan>());
  rclcpp::shutdown();
}
