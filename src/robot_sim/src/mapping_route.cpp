#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// A deterministic scene-specific tour for map acquisition, not an exploration planner.
// Laser clearance and stale-data guards stop the robot instead of pushing through obstacles.
class MappingRoute : public rclcpp::Node {
public:
  MappingRoute() : Node("mapping_route") {
    points_ = declare_parameter<std::vector<double>>("waypoints", {0.0, 0.0});
    speed_ = declare_parameter("speed", 0.30);
    if (points_.empty() || points_.size() % 2 != 0 || speed_ <= 0.0 || speed_ > 0.4) {
      throw std::invalid_argument("waypoints must contain x/y pairs; speed must be in (0, 0.4]");
    }
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        clearance_ = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
          const double a = msg->angle_min + i * msg->angle_increment;
          const double r = msg->ranges[i];
          if (std::abs(a) < 0.4 && std::isfinite(r) && r >= msg->range_min) {
            clearance_ = std::min(clearance_, r);
          }
        }
        scan_stamp_ = std::chrono::steady_clock::now(); have_scan_ = true;
      });
    timer_ = create_wall_timer(50ms, [this] { tick(); });
  }
  bool done() const { return done_; }
  bool failed() const { return failed_; }
  void stop() { pub_->publish(geometry_msgs::msg::Twist()); }
private:
  void tick() {
    geometry_msgs::msg::Twist cmd;
    const auto steady = std::chrono::steady_clock::now();
    try {
      const auto pose = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      if ((now() - rclcpp::Time(pose.header.stamp)).seconds() > 1.0) { stop(); return; }
      x_ = pose.transform.translation.x; y_ = pose.transform.translation.y;
      const auto &q = pose.transform.rotation;
      yaw_ = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
    } catch (const tf2::TransformException &) {
      stop(); return;
    }
    if (!have_scan_ || steady - scan_stamp_ > 2s) {
      stop(); return;
    }
    if (index_ >= points_.size()) {
      stop(); done_ = true; return;
    }
    const double dx = points_[index_] - x_, dy = points_[index_ + 1] - y_;
    const double distance = std::hypot(dx, dy);
    if (distance < 0.12) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu/%zu (%.2f, %.2f)",
                  index_/2 + 1, points_.size()/2, x_, y_);
      index_ += 2; blocked_ = false; stop(); return;
    }
    const double angle = std::atan2(dy, dx) - yaw_;
    const double error = std::atan2(std::sin(angle), std::cos(angle));
    cmd.angular.z = std::clamp(1.5 * error, -0.65, 0.65);
    if (std::abs(error) < 0.18) {
      if (clearance_ < 0.40) {
        stop();
        if (!blocked_) { blocked_since_ = steady; blocked_ = true; }
        if (steady - blocked_since_ > 15s) {
          RCLCPP_ERROR(get_logger(), "Route blocked before waypoint %zu; stopped. Clearance %.2f m", index_/2 + 1, clearance_);
          failed_ = true; done_ = true;
        }
        return;
      }
      blocked_ = false;
      cmd.linear.x = std::min(speed_, 0.8 * distance);
    }
    pub_->publish(cmd);
  }
  std::vector<double> points_;
  size_t index_{0};
  double speed_{0.3}, x_{0}, y_{0}, yaw_{0}, clearance_{0};
  bool have_scan_{false}, blocked_{false}, done_{false}, failed_{false};
  std::chrono::steady_clock::time_point scan_stamp_, blocked_since_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingRoute>();
  const auto deadline = std::chrono::steady_clock::now() + 30min;
  while (rclcpp::ok() && !node->done() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node); std::this_thread::sleep_for(10ms);
  }
  if (rclcpp::ok()) { node->stop(); }
  const bool success = node->done() && !node->failed();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
