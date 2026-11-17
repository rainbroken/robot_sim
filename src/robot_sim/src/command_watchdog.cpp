#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

// Classic diff_drive retains its last command. Stop on a dead publisher even if
// simulated time pauses, and enforce the non-holonomic command interface.
class CommandWatchdog : public rclcpp::Node {
public:
  CommandWatchdog() : Node("command_watchdog") {
    declare_parameter("require_localization", false);
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/navigation/localization_valid", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        localized_ = msg->data;
        localization_stamp_ = std::chrono::steady_clock::now();
      });
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd_ = geometry_msgs::msg::Twist();
        if (std::isfinite(msg->linear.x) && std::isfinite(msg->angular.z)) {
          cmd_.linear.x = std::clamp(msg->linear.x, -0.2, 0.4);
          cmd_.angular.z = std::clamp(msg->angular.z, -0.8, 0.8);
        }
        last_ = std::chrono::steady_clock::now();
      });
    timer_ = create_wall_timer(50ms, [this] {
      const auto stamp = std::chrono::steady_clock::now();
      const bool localization_ok = !get_parameter("require_localization").as_bool() ||
        (localized_ && stamp - localization_stamp_ < 800ms);
      pub_->publish(localization_ok && stamp - last_ < 600ms ? cmd_ : geometry_msgs::msg::Twist());
    });
  }
private:
  geometry_msgs::msg::Twist cmd_;
  std::chrono::steady_clock::time_point last_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool localized_{false};
  std::chrono::steady_clock::time_point localization_stamp_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandWatchdog>());
  rclcpp::shutdown();
  return 0;
}
