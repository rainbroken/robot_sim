#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using Navigate = nav2_msgs::action::NavigateToPose;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav_goal");
  const double x = node->declare_parameter("x", 1.0);
  const double y = node->declare_parameter("y", 0.0);
  const double yaw = node->declare_parameter("yaw", 0.0);
  const double timeout = node->declare_parameter("timeout", 180.0);
  auto client = rclcpp_action::create_client<Navigate>(node, "navigate_to_pose");
  if (!client->wait_for_action_server(60s)) {
    RCLCPP_ERROR(node->get_logger(), "Nav2 action server unavailable");
    rclcpp::shutdown(); return 1;
  }
  Navigate::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = x; goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw/2);
  goal.pose.pose.orientation.w = std::cos(yaw/2);
  auto handle_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, handle_future, 15s) != rclcpp::FutureReturnCode::SUCCESS || !handle_future.get()) {
    RCLCPP_ERROR(node->get_logger(), "Navigation goal rejected or response timed out");
    rclcpp::shutdown(); return 2;
  }
  auto handle = handle_future.get();
  auto result = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::duration<double>(timeout)) != rclcpp::FutureReturnCode::SUCCESS) {
    auto cancel = client->async_cancel_goal(handle);
    rclcpp::spin_until_future_complete(node, cancel, 5s);
    RCLCPP_ERROR(node->get_logger(), "Navigation timed out; cancellation requested");
    rclcpp::shutdown(); return 3;
  }
  const bool success = result.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  RCLCPP_INFO(node->get_logger(), "Navigation goal (%.2f, %.2f, %.2f): %s", x, y, yaw, success ? "SUCCEEDED" : "FAILED");
  rclcpp::shutdown();
  return success ? 0 : 4;
}
