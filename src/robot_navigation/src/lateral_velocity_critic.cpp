#include "dog_nav/lateral_velocity_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

double lateralVelocityPenalty(
  const double lateral_velocity, const double deadband)
{
  return std::max(0.0, std::abs(lateral_velocity) - deadband);
}

bool shouldPenalizeLateralVelocity(
  const double goal_distance, const double goal_distance_threshold)
{
  return goal_distance > goal_distance_threshold;
}

void LateralVelocityCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(
            "LateralVelocityCritic lifecycle node expired during initialization");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto get_double = [&node, &prefix](const std::string & name, const double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, prefix + name, rclcpp::ParameterValue(value));
      return node->get_parameter(prefix + name).as_double();
    };

  deadband_ = get_double("deadband", 0.03);
  goal_distance_threshold_ = get_double("goal_distance_threshold", 0.50);

  if (!std::isfinite(deadband_) || !std::isfinite(goal_distance_threshold_) ||
    deadband_ < 0.0 || goal_distance_threshold_ < 0.0)
  {
    throw std::runtime_error(prefix + " parameters are invalid");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "LateralVelocity critic enabled: deadband %.2f m/s, disabled within %.2f m of goal",
    deadband_, goal_distance_threshold_);
}

bool LateralVelocityCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  enabled_ = shouldPenalizeLateralVelocity(
    goal_distance, goal_distance_threshold_);
  return true;
}

double LateralVelocityCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!enabled_) {
    return 0.0;
  }
  return lateralVelocityPenalty(trajectory.velocity.y, deadband_);
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::LateralVelocityCritic, dwb_core::TrajectoryCritic)
