#include "dog_nav/goal_approach_motion_critic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

void GoalApproachMotionCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GoalApproachMotionCritic lifecycle node expired during initialization");
  }
  const std::string parameter = dwb_plugin_name_ + "." + name_ + ".enable_distance";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter, rclcpp::ParameterValue(0.30));
  enable_distance_ = node->get_parameter(parameter).as_double();
  if (!std::isfinite(enable_distance_) || enable_distance_ < 0.0) {
    throw std::runtime_error(parameter + " must be finite and non-negative");
  }
  RCLCPP_INFO(
    node->get_logger(), "Reverse and lateral navigation allowed only within %.2f m of goal",
    enable_distance_);
}

bool GoalApproachMotionCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  // Use the current pose and actual path goal supplied by DWB, rather than
  // asynchronous action feedback or the endpoint of a sampled trajectory.
  const double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  near_goal_ = std::isfinite(distance) && distance <= enable_distance_;
  return true;
}

double GoalApproachMotionCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  constexpr double epsilon = 1.0e-6;
  if (!near_goal_ &&
    (trajectory.velocity.x < -epsilon || std::abs(trajectory.velocity.y) > epsilon))
  {
    throw dwb_core::IllegalTrajectoryException(
            name_, "reverse and lateral motion are restricted to the goal approach area");
  }
  return 0.0;
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::GoalApproachMotionCritic, dwb_core::TrajectoryCritic)
