#include "dog_nav/forward_progress_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double forwardProgressPenalty(
  const double forward_velocity, const double minimum_forward_speed)
{
  return std::max(0.0, minimum_forward_speed - forward_velocity);
}

bool shouldEncourageForwardProgress(
  const double goal_distance, const double heading_error,
  const double goal_distance_threshold, const double heading_error_threshold)
{
  return goal_distance > goal_distance_threshold &&
         std::abs(heading_error) < heading_error_threshold;
}

void ForwardProgressCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(
            "ForwardProgressCritic lifecycle node expired during initialization");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto get_double = [&node, &prefix](const std::string & name, const double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, prefix + name, rclcpp::ParameterValue(value));
      return node->get_parameter(prefix + name).as_double();
    };

  minimum_forward_speed_ = get_double("minimum_forward_speed", 0.21);
  goal_distance_threshold_ = get_double("goal_distance_threshold", 0.50);
  heading_error_threshold_ = get_double("heading_error_threshold", 0.70);
  lookahead_distance_ = get_double("lookahead_distance", 0.40);

  if (!std::isfinite(minimum_forward_speed_) ||
    !std::isfinite(goal_distance_threshold_) ||
    !std::isfinite(heading_error_threshold_) ||
    !std::isfinite(lookahead_distance_) ||
    minimum_forward_speed_ < 0.0 || goal_distance_threshold_ < 0.0 ||
    heading_error_threshold_ <= 0.0 || heading_error_threshold_ > kPi ||
    lookahead_distance_ <= 0.0)
  {
    throw std::runtime_error(prefix + " parameters are invalid");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Forward progress critic enabled: minimum x %.2f m/s, goal distance %.2f m, "
    "heading error %.1f deg",
    minimum_forward_speed_, goal_distance_threshold_,
    heading_error_threshold_ * 180.0 / kPi);
}

bool ForwardProgressCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  enabled_ = false;
  if (global_plan.poses.empty()) {
    return true;
  }

  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < global_plan.poses.size(); ++index) {
    const double distance = std::hypot(
      global_plan.poses[index].x - pose.x,
      global_plan.poses[index].y - pose.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  const geometry_msgs::msg::Pose2D * target = nullptr;
  for (std::size_t index = nearest_index; index < global_plan.poses.size(); ++index) {
    const auto & candidate = global_plan.poses[index];
    if (std::hypot(candidate.x - pose.x, candidate.y - pose.y) >= lookahead_distance_) {
      target = &candidate;
      break;
    }
  }
  if (target == nullptr) {
    target = &global_plan.poses.back();
  }

  const double dx = target->x - pose.x;
  const double dy = target->y - pose.y;
  if (std::hypot(dx, dy) < 0.05) {
    return true;
  }

  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const double path_heading = std::atan2(dy, dx);
  const double heading_error = std::atan2(
    std::sin(path_heading - pose.theta), std::cos(path_heading - pose.theta));
  enabled_ = shouldEncourageForwardProgress(
    goal_distance, heading_error,
    goal_distance_threshold_, heading_error_threshold_);
  return true;
}

double ForwardProgressCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (!enabled_) {
    return 0.0;
  }
  return forwardProgressPenalty(trajectory.velocity.x, minimum_forward_speed_);
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::ForwardProgressCritic, dwb_core::TrajectoryCritic)
