#include "dog_nav/rotate_to_path_critic.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool updateRotateToPathState(
  const bool active, const double heading_error, const double goal_distance,
  const double enter_angle, const double exit_angle,
  const double goal_distance_threshold)
{
  if (goal_distance <= goal_distance_threshold) {
    return false;
  }
  const double absolute_error = std::abs(heading_error);
  if (active) {
    return absolute_error > exit_angle;
  }
  return absolute_error >= enter_angle;
}

bool isRotateToPathVelocityAllowed(
  const bool active, const double heading_error,
  const double x, const double y, const double theta,
  const double max_translation_speed, const double min_rotation_speed,
  const double current_theta)
{
  if (!active) {
    return true;
  }
  if (std::hypot(x, y) > max_translation_speed) {
    return false;
  }
  // A zero command is only a braking sample while already rotating. From rest
  // it can otherwise outscore every command that actually performs the turn.
  if (std::abs(theta) <= 1.0e-6) {
    return std::abs(current_theta) >= min_rotation_speed;
  }
  if (std::abs(theta) < min_rotation_speed) {
    return false;
  }
  return theta * heading_error > 0.0;
}

void RotateToPathCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RotateToPathCritic lifecycle node expired during initialization");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto get_double = [&node, &prefix](const std::string & name, const double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, prefix + name, rclcpp::ParameterValue(value));
      return node->get_parameter(prefix + name).as_double();
    };

  enter_angle_ = get_double("enter_angle", 0.65);
  exit_angle_ = get_double("exit_angle", 0.30);
  lookahead_distance_ = get_double("lookahead_distance", 0.40);
  goal_distance_threshold_ = get_double("goal_distance_threshold", 0.50);
  goal_change_tolerance_ = get_double("goal_change_tolerance", 0.10);
  max_translation_speed_ = get_double("max_translation_speed", 0.03);
  min_rotation_speed_ = get_double("min_rotation_speed", 0.20);

  if (!std::isfinite(enter_angle_) || !std::isfinite(exit_angle_) ||
    exit_angle_ <= 0.0 || enter_angle_ <= exit_angle_ || enter_angle_ > kPi ||
    lookahead_distance_ <= 0.0 || goal_distance_threshold_ < 0.0 ||
    goal_change_tolerance_ < 0.0 || max_translation_speed_ < 0.0 ||
    min_rotation_speed_ < 0.0)
  {
    throw std::runtime_error(prefix + " parameters are invalid");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "RotateToPath critic enabled: enter %.1f deg, exit %.1f deg, disabled within %.2f m of goal",
    enter_angle_ * 180.0 / kPi, exit_angle_ * 180.0 / kPi,
    goal_distance_threshold_);
}

bool RotateToPathCritic::findPathHeading(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Path2D & global_plan,
  double & path_heading) const
{
  if (global_plan.poses.empty()) {
    return false;
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
    return false;
  }
  path_heading = std::atan2(dy, dx);
  return true;
}

bool RotateToPathCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  current_theta_ = velocity.theta;
  if (!have_goal_ ||
    std::hypot(goal.x - previous_goal_.x, goal.y - previous_goal_.y) > goal_change_tolerance_ ||
    std::abs(normalizeAngle(goal.theta - previous_goal_.theta)) > 0.20)
  {
    active_ = false;
    previous_goal_ = goal;
    have_goal_ = true;
  }

  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  double candidate_heading = 0.0;
  if (!findPathHeading(pose, global_plan, candidate_heading)) {
    active_ = false;
    return true;
  }

  path_heading_ = candidate_heading;
  heading_error_ = normalizeAngle(path_heading_ - pose.theta);
  active_ = updateRotateToPathState(
    active_, heading_error_, goal_distance,
    enter_angle_, exit_angle_, goal_distance_threshold_);
  return true;
}

double RotateToPathCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  const auto & velocity = trajectory.velocity;
  if (!isRotateToPathVelocityAllowed(
      active_, heading_error_, velocity.x, velocity.y, velocity.theta,
      max_translation_speed_, min_rotation_speed_, current_theta_))
  {
    throw dwb_core::IllegalTrajectoryException(
            name_, "trajectory does not perform the required in-place turn toward the path");
  }

  if (!active_ || trajectory.poses.empty()) {
    return 0.0;
  }
  return std::abs(normalizeAngle(path_heading_ - trajectory.poses.back().theta));
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::RotateToPathCritic, dwb_core::TrajectoryCritic)
