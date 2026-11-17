#include "dog_nav/stable_rotate_to_goal_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "dog_nav/rotate_to_path_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

namespace
{
constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kVelocityEpsilon = 1.0e-6;

const char * modeName(const FinalPoseMode mode)
{
  switch (mode) {
    case FinalPoseMode::APPROACH:
      return "APPROACH";
    case FinalPoseMode::SLOWING:
      return "SLOWING";
    case FinalPoseMode::ROTATING:
      return "ROTATING";
  }
  return "UNKNOWN";
}
}  // namespace

FinalPoseMode updateFinalPoseMode(
  FinalPoseMode mode, const bool goal_changed, const double goal_distance,
  const double heading_error, const double enter_distance,
  const double position_tolerance, const double exit_distance,
  const double reposition_yaw_tolerance,
  const double current_speed, const double stopped_velocity)
{
  if (goal_changed) {
    mode = FinalPoseMode::APPROACH;
  }

  if (mode != FinalPoseMode::APPROACH && goal_distance > exit_distance) {
    mode = FinalPoseMode::APPROACH;
  }
  if (mode == FinalPoseMode::ROTATING && goal_distance > position_tolerance &&
    std::abs(heading_error) <= reposition_yaw_tolerance)
  {
    mode = FinalPoseMode::APPROACH;
  }
  if (mode == FinalPoseMode::APPROACH && goal_distance <= enter_distance) {
    mode = FinalPoseMode::SLOWING;
  }
  if (mode == FinalPoseMode::SLOWING && current_speed <= stopped_velocity) {
    mode = FinalPoseMode::ROTATING;
  }
  return mode;
}

double selectRotationDirection(
  const double previous_direction, const double heading_error)
{
  if (std::abs(heading_error) < 1.0e-9) {
    return previous_direction;
  }
  const double requested_direction = std::copysign(1.0, heading_error);
  if (previous_direction == 0.0) {
    return requested_direction;
  }
  // Around +/- pi, angle normalization can change sign without the robot
  // crossing the goal heading. Keep the chosen direction until the remaining
  // error is under 90 degrees; after that, permit an overshoot correction.
  if (std::abs(heading_error) < kHalfPi) {
    return requested_direction;
  }
  return previous_direction;
}

double finalRotationScore(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double goal_yaw, const double lookahead_time)
{
  if (trajectory.poses.empty()) {
    throw std::invalid_argument("Cannot score an empty rotation trajectory");
  }
  double yaw = trajectory.poses.back().theta;
  // Some installed StandardTrajectoryGenerator versions append a duplicate
  // final pose without its timestamp. Bound both arrays rather than indexing
  // projectPose() with mismatched lengths. Missing timestamps keep legacy scoring.
  const std::size_t count = std::min(trajectory.poses.size(), trajectory.time_offsets.size());
  if (lookahead_time >= 0.0 && count > 0) {
    yaw = trajectory.poses[count - 1].theta;
    for (std::size_t i = 0; i < count; ++i) {
      const double time = rclcpp::Duration(trajectory.time_offsets[i]).seconds();
      if (time < lookahead_time) {continue;}
      yaw = trajectory.poses[i].theta;
      if (i > 0) {
        const double previous_time = rclcpp::Duration(trajectory.time_offsets[i - 1]).seconds();
        if (time > previous_time) {
          const double fraction = std::clamp(
            (lookahead_time - previous_time) / (time - previous_time), 0.0, 1.0);
          yaw = trajectory.poses[i - 1].theta + fraction * normalizeAngle(
            trajectory.poses[i].theta - trajectory.poses[i - 1].theta);
        }
      }
      break;
    }
  }
  return std::abs(normalizeAngle(goal_yaw - yaw));
}

bool isStableGoalVelocityAllowed(
  const FinalPoseMode mode, const double current_speed,
  const double x, const double y, const double theta,
  const double rotation_direction, const double heading_error,
  const double yaw_tolerance, const double min_rotation_speed,
  const double max_rotation_speed)
{
  if (mode == FinalPoseMode::APPROACH) {
    return true;
  }

  const double candidate_speed = std::hypot(x, y);
  if (mode == FinalPoseMode::SLOWING) {
    if (candidate_speed + kVelocityEpsilon >= current_speed ||
      std::abs(theta) > max_rotation_speed + kVelocityEpsilon)
    {
      return false;
    }
    return std::abs(theta) <= kVelocityEpsilon || theta * rotation_direction > 0.0;
  }

  if (candidate_speed > kVelocityEpsilon) {
    return false;
  }
  // A zero command must remain legal so the dynamic window can brake an
  // existing wrong-direction or over-limit angular velocity before turning.
  if (std::abs(theta) <= kVelocityEpsilon) {
    return true;
  }
  if (std::abs(heading_error) <= yaw_tolerance) {
    return false;
  }
  if (std::abs(theta) + kVelocityEpsilon < min_rotation_speed ||
    std::abs(theta) > max_rotation_speed + kVelocityEpsilon)
  {
    return false;
  }
  return theta * rotation_direction > 0.0;
}

void StableRotateToGoalCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error(
            "StableRotateToGoalCritic lifecycle node expired during initialization");
  }

  const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
  const auto get_double = [&node, &prefix](const std::string & name, const double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, prefix + name, rclcpp::ParameterValue(value));
      return node->get_parameter(prefix + name).as_double();
    };

  enter_distance_ = get_double("enter_distance", 0.13);
  position_tolerance_ = get_double("position_tolerance", 0.15);
  exit_distance_ = get_double("exit_distance", 0.20);
  reposition_yaw_tolerance_ = get_double("reposition_yaw_tolerance", 0.30);
  yaw_tolerance_ = get_double("yaw_tolerance", 0.15);
  min_rotation_speed_ = get_double("min_rotation_speed", 0.21);
  max_rotation_speed_ = get_double("max_rotation_speed", 0.50);
  slowing_factor_ = get_double("slowing_factor", 4.0);
  lookahead_time_ = get_double("lookahead_time", -1.0);
  approach_angular_scale_ = get_double("approach_angular_scale", 0.0);
  approach_distance_ = get_double("approach_distance", 0.30);
  goal_change_position_tolerance_ = get_double("goal_change_position_tolerance", 0.05);
  goal_change_yaw_tolerance_ = get_double("goal_change_yaw_tolerance", 0.10);

  const std::string stopped_velocity_parameter =
    dwb_plugin_name_ + ".trans_stopped_velocity";
  nav2_util::declare_parameter_if_not_declared(
    node, stopped_velocity_parameter, rclcpp::ParameterValue(0.08));
  stopped_velocity_ = node->get_parameter(stopped_velocity_parameter).as_double();

  if (!std::isfinite(enter_distance_) || !std::isfinite(position_tolerance_) ||
    !std::isfinite(exit_distance_) || !std::isfinite(reposition_yaw_tolerance_) ||
    !std::isfinite(stopped_velocity_) || !std::isfinite(yaw_tolerance_) ||
    !std::isfinite(min_rotation_speed_) || !std::isfinite(max_rotation_speed_) ||
    !std::isfinite(lookahead_time_) || !std::isfinite(approach_angular_scale_) ||
    !std::isfinite(approach_distance_) || !std::isfinite(slowing_factor_) ||
    enter_distance_ <= 0.0 || position_tolerance_ < enter_distance_ ||
    exit_distance_ <= position_tolerance_ || reposition_yaw_tolerance_ <= yaw_tolerance_ ||
    stopped_velocity_ < 0.0 || yaw_tolerance_ <= 0.0 ||
    min_rotation_speed_ < 0.0 || max_rotation_speed_ < min_rotation_speed_ ||
    slowing_factor_ < 0.0 || goal_change_position_tolerance_ < 0.0 ||
    approach_angular_scale_ < 0.0 || approach_distance_ < position_tolerance_ ||
    goal_change_yaw_tolerance_ < 0.0)
  {
    throw std::runtime_error(prefix + " parameters are invalid");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Stable final-pose critic: enter %.2f m, target %.2f m, hard exit %.2f m, "
    "stopped %.2f m/s, "
    "rotation %.2f..%.2f rad/s",
    enter_distance_, position_tolerance_, exit_distance_, stopped_velocity_,
    min_rotation_speed_, max_rotation_speed_);
}

void StableRotateToGoalCritic::reset()
{
  // DWB calls reset() whenever the 1 Hz global planner supplies a new path.
  // Final-pose state is intentionally retained here and reset in prepare()
  // only when the actual navigation goal changes.
}

bool StableRotateToGoalCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  const bool goal_changed = !have_goal_ ||
    std::hypot(goal.x - previous_goal_.x, goal.y - previous_goal_.y) >
    goal_change_position_tolerance_ ||
    std::abs(normalizeAngle(goal.theta - previous_goal_.theta)) >
    goal_change_yaw_tolerance_;

  previous_goal_ = goal;
  have_goal_ = true;
  goal_yaw_ = goal.theta;
  heading_error_ = normalizeAngle(goal_yaw_ - pose.theta);
  current_speed_ = std::hypot(velocity.x, velocity.y);
  current_theta_ = velocity.theta;
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  goal_distance_ = goal_distance;

  const FinalPoseMode previous_mode = mode_;
  mode_ = updateFinalPoseMode(
    mode_, goal_changed, goal_distance, heading_error_,
    enter_distance_, position_tolerance_, exit_distance_, reposition_yaw_tolerance_,
    current_speed_, stopped_velocity_);

  if (mode_ == FinalPoseMode::APPROACH) {
    rotation_direction_ = 0.0;
  } else {
    rotation_direction_ = selectRotationDirection(rotation_direction_, heading_error_);
  }

  if (mode_ != previous_mode) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_INFO(
        node->get_logger(),
        "Stable final-pose mode %s -> %s (distance %.3f m, speed %.3f m/s, yaw error %.1f deg)",
        modeName(previous_mode), modeName(mode_), goal_distance, current_speed_,
        heading_error_ * 180.0 / 3.14159265358979323846);
    }
  }
  return true;
}

double StableRotateToGoalCritic::scoreRotation(
  const dwb_msgs::msg::Trajectory2D & trajectory) const
{
  if (trajectory.poses.empty()) {
    throw dwb_core::IllegalTrajectoryException(name_, "empty trajectory");
  }
  return finalRotationScore(trajectory, goal_yaw_, lookahead_time_);
}

double StableRotateToGoalCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  const auto & velocity = trajectory.velocity;
  if (!isStableGoalVelocityAllowed(
      mode_, current_speed_, velocity.x, velocity.y, velocity.theta,
      rotation_direction_, heading_error_, yaw_tolerance_,
      min_rotation_speed_, max_rotation_speed_))
  {
    throw dwb_core::IllegalTrajectoryException(
            name_, "trajectory is incompatible with stable final-pose mode");
  }

  if (mode_ == FinalPoseMode::APPROACH) {
    if (goal_distance_ <= approach_distance_) {
      const double change = velocity.theta - current_theta_;
      // Suppress grid-quantization driven steering flips during the last
      // approach, but leave curves and obstacle avoidance available.
      return approach_angular_scale_ * (velocity.theta * velocity.theta + change * change);
    }
    return 0.0;
  }
  const double rotation_score = scoreRotation(trajectory);
  if (mode_ == FinalPoseMode::SLOWING) {
    const double speed = std::hypot(velocity.x, velocity.y);
    return slowing_factor_ * speed * speed + rotation_score;
  }
  return rotation_score;
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::StableRotateToGoalCritic, dwb_core::TrajectoryCritic)
