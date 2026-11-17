#ifndef DOG_NAV__STABLE_ROTATE_TO_GOAL_CRITIC_HPP_
#define DOG_NAV__STABLE_ROTATE_TO_GOAL_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{

enum class FinalPoseMode
{
  APPROACH,
  SLOWING,
  ROTATING,
};

FinalPoseMode updateFinalPoseMode(
  FinalPoseMode mode, bool goal_changed, double goal_distance,
  double heading_error, double enter_distance,
  double position_tolerance, double exit_distance,
  double reposition_yaw_tolerance,
  double current_speed, double stopped_velocity);

double selectRotationDirection(double previous_direction, double heading_error);

bool isStableGoalVelocityAllowed(
  FinalPoseMode mode, double current_speed,
  double x, double y, double theta,
  double rotation_direction, double heading_error,
  double yaw_tolerance, double min_rotation_speed,
  double max_rotation_speed);

class StableRotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  double scoreRotation(const dwb_msgs::msg::Trajectory2D & trajectory) const;

  FinalPoseMode mode_{FinalPoseMode::APPROACH};
  bool have_goal_{false};
  geometry_msgs::msg::Pose2D previous_goal_;
  double current_speed_{0.0};
  double goal_yaw_{0.0};
  double heading_error_{0.0};
  double rotation_direction_{0.0};

  double enter_distance_{0.13};
  double position_tolerance_{0.15};
  double exit_distance_{0.20};
  double reposition_yaw_tolerance_{0.30};
  double stopped_velocity_{0.08};
  double yaw_tolerance_{0.15};
  double min_rotation_speed_{0.21};
  double max_rotation_speed_{0.50};
  double slowing_factor_{4.0};
  double goal_change_position_tolerance_{0.05};
  double goal_change_yaw_tolerance_{0.10};
};

}  // namespace dog_nav

#endif  // DOG_NAV__STABLE_ROTATE_TO_GOAL_CRITIC_HPP_
