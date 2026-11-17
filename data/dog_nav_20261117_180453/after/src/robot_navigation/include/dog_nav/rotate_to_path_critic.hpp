#ifndef DOG_NAV__ROTATE_TO_PATH_CRITIC_HPP_
#define DOG_NAV__ROTATE_TO_PATH_CRITIC_HPP_

#include <limits>

#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{

double normalizeAngle(double angle);

bool updateRotateToPathState(
  bool active, double heading_error, double goal_distance,
  double enter_angle, double exit_angle, double goal_distance_threshold);

bool isRotateToPathVelocityAllowed(
  bool active, double heading_error, double x, double y, double theta,
  double max_translation_speed, double min_rotation_speed,
  double current_theta = 0.0);

class RotateToPathCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  bool findPathHeading(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Path2D & global_plan,
    double & path_heading) const;

  bool active_{false};
  bool have_goal_{false};
  geometry_msgs::msg::Pose2D previous_goal_;
  double path_heading_{0.0};
  double heading_error_{0.0};
  double current_theta_{0.0};

  double enter_angle_{0.65};
  double exit_angle_{0.30};
  double lookahead_distance_{0.40};
  double goal_distance_threshold_{0.50};
  double goal_change_tolerance_{0.10};
  double max_translation_speed_{0.03};
  double min_rotation_speed_{0.20};
};

}  // namespace dog_nav

#endif  // DOG_NAV__ROTATE_TO_PATH_CRITIC_HPP_
