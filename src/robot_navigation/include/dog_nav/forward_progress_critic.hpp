#ifndef DOG_NAV__FORWARD_PROGRESS_CRITIC_HPP_
#define DOG_NAV__FORWARD_PROGRESS_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{

double forwardProgressPenalty(double forward_velocity, double minimum_forward_speed);

bool shouldEncourageForwardProgress(
  double goal_distance, double heading_error,
  double goal_distance_threshold, double heading_error_threshold);

class ForwardProgressCritic : public dwb_core::TrajectoryCritic
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
  bool enabled_{false};
  double minimum_forward_speed_{0.21};
  double goal_distance_threshold_{0.50};
  double heading_error_threshold_{0.70};
  double lookahead_distance_{0.40};
};

}  // namespace dog_nav

#endif  // DOG_NAV__FORWARD_PROGRESS_CRITIC_HPP_
