#ifndef DOG_NAV__LATERAL_VELOCITY_CRITIC_HPP_
#define DOG_NAV__LATERAL_VELOCITY_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{

double lateralVelocityPenalty(double lateral_velocity, double deadband);

bool shouldPenalizeLateralVelocity(
  double goal_distance, double goal_distance_threshold);

class LateralVelocityCritic : public dwb_core::TrajectoryCritic
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
  bool enabled_{true};
  double deadband_{0.03};
  double goal_distance_threshold_{0.50};
};

}  // namespace dog_nav

#endif  // DOG_NAV__LATERAL_VELOCITY_CRITIC_HPP_
