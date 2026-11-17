#ifndef DOG_NAV__GOAL_APPROACH_MOTION_CRITIC_HPP_
#define DOG_NAV__GOAL_APPROACH_MOTION_CRITIC_HPP_

#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{

class GoalApproachMotionCritic : public dwb_core::TrajectoryCritic
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
  double enable_distance_{0.30};
  bool near_goal_{false};
};

}  // namespace dog_nav

#endif  // DOG_NAV__GOAL_APPROACH_MOTION_CRITIC_HPP_
