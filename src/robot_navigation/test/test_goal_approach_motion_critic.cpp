#include <limits>
#include <memory>

#include "dog_nav/goal_approach_motion_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

namespace
{
dwb_msgs::msg::Trajectory2D motion(const double x, const double y, const double theta = 0.0)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = x;
  trajectory.velocity.y = y;
  trajectory.velocity.theta = theta;
  return trajectory;
}

void prepareAtDistance(dog_nav::GoalApproachMotionCritic & critic, const double distance)
{
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Pose2D goal;
  goal.x = distance;
  // A cropped path endpoint near the robot must not enable terminal motion.
  nav_2d_msgs::msg::Path2D local_plan;
  local_plan.poses.push_back(pose);
  critic.prepare(pose, nav_2d_msgs::msg::Twist2D(), goal, local_plan);
}
}  // namespace

TEST(GoalApproachMotion, OutsideAreaAllowsOnlyForwardStopAndRotation)
{
  dog_nav::GoalApproachMotionCritic critic;
  for (const double distance : {0.3001, 1.0, 10.0}) {
    prepareAtDistance(critic, distance);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.25, 0.0)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.0, 0.0)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.0, 0.0, -0.25)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.25, 0.0, 0.25)), 0.0);
    EXPECT_THROW(critic.scoreTrajectory(motion(-0.25, 0.0)), dwb_core::IllegalTrajectoryException);
    EXPECT_THROW(critic.scoreTrajectory(motion(0.0, 0.25)), dwb_core::IllegalTrajectoryException);
    EXPECT_THROW(critic.scoreTrajectory(motion(0.0, -0.25)), dwb_core::IllegalTrajectoryException);
    EXPECT_THROW(critic.scoreTrajectory(motion(0.25, 0.10)), dwb_core::IllegalTrajectoryException);
  }
}

TEST(GoalApproachMotion, WithinAreaAndAtBoundaryAllowsReverseAndLateral)
{
  dog_nav::GoalApproachMotionCritic critic;
  for (const double distance : {0.0, 0.20, 0.30}) {
    prepareAtDistance(critic, distance);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(-0.25, 0.0)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.0, 0.25)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.0, -0.25)), 0.0);
    EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(-0.15, 0.15)), 0.0);
  }
  // Leaving the area immediately restores the restriction; no latched state.
  prepareAtDistance(critic, 0.31);
  EXPECT_THROW(critic.scoreTrajectory(motion(-0.25, 0.0)), dwb_core::IllegalTrajectoryException);
}

TEST(GoalApproachMotion, UsesPlanarDistanceAndRejectsUnknownDistance)
{
  dog_nav::GoalApproachMotionCritic critic;
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Pose2D goal;
  pose.x = 2.0;
  pose.y = 3.0;
  goal.x = 2.22;
  goal.y = 3.22;
  critic.prepare(pose, nav_2d_msgs::msg::Twist2D(), goal, nav_2d_msgs::msg::Path2D());
  EXPECT_THROW(critic.scoreTrajectory(motion(0.0, 0.25)), dwb_core::IllegalTrajectoryException);
  goal.x = 2.20;
  goal.y = 3.20;
  critic.prepare(pose, nav_2d_msgs::msg::Twist2D(), goal, nav_2d_msgs::msg::Path2D());
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(motion(0.0, 0.25)), 0.0);
  for (const double distance : {
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    prepareAtDistance(critic, distance);
    EXPECT_THROW(critic.scoreTrajectory(motion(-0.25, 0.0)), dwb_core::IllegalTrajectoryException);
  }
}

TEST(GoalApproachMotion, PluginLoadsAndReadsConfiguredDistance)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("StationFollowPath.GoalApproachMotion.enable_distance", 0.10)});
  auto node = std::make_shared<nav2_util::LifecycleNode>("approach_critic_test", "", options);
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("dog_nav::GoalApproachMotionCritic");
  critic->initialize(node, "GoalApproachMotion", "StationFollowPath", nullptr);
  geometry_msgs::msg::Pose2D pose;
  geometry_msgs::msg::Pose2D goal;
  goal.x = 0.20;
  critic->prepare(pose, nav_2d_msgs::msg::Twist2D(), goal, nav_2d_msgs::msg::Path2D());
  EXPECT_THROW(critic->scoreTrajectory(motion(-0.25, 0.0)), dwb_core::IllegalTrajectoryException);
  goal.x = 0.10;
  critic->prepare(pose, nav_2d_msgs::msg::Twist2D(), goal, nav_2d_msgs::msg::Path2D());
  EXPECT_DOUBLE_EQ(critic->scoreTrajectory(motion(-0.25, 0.0)), 0.0);
  critic.reset();
  node.reset();
  rclcpp::shutdown();
}
