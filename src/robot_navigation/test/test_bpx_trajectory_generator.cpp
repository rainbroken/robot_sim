#include <cmath>
#include <memory>

#include "dog_nav/bpx_trajectory_generator.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(BpxVelocityConstraint, AllowsStopAndExecutableCommands)
{
  EXPECT_TRUE(dog_nav::isBpxExecutableVelocity(0.0, 0.0, 0.0, 0.21));
  EXPECT_FALSE(dog_nav::isBpxExecutableVelocity(0.10, 0.10, 0.10, 0.21));
  EXPECT_FALSE(dog_nav::isBpxExecutableVelocity(0.20, 0.0, 0.0, 0.21));
  EXPECT_TRUE(dog_nav::isBpxExecutableVelocity(0.21, 0.0, 0.0, 0.21));
  EXPECT_TRUE(dog_nav::isBpxExecutableVelocity(0.15, 0.15, 0.0, 0.21));
  EXPECT_TRUE(dog_nav::isBpxExecutableVelocity(0.05, 0.0, 0.30, 0.21));
}

TEST(BpxVelocityConstraint, PluginLoads)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  auto generator = loader.createSharedInstance("dog_nav::BpxTrajectoryGenerator");
  ASSERT_NE(generator, nullptr);
}

class BpxStartupTest : public testing::TestWithParam<bool> {};

TEST_P(BpxStartupTest, GeneratedTwistsExcludeSdkDeadZone)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("FollowPath.bpx_startup_threshold", 0.21),
    rclcpp::Parameter("FollowPath.min_vel_x", GetParam() ? -0.25 : 0.0),
    rclcpp::Parameter("FollowPath.max_vel_x", 0.30),
    rclcpp::Parameter("FollowPath.min_vel_y", GetParam() ? 0.0 : -0.25),
    rclcpp::Parameter("FollowPath.max_vel_y", GetParam() ? 0.0 : 0.25),
    rclcpp::Parameter("FollowPath.max_vel_theta", 0.50),
    rclcpp::Parameter("FollowPath.min_speed_xy", 0.0),
    rclcpp::Parameter("FollowPath.max_speed_xy", 0.30),
    rclcpp::Parameter("FollowPath.min_speed_theta", 0.0),
    rclcpp::Parameter("FollowPath.acc_lim_x", 0.25),
    rclcpp::Parameter("FollowPath.acc_lim_y", 0.25),
    rclcpp::Parameter("FollowPath.acc_lim_theta", 0.50),
    rclcpp::Parameter("FollowPath.decel_lim_x", -0.35),
    rclcpp::Parameter("FollowPath.decel_lim_y", -0.35),
    rclcpp::Parameter("FollowPath.decel_lim_theta", -0.70),
    rclcpp::Parameter("FollowPath.vx_samples", 8),
    rclcpp::Parameter("FollowPath.vy_samples", GetParam() ? 1 : 10),
    rclcpp::Parameter("FollowPath.vtheta_samples", 16),
    rclcpp::Parameter("FollowPath.sim_time", 2.2),
    rclcpp::Parameter("FollowPath.linear_granularity", 0.05),
    rclcpp::Parameter("FollowPath.angular_granularity", 0.025)
  });
  auto node = std::make_shared<nav2_util::LifecycleNode>(
    "bpx_generator_test", "", options);
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  auto generator = loader.createSharedInstance("dog_nav::BpxTrajectoryGenerator");
  generator->initialize(node, "FollowPath");

  nav_2d_msgs::msg::Twist2D stopped;
  const auto twists = generator->getTwists(stopped);
  bool found_motion = false;
  bool found_stop = false;
  bool found_forward_startup = false;
  bool found_reverse_startup = false;
  bool found_left_startup = false;
  bool found_right_startup = false;
  bool found_positive_rotation_startup = false;
  bool found_negative_rotation_startup = false;
  for (const auto & twist : twists) {
    const double magnitude = std::sqrt(
      twist.x * twist.x + twist.y * twist.y + twist.theta * twist.theta);
    EXPECT_TRUE(magnitude <= 1.0e-4 || magnitude >= 0.21);
    found_stop = found_stop || magnitude <= 1.0e-4;
    found_motion = found_motion || magnitude >= 0.21;
    found_forward_startup = found_forward_startup ||
      (std::abs(twist.x - 0.21) < 1.0e-9 &&
      std::abs(twist.y) < 1.0e-9 && std::abs(twist.theta) < 1.0e-9);
    EXPECT_GE(twist.x, (GetParam() ? -0.25 : 0.0) - 1.0e-9);
    EXPECT_LE(std::abs(twist.y), (GetParam() ? 0.0 : 0.25) + 1.0e-9);
    found_reverse_startup = found_reverse_startup ||
      (std::abs(twist.x + 0.21) < 1.0e-9 &&
      std::abs(twist.y) < 1.0e-9 && std::abs(twist.theta) < 1.0e-9);
    found_left_startup = found_left_startup ||
      (std::abs(twist.x) < 1.0e-9 && std::abs(twist.y - 0.21) < 1.0e-9 &&
      std::abs(twist.theta) < 1.0e-9);
    found_right_startup = found_right_startup ||
      (std::abs(twist.x) < 1.0e-9 && std::abs(twist.y + 0.21) < 1.0e-9 &&
      std::abs(twist.theta) < 1.0e-9);
    found_positive_rotation_startup = found_positive_rotation_startup ||
      (std::abs(twist.x) < 1.0e-9 && std::abs(twist.y) < 1.0e-9 &&
      std::abs(twist.theta - 0.21) < 1.0e-9);
    found_negative_rotation_startup = found_negative_rotation_startup ||
      (std::abs(twist.x) < 1.0e-9 && std::abs(twist.y) < 1.0e-9 &&
      std::abs(twist.theta + 0.21) < 1.0e-9);
  }
  EXPECT_TRUE(found_motion);
  EXPECT_TRUE(found_stop);
  EXPECT_TRUE(found_forward_startup);
  EXPECT_EQ(found_reverse_startup, GetParam());
  EXPECT_EQ(found_left_startup, !GetParam());
  EXPECT_EQ(found_right_startup, !GetParam());
  EXPECT_TRUE(found_positive_rotation_startup);
  EXPECT_TRUE(found_negative_rotation_startup);

  generator.reset();
  node.reset();
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(MotionDirections, BpxStartupTest, testing::Bool());

TEST(DifferentialSampling, DisablesFirmwareDeadZoneAndRetainsStop)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2_util::LifecycleNode>("differential_sampling_test", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("FollowPath.bpx_enforce_startup_threshold", false),
      rclcpp::Parameter("FollowPath.min_vel_x", 0.0),
      rclcpp::Parameter("FollowPath.max_vel_x", 0.30),
      rclcpp::Parameter("FollowPath.min_vel_y", 0.0),
      rclcpp::Parameter("FollowPath.max_vel_y", 0.0),
      rclcpp::Parameter("FollowPath.max_vel_theta", 0.60),
      rclcpp::Parameter("FollowPath.acc_lim_x", 0.4),
      rclcpp::Parameter("FollowPath.decel_lim_x", -0.5),
      rclcpp::Parameter("FollowPath.acc_lim_theta", 0.8),
      rclcpp::Parameter("FollowPath.decel_lim_theta", -1.0),
      rclcpp::Parameter("FollowPath.min_speed_xy", 0.0),
      rclcpp::Parameter("FollowPath.min_speed_theta", 0.05),
      rclcpp::Parameter("FollowPath.max_speed_xy", 0.30)}));
  dog_nav::BpxTrajectoryGenerator generator;
  generator.initialize(node, "FollowPath");
  bool found_slow_forward = false, found_slow_rotation = false, found_stop = false;
  for (const auto & twist : generator.getTwists(nav_2d_msgs::msg::Twist2D())) {
    EXPECT_GE(twist.x, 0.0);
    EXPECT_LE(twist.x, .30);
    EXPECT_DOUBLE_EQ(twist.y, 0.0);
    found_stop |= twist.x == 0 && twist.theta == 0;
    found_slow_forward |= twist.x > 0 && twist.x < .20 && twist.theta == 0;
    found_slow_rotation |= twist.x == 0 && std::abs(twist.theta) > 0 && std::abs(twist.theta) < .20;
  }
  EXPECT_TRUE(found_stop);
  EXPECT_TRUE(found_slow_forward);
  EXPECT_TRUE(found_slow_rotation);
  node.reset();
  rclcpp::shutdown();
}
