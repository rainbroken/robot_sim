#include "dog_nav/lateral_velocity_critic.hpp"

#include "dwb_core/trajectory_critic.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

TEST(LateralVelocityPenalty, AppliesOnlyOutsideDeadband)
{
  EXPECT_DOUBLE_EQ(dog_nav::lateralVelocityPenalty(0.00, 0.03), 0.0);
  EXPECT_DOUBLE_EQ(dog_nav::lateralVelocityPenalty(0.03, 0.03), 0.0);
  EXPECT_NEAR(dog_nav::lateralVelocityPenalty(0.13, 0.03), 0.10, 1e-12);
  EXPECT_NEAR(dog_nav::lateralVelocityPenalty(-0.13, 0.03), 0.10, 1e-12);
}

TEST(LateralVelocityPenalty, IsDisabledNearGoal)
{
  EXPECT_TRUE(dog_nav::shouldPenalizeLateralVelocity(0.51, 0.50));
  EXPECT_FALSE(dog_nav::shouldPenalizeLateralVelocity(0.50, 0.50));
  EXPECT_FALSE(dog_nav::shouldPenalizeLateralVelocity(0.20, 0.50));
}

TEST(LateralVelocityCritic, PluginLoads)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("dog_nav::LateralVelocityCritic");
  ASSERT_NE(critic, nullptr);
}
