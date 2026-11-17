#include "dog_nav/stable_rotate_to_goal_critic.hpp"

#include <cmath>

#include "dwb_core/trajectory_critic.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

TEST(StableFinalPoseMode, PersistsUntilPositionExitsTolerance)
{
  using dog_nav::FinalPoseMode;
  auto mode = dog_nav::updateFinalPoseMode(
    FinalPoseMode::APPROACH, false, 0.12, 1.0,
    0.13, 0.15, 0.20, 0.30, 0.10, 0.08);
  EXPECT_EQ(mode, FinalPoseMode::SLOWING);
  mode = dog_nav::updateFinalPoseMode(
    mode, false, 0.12, 1.0,
    0.13, 0.15, 0.20, 0.30, 0.07, 0.08);
  EXPECT_EQ(mode, FinalPoseMode::ROTATING);
  mode = dog_nav::updateFinalPoseMode(
    mode, false, 0.17, 1.0,
    0.13, 0.15, 0.20, 0.30, 0.20, 0.08);
  EXPECT_EQ(mode, FinalPoseMode::ROTATING);
  mode = dog_nav::updateFinalPoseMode(
    mode, false, 0.21, 1.0,
    0.13, 0.15, 0.20, 0.30, 0.00, 0.08);
  EXPECT_EQ(mode, FinalPoseMode::APPROACH);
}

TEST(StableFinalPoseMode, GoalChangeClearsOldRotationState)
{
  using dog_nav::FinalPoseMode;
  const auto mode = dog_nav::updateFinalPoseMode(
    FinalPoseMode::ROTATING, true, 1.0, 1.0,
    0.13, 0.15, 0.20, 0.30, 0.0, 0.08);
  EXPECT_EQ(mode, FinalPoseMode::APPROACH);
}

TEST(StableFinalPoseVelocity, RotatingAllowsOnlyBoundedCorrectRotation)
{
  using dog_nav::FinalPoseMode;
  EXPECT_TRUE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::ROTATING, 0.0, 0.0, 0.0, -0.25,
      -1.0, -1.2, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::ROTATING, 0.0, 0.0, 0.04, -0.25,
      -1.0, -1.2, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::ROTATING, 0.0, 0.0, 0.0, 0.75,
      1.0, 1.2, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::ROTATING, 0.0, 0.0, 0.0, 0.25,
      -1.0, -1.2, 0.15, 0.21, 0.50));
  EXPECT_TRUE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::ROTATING, 0.0, 0.0, 0.0, 0.0,
      -1.0, -1.2, 0.15, 0.21, 0.50));
}

TEST(StableFinalPoseMode, RepositionsAfterYawIsNearlyAligned)
{
  using dog_nav::FinalPoseMode;
  EXPECT_EQ(
    dog_nav::updateFinalPoseMode(
      FinalPoseMode::ROTATING, false, 0.17, 0.20,
      0.13, 0.15, 0.20, 0.30, 0.0, 0.08),
    FinalPoseMode::APPROACH);
  EXPECT_EQ(
    dog_nav::updateFinalPoseMode(
      FinalPoseMode::ROTATING, false, 0.17, 1.00,
      0.13, 0.15, 0.20, 0.30, 0.0, 0.08),
    FinalPoseMode::ROTATING);
}

TEST(StableFinalPoseVelocity, SlowingRequiresStrictlyLowerLinearSpeed)
{
  using dog_nav::FinalPoseMode;
  EXPECT_TRUE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::SLOWING, 0.10, 0.06, 0.00, 0.0,
      1.0, 1.0, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::SLOWING, 0.10, 0.10, 0.00, 0.0,
      1.0, 1.0, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::SLOWING, 0.10, 0.06, 0.00, -0.25,
      1.0, 1.0, 0.15, 0.21, 0.50));
  EXPECT_FALSE(
    dog_nav::isStableGoalVelocityAllowed(
      FinalPoseMode::SLOWING, 0.10, 0.06, 0.00, 0.75,
      1.0, 1.0, 0.15, 0.21, 0.50));
}

TEST(StableFinalPoseRotationDirection, DoesNotFlipAtAngleWrap)
{
  constexpr double pi = 3.14159265358979323846;
  EXPECT_DOUBLE_EQ(dog_nav::selectRotationDirection(0.0, pi - 0.01), 1.0);
  EXPECT_DOUBLE_EQ(dog_nav::selectRotationDirection(1.0, -pi + 0.01), 1.0);
  EXPECT_DOUBLE_EQ(dog_nav::selectRotationDirection(1.0, -0.20), -1.0);
}

TEST(StableRotateToGoalCritic, PluginLoads)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("dog_nav::StableRotateToGoalCritic");
  ASSERT_NE(critic, nullptr);
}
