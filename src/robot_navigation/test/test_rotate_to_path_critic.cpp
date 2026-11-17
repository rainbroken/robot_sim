#include "dog_nav/rotate_to_path_critic.hpp"

#include "dwb_core/trajectory_critic.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

TEST(RotateToPathState, OnlyActivatesForLargeHeadingErrorsAwayFromGoal)
{
  EXPECT_FALSE(dog_nav::updateRotateToPathState(false, 1.50, 2.0, 2.0, 0.45, 0.50));
  EXPECT_TRUE(dog_nav::updateRotateToPathState(false, 2.10, 2.0, 2.0, 0.45, 0.50));
  EXPECT_TRUE(dog_nav::updateRotateToPathState(true, 1.00, 2.0, 2.0, 0.45, 0.50));
  EXPECT_FALSE(dog_nav::updateRotateToPathState(true, 0.40, 2.0, 2.0, 0.45, 0.50));
  EXPECT_FALSE(dog_nav::updateRotateToPathState(true, 2.50, 0.40, 2.0, 0.45, 0.50));
}

TEST(RotateToPathVelocity, ActiveModeOnlyAllowsCorrectInPlaceRotation)
{
  constexpr double error = 2.5;
  EXPECT_TRUE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, error, 0.0, 0.0, 0.30, 0.03, 0.20));
  EXPECT_FALSE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, error, 0.0, 0.0, -0.30, 0.03, 0.20));
  EXPECT_FALSE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, error, 0.0, 0.0, 0.0, 0.03, 0.20));
  EXPECT_TRUE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, error, 0.0, 0.0, 0.0, 0.03, 0.20, 0.20));
  EXPECT_FALSE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, error, 0.05, 0.0, 0.30, 0.03, 0.20));
  EXPECT_TRUE(
    dog_nav::isRotateToPathVelocityAllowed(
      false, error, 0.30, 0.20, -0.10, 0.03, 0.20));
}

TEST(RotateToPathVelocity, ZeroTranslationLimitRejectsCombinedMotion)
{
  for (const double translation : {-0.21, 0.21}) {
    EXPECT_FALSE(
      dog_nav::isRotateToPathVelocityAllowed(
        true, 1.0, translation, 0.0, 0.3, 0.0, 0.21));
    EXPECT_FALSE(
      dog_nav::isRotateToPathVelocityAllowed(
        true, 1.0, 0.0, translation, 0.3, 0.0, 0.21));
  }
  EXPECT_TRUE(
    dog_nav::isRotateToPathVelocityAllowed(
      true, 1.0, 0.0, 0.0, 0.3, 0.0, 0.21));
}

TEST(RotateToPathCritic, PluginLoads)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("dog_nav::RotateToPathCritic");
  ASSERT_NE(critic, nullptr);
}
