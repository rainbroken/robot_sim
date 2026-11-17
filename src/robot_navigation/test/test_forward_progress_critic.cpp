#include "dog_nav/forward_progress_critic.hpp"

#include "dwb_core/trajectory_critic.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

TEST(ForwardProgressPenalty, PenalizesOnlyForwardSpeedDeficit)
{
  EXPECT_DOUBLE_EQ(dog_nav::forwardProgressPenalty(0.0, 0.21), 0.21);
  EXPECT_NEAR(dog_nav::forwardProgressPenalty(0.12, 0.21), 0.09, 1e-12);
  EXPECT_DOUBLE_EQ(dog_nav::forwardProgressPenalty(0.21, 0.21), 0.0);
  EXPECT_DOUBLE_EQ(dog_nav::forwardProgressPenalty(0.40, 0.21), 0.0);
}

TEST(ForwardProgressCondition, RequiresDistanceAndPathAlignment)
{
  EXPECT_TRUE(dog_nav::shouldEncourageForwardProgress(0.51, 0.69, 0.50, 0.70));
  EXPECT_FALSE(dog_nav::shouldEncourageForwardProgress(0.50, 0.10, 0.50, 0.70));
  EXPECT_FALSE(dog_nav::shouldEncourageForwardProgress(1.00, 0.70, 0.50, 0.70));
  EXPECT_FALSE(dog_nav::shouldEncourageForwardProgress(1.00, -0.71, 0.50, 0.70));
}

TEST(ForwardProgressCritic, PluginLoads)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> loader(
    "dwb_core", "dwb_core::TrajectoryCritic");
  auto critic = loader.createSharedInstance("dog_nav::ForwardProgressCritic");
  ASSERT_NE(critic, nullptr);
}
