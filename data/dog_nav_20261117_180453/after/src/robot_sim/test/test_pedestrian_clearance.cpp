#include "gtest/gtest.h"
#include "pedestrian_clearance.hpp"

TEST(PedestrianClearance, KeepsApproachingActorsOutsideExclusionCircle)
{
  EXPECT_TRUE(robot_sim::robotBlocksPedestrian(0.72, 0.69));
  EXPECT_FALSE(robot_sim::robotBlocksPedestrian(0.85, 0.80));
}

TEST(PedestrianClearance, AllowsAnActorToEscapeAfterRobotApproaches)
{
  EXPECT_FALSE(robot_sim::robotBlocksPedestrian(0.35, 0.37));
  EXPECT_FALSE(robot_sim::robotBlocksPedestrian(0.69, 0.71));
}

TEST(PedestrianClearance, StillBlocksCloserOrStationaryStepsInsideCircle)
{
  EXPECT_TRUE(robot_sim::robotBlocksPedestrian(0.35, 0.33));
  EXPECT_TRUE(robot_sim::robotBlocksPedestrian(0.35, 0.35));
}
