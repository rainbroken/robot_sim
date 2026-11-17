#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include "dog_slam/map_rebuilder.hpp"

namespace
{
using dog_slam::Cloud;
using dog_slam::Keyframe;
using dog_slam::MapRebuilder;

MapRebuilder::Config config()
{
  MapRebuilder::Config c;
  c.occupancy_resolution = 0.05;
  c.fallback_ground_below_sensor = 0.0;
  c.lidar_origin_x = 0.0;
  c.lidar_origin_y = 0.0;
  c.lidar_origin_z = 0.8;
  return c;
}

void point(Cloud & cloud, float x, float y, float z)
{
  pcl::PointXYZI p;
  p.x = x; p.y = y; p.z = z; p.intensity = 1.0f;
  cloud.push_back(p);
}

int cell(const nav_msgs::msg::OccupancyGrid & grid, double x, double y)
{
  const auto ix = static_cast<int>(std::floor(
      (x - grid.info.origin.position.x) / grid.info.resolution + 1.e-6));
  const auto iy = static_cast<int>(std::floor(
      (y - grid.info.origin.position.y) / grid.info.resolution + 1.e-6));
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) ||
    iy >= static_cast<int>(grid.info.height))
  {
    return -1;
  }
  return grid.data[iy * grid.info.width + ix];
}

Keyframe furnitureScene(bool reverse = false)
{
  Keyframe k;
  // A low piece of furniture and returns from a wall above/behind it.
  // These are valid simultaneous 3-D observations, not evidence that the
  // furniture's 2-D footprint is free.
  point(*k.cloud, 1.025f, 0.025f, 0.4f);
  for (int i = 0; i < 60; ++i) {
    point(*k.cloud, 2.025f + 0.05f * i, 0.025f, 1.0f);
  }
  if (reverse) {
    std::reverse(k.cloud->begin(), k.cloud->end());
  }
  return k;
}

TEST(MapOccupancy, FurnitureSurvivesRaysAboveIt)
{
  MapRebuilder mapper(config());
  const auto grid = mapper.occupancy({furnitureScene()}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid);
  EXPECT_EQ(cell(*grid, 1.025, 0.025), 100);
  EXPECT_EQ(cell(*grid, 0.525, 0.025), 0);
}

TEST(MapOccupancy, EndpointOrderDoesNotChangeMap)
{
  MapRebuilder mapper(config());
  const auto a = mapper.occupancy({furnitureScene()}, "map", builtin_interfaces::msg::Time());
  const auto b = mapper.occupancy({furnitureScene(true)}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(a); ASSERT_TRUE(b);
  EXPECT_EQ(a->data, b->data);
}

TEST(MapOccupancy, SavedAndIncrementalMapsAgree)
{
  MapRebuilder mapper(config());
  std::vector<Keyframe> ks{furnitureScene()};
  const auto saved = mapper.occupancy(ks, "map", builtin_interfaces::msg::Time());
  const auto live = mapper.rebuildIncremental(ks, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(saved); ASSERT_TRUE(live.occupancy);
  EXPECT_EQ(saved->info.width, live.occupancy->info.width);
  EXPECT_EQ(saved->info.height, live.occupancy->info.height);
  EXPECT_EQ(saved->data, live.occupancy->data);
  EXPECT_EQ(cell(*live.occupancy, 1.025, 0.025), 100);
}

TEST(MapOccupancy, ClearingStopsAtProjectedObstacle)
{
  MapRebuilder mapper(config());
  Keyframe k;
  point(*k.cloud, 1.025f, .025f, .4f);
  point(*k.cloud, 3.025f, .025f, 1.0f);
  const auto grid = mapper.occupancy({k}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid);
  EXPECT_EQ(cell(*grid, .525, .025), 0);
  EXPECT_EQ(cell(*grid, 1.025, .025), 100);
  EXPECT_EQ(cell(*grid, 2.025, .025), -1);
  EXPECT_EQ(cell(*grid, 3.025, .025), 100);
}

TEST(MapOccupancy, LowObstacleHeightUsesBaseNotLidar)
{
  MapRebuilder mapper(config());
  Keyframe k;
  // The base is on a raised floor. A 0.2 m obstacle is below the 0.8 m LiDAR.
  k.optimized_pose.translation().z() = 3.0;
  point(*k.cloud, 1.025f, .025f, .2f);
  point(*k.cloud, 2.025f, .525f, 0.0f);
  point(*k.cloud, 3.025f, .525f, 2.0f);  // Ceiling excluded.
  const auto grid = mapper.occupancy({k}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid);
  EXPECT_EQ(cell(*grid, 1.025, .025), 100);
  EXPECT_EQ(cell(*grid, 2.025, .525), 0);
  EXPECT_EQ(cell(*grid, 3.025, .525), -1);
}

TEST(MapOccupancy, RaysStartAtPhysicalLidarXY)
{
  auto c = config();
  c.lidar_origin_y = 1.0;
  MapRebuilder mapper(c);
  Keyframe k;
  point(*k.cloud, 2.025f, 1.025f, .4f);
  const auto grid = mapper.occupancy({k}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid);
  EXPECT_EQ(cell(*grid, .525, 1.025), 0);
  EXPECT_EQ(cell(*grid, .525, .025), -1);
}

TEST(MapOccupancy, UsesMappingCloudInsteadOfCoarseDescriptor)
{
  MapRebuilder mapper(config());
  Keyframe k;
  point(*k.cloud, 2.025f, .025f, 1.0f);
  k.map_cloud = std::make_shared<Cloud>(*k.cloud);
  point(*k.map_cloud, 1.025f, .025f, .4f);
  const auto grid = mapper.occupancy({k}, "map", builtin_interfaces::msg::Time());
  const auto live = mapper.rebuildIncremental({k}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid); ASSERT_TRUE(live.occupancy);
  EXPECT_EQ(cell(*grid, 1.025, .025), 100);
  EXPECT_EQ(grid->data, live.occupancy->data);
}

TEST(MapOccupancy, EmptyAndOutOfHeightOrRangeStayUnknown)
{
  MapRebuilder mapper(config());
  EXPECT_FALSE(mapper.occupancy({}, "map", builtin_interfaces::msg::Time()));
  Keyframe k;
  point(*k.cloud, 1.0f, .0f, 2.0f);
  point(*k.cloud, 20.0f, .0f, .4f);
  EXPECT_FALSE(mapper.occupancy({k}, "map", builtin_interfaces::msg::Time()));
}

TEST(MapOccupancy, RepeatedFreeObservationsCanClearAnOldObstacle)
{
  MapRebuilder mapper(config());
  std::vector<Keyframe> ks;
  Keyframe hit;
  point(*hit.cloud, 1.025f, .025f, .4f);
  ks.push_back(hit);
  for (int i = 0; i < 4; ++i) {
    Keyframe clear;
    point(*clear.cloud, 2.025f, .025f, .0f);
    ks.push_back(clear);
  }
  const auto grid = mapper.occupancy(ks, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(grid);
  EXPECT_EQ(cell(*grid, 1.025, .025), 0);
}

TEST(MapOccupancy, KeyframeOrderDoesNotChangeEvidence)
{
  MapRebuilder mapper(config());
  std::vector<Keyframe> ks{furnitureScene(), furnitureScene(true)};
  Keyframe floor;
  point(*floor.cloud, 4.025f, .025f, 0.0f);
  ks.push_back(floor);
  const auto a = mapper.occupancy(ks, "map", builtin_interfaces::msg::Time());
  std::reverse(ks.begin(), ks.end());
  const auto b = mapper.occupancy(ks, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(a); ASSERT_TRUE(b);
  EXPECT_EQ(a->data, b->data);
}

TEST(MapOccupancy, LoopCorrectionRemovesOldEvidence)
{
  MapRebuilder mapper(config());
  auto k = furnitureScene();
  mapper.rebuildIncremental({k}, "map", builtin_interfaces::msg::Time());
  k.optimized_pose.translation().y() = 2.0;
  const auto live = mapper.rebuildIncremental({k}, "map", builtin_interfaces::msg::Time());
  const auto saved = mapper.occupancy({k}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(live.occupancy); ASSERT_TRUE(saved);
  EXPECT_TRUE(live.reset_required);
  EXPECT_EQ(live.occupancy->data, saved->data);
  EXPECT_EQ(cell(*live.occupancy, 1.025, .025), -1);
  EXPECT_EQ(cell(*live.occupancy, 1.025, 2.025), 100);
}

TEST(MapOccupancy, LayerResetDoesNotLeakPreviousLayer)
{
  MapRebuilder mapper(config());
  std::vector<Keyframe> ks{furnitureScene()};
  mapper.rebuildIncremental(ks, "map", builtin_interfaces::msg::Time());
  mapper.resetOccupancy(1);
  auto next = furnitureScene();
  next.optimized_pose.translation().y() = 3.0;
  ks.push_back(next);
  const auto live = mapper.rebuildIncremental(ks, "map", builtin_interfaces::msg::Time());
  const auto saved = mapper.occupancy({next}, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(live.occupancy); ASSERT_TRUE(saved);
  EXPECT_EQ(live.occupancy->data, saved->data);
}

TEST(MapOccupancy, IncrementalPatchMatchesSavedGrid)
{
  MapRebuilder mapper(config());
  Keyframe clear;
  point(*clear.cloud, 5.025f, .025f, .0f);
  std::vector<Keyframe> ks{clear};
  const auto first = mapper.rebuildIncremental(ks, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(first.occupancy);
  auto live = *first.occupancy;
  // Several observations overcome the earlier free observation at the hit.
  for (int i = 0; i < 2; ++i) {
    ks.push_back(furnitureScene());
    const auto next = mapper.rebuildIncremental(
      ks, "map", builtin_interfaces::msg::Time(), false, false);
    if (next.occupancy) {
      live = *next.occupancy;
    } else {
      ASSERT_TRUE(next.occupancy_update);
      const auto & u = *next.occupancy_update;
      for (unsigned y = 0; y < u.height; ++y) {
        for (unsigned x = 0; x < u.width; ++x) {
          live.data[(y + u.y) * live.info.width + x + u.x] = u.data[y * u.width + x];
        }
      }
    }
  }
  const auto saved = mapper.occupancy(ks, "map", builtin_interfaces::msg::Time());
  ASSERT_TRUE(saved);
  EXPECT_EQ(live.data, saved->data);
  EXPECT_EQ(cell(live, 1.025, .025), 100);
}
}  // namespace
