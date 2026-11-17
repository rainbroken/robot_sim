#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "dwb_core/dwb_local_planner.hpp"
#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace
{
// Exercise the installed DWB implementation without activating a costmap,
// controller, action server or velocity publisher.
class EmptyCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  EmptyCostmap() : Costmap2DROS("pruning_test_costmap")
  {
    global_frame_ = "map";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      "map", false, false);
    layered_costmap_->resizeMap(120, 120, 0.05, -3.0, -3.0);
  }
};

class PruningPlanner : public dwb_core::DWBLocalPlanner
{
public:
  explicit PruningPlanner(double prune_distance)
  {
    node_owner_ = std::make_shared<nav2_util::LifecycleNode>("pruning_test_controller");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_owner_->get_clock());
    costmap_ros_ = std::make_shared<EmptyCostmap>();
    pub_ = std::make_unique<dwb_core::DWBPublisher>(node_owner_, "test");
    pub_->on_configure();
    prune_plan_ = true;
    prune_distance_ = prune_distance;
    forward_prune_distance_ = 2.0;
    shorten_transformed_plan_ = true;
    global_plan_.header.frame_id = "map";
    // Two parallel flights 1.5 m apart, joined by a landing.
    for (int i = 0; i <= 50; ++i) {append(0.0, i * 0.1);}
    for (int i = 1; i <= 15; ++i) {append(-i * 0.1, 5.0);}
    for (int i = 1; i <= 50; ++i) {append(-1.5, 5.0 - i * 0.1);}
  }

  nav_2d_msgs::msg::Path2D crop(double x, double y)
  {
    nav_2d_msgs::msg::Pose2DStamped pose;
    pose.header.frame_id = "map";
    pose.pose.x = x;
    pose.pose.y = y;
    return transformGlobalPlan(pose);
  }

private:
  void append(double x, double y)
  {
    geometry_msgs::msg::Pose2D pose;
    pose.x = x;
    pose.y = y;
    global_plan_.poses.push_back(pose);
  }
  nav2_util::LifecycleNode::SharedPtr node_owner_;
};

double distanceToPlan(const nav_2d_msgs::msg::Path2D & plan, double x, double y)
{
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & p : plan.poses) {
    nearest = std::min(nearest, std::hypot(p.x - x, p.y - y));
  }
  return nearest;
}

class SwitchbackPruning : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(SwitchbackPruning, ShortPassedPortionKeepsCurrentReturnFlight)
{
  PruningPlanner planner(0.5);
  for (int i = 0; i <= 50; ++i) {planner.crop(0.0, i * 0.1);}
  for (int i = 1; i <= 15; ++i) {planner.crop(-i * 0.1, 5.0);}
  for (int i = 1; i <= 40; ++i) {
    const double y = 5.0 - i * 0.1;
    const auto plan = planner.crop(-1.5, y);
    ASSERT_LT(distanceToPlan(plan, -1.5, y), 0.15) << "return y=" << y;
    EXPECT_NEAR(plan.poses.back().x, -1.5, 1e-6);
    EXPECT_LT(plan.poses.back().y, y);
  }
}

TEST_F(SwitchbackPruning, OldTwoMetreRetentionCanLoseReturnFlight)
{
  PruningPlanner planner(2.0);
  for (int i = 0; i <= 50; ++i) {planner.crop(0.0, i * 0.1);}
  for (int i = 1; i <= 15; ++i) {planner.crop(-i * 0.1, 5.0);}
  bool lost_current_flight = false;
  for (int i = 1; i <= 40; ++i) {
    const double y = 5.0 - i * 0.1;
    try {
      lost_current_flight |= distanceToPlan(planner.crop(-1.5, y), -1.5, y) > 1.0;
    } catch (const nav2_core::PlannerException & error) {
      EXPECT_STREQ(error.what(), "Resulting plan has 0 poses in it.");
      lost_current_flight = true;
      break;
    }
  }
  EXPECT_TRUE(lost_current_flight);
}
}  // namespace
