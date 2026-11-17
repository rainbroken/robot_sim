// Deterministic closed-loop controller tests, no /cmd_vel publisher or hardware.
// Uses the installed DWB and the real YAML, plugins, footprint and inflation law.
// Ideal odometry and supplied paths intentionally isolate controller behavior
// from perception, global planning, localization and the behavior tree.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include "dog_nav/parallel_dwb_local_planner.hpp"
#include "dog_nav/cached_obstacle_footprint_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
using Pose = geometry_msgs::msg::Pose2D;
constexpr double dt = 1.0 / 15.0;
double angle(double x) {return std::atan2(std::sin(x), std::cos(x));}
const char * configPath()
{
  const auto path = std::getenv("NAV_SCENARIO_CONFIG");
  return path ? path : NAV_CONFIG;
}

class SceneCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  SceneCostmap(bool person, double wall, bool blocked)
  : Costmap2DROS("scenario_costmap")
  {
    const auto config = YAML::LoadFile(configPath())["local_costmap"]["local_costmap"]["ros__parameters"];
    const double resolution = config["resolution"].as<double>();
    footprint_padding_ = config["footprint_padding"].as<double>();
    global_frame_ = "map";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(
      static_cast<unsigned>(std::lround(config["width"].as<double>() / resolution)),
      static_cast<unsigned>(std::lround(config["height"].as<double>() / resolution)),
      resolution, -3.0, -3.0);
    std::vector<geometry_msgs::msg::Point> footprint;
    for (const auto xy : YAML::Load(config["footprint"].as<std::string>())) {
      geometry_msgs::msg::Point p;
      p.x = xy[0].as<double>(); p.y = xy[1].as<double>();
      footprint.push_back(p);
    }
    setRobotFootprint(footprint);
    const double radius = config["inflation_layer"]["inflation_radius"].as<double>();
    const double scaling = config["inflation_layer"]["cost_scaling_factor"].as<double>();
    const double inscribed = layered_costmap_->getInscribedRadius();
    auto * map = getCostmap();
    // Rasterize lethal cells first, then inflate distances to lethal cell centers.
    const unsigned width = map->getSizeInCellsX(), height = map->getSizeInCellsY();
    std::vector<bool> lethal(width * height, false);
    for (unsigned y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned x = 0; x < map->getSizeInCellsX(); ++x) {
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        if (std::abs(wy) >= wall ||
          (person && std::hypot(wx, wy) <= 0.20) ||
          (blocked && std::abs(wx) < 0.10))
        {
          lethal[y * width + x] = true;
        }
      }
    }
    struct Offset {int x, y; unsigned char cost;};
    std::vector<Offset> kernel;
    const int cells = static_cast<int>(std::ceil(radius / resolution));
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        const double distance = std::hypot(dx, dy) * resolution;
        if (distance > radius) {continue;}
        const auto cost = distance <= inscribed ? 253 :
          static_cast<unsigned char>(252 * std::exp(-scaling * (distance - inscribed)));
        kernel.push_back({dx, dy, static_cast<unsigned char>(cost)});
      }
    }
    std::sort(kernel.begin(), kernel.end(), [](const Offset & a, const Offset & b) {
      return a.cost > b.cost;
    });
    for (unsigned y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned x = 0; x < map->getSizeInCellsX(); ++x) {
        unsigned char cost = lethal[y * width + x] ? 254 : 0;
        if (!cost) {
          for (const auto & offset : kernel) {
            const int nx = static_cast<int>(x) + offset.x, ny = static_cast<int>(y) + offset.y;
            if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width) && ny < static_cast<int>(height) &&
              lethal[ny * width + nx])
            {
              cost = offset.cost; break;
            }
          }
        }
        map->setCost(x, y, cost);
      }
    }
  }
};

nav_msgs::msg::Path makePath(const std::vector<Pose> & corners, double yaw)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const auto & a = corners[i - 1]; const auto & b = corners[i];
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / .025)));
    for (int j = (i == 1 ? 0 : 1); j <= n; ++j) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = a.x + (b.x - a.x) * j / n;
      p.pose.position.y = a.y + (b.y - a.y) * j / n;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
      path.poses.push_back(p);
    }
  }
  return path;
}
Pose point(double x, double y, double theta = 0)
{Pose p; p.x = x; p.y = y; p.theta = theta; return p;}

struct Outcome
{
  bool reached{false};
  int failures{0};
  int longest_failure_streak{0};
  int reversals{0};
  double seconds{0};
  double min_clearance{100};
  Pose pose;
};

Outcome run(const std::vector<Pose> & corners, Pose pose, double goal_yaw,
  bool person = false, double wall = 2.9, bool blocked = false)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("controller_server", "",
    rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", configPath()})
    .parameter_overrides({rclcpp::Parameter("use_sim_time", false)}));
  auto costmap = std::make_shared<SceneCostmap>(person, wall, blocked);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  dog_nav::ParallelDwbLocalPlanner planner;
  planner.configure(node, "FollowPath", tf, costmap);
  const auto path = makePath(corners, goal_yaw);
  planner.setPlan(path);
  Outcome out;
  nav_2d_msgs::msg::Twist2D velocity;
  double previous_turn = 0;
  double next_replan = 0;
  int failure_streak = 0;
  std::map<std::string, int> reasons;
  std::ofstream trace;
  if (const auto directory = std::getenv("NAV_SCENARIO_TRACE_DIR")) {
    trace.open(std::string(directory) + "/" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".csv");
    trace << "time,x,y,yaw,vx,wz,command_x,command_wz\n";
  }
  const auto params = YAML::LoadFile(configPath())["controller_server"]["ros__parameters"];
  const bool stateful = params["general_goal_checker"]["stateful"].as<bool>();
  const double xy_tolerance = params["general_goal_checker"]["xy_goal_tolerance"].as<double>();
  const double yaw_tolerance = params["general_goal_checker"]["yaw_goal_tolerance"].as<double>();
  const auto replan_env = std::getenv("NAV_SCENARIO_REPLAN_HZ");
  const double replan_period = 1.0 / (replan_env ? std::stod(replan_env) : 2.0);
  double half_x = 0, half_y = 0;
  for (const auto & p : costmap->getRobotFootprint()) {
    half_x = std::max(half_x, std::abs(p.x));
    half_y = std::max(half_y, std::abs(p.y));
  }
  bool xy_reached = false;
  // Each run is at most 65 seconds of ideal simulated time.
  for (int tick = 0; tick < 975; ++tick) {
    const double distance = std::hypot(pose.x - corners.back().x, pose.y - corners.back().y);
    xy_reached = distance <= xy_tolerance || (stateful && xy_reached);
    if (xy_reached && std::abs(angle(goal_yaw - pose.theta)) <= yaw_tolerance) {
      out.reached = true; break;
    }
    // A new global path starts at the CURRENT robot, not the original start.
    // Replaying the original start would violate DWB's bounded prune search.
    if (out.seconds + 1e-6 >= next_replan) {
      next_replan += replan_period;
      auto replanned = path;
      const auto nearest = std::min_element(path.poses.begin(), path.poses.end(),
        [&pose](const auto & a, const auto & b) {
          return std::hypot(a.pose.position.x - pose.x, a.pose.position.y - pose.y) <
                 std::hypot(b.pose.position.x - pose.x, b.pose.position.y - pose.y);
        });
      replanned.poses.erase(replanned.poses.begin(),
        replanned.poses.begin() + std::distance(path.poses.begin(), nearest));
      planner.setPlan(replanned);
    }
    nav_2d_msgs::msg::Pose2DStamped stamped;
    stamped.header.frame_id = "map";
    stamped.pose = pose;
    nav_2d_msgs::msg::Twist2D command;
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> evaluation;
    try {
      command = planner.computeVelocityCommands(stamped, velocity, evaluation).velocity;
      failure_streak = 0;
    } catch (const nav2_core::PlannerException & e) {
      ++out.failures; ++failure_streak; ++reasons[e.what()];
      out.longest_failure_streak = std::max(out.longest_failure_streak, failure_streak);
    }
    if (trace) {
      trace << out.seconds << ',' << pose.x << ',' << pose.y << ',' << pose.theta << ',' <<
        velocity.x << ',' << velocity.theta << ',' << command.x << ',' << command.theta << '\n';
    }
    // Deterministic acceleration-limited differential plant, no noise or slip.
    const auto approach = [](double current, double target, double acc, double dec) {
        return current + std::clamp(target - current, -dec * dt, acc * dt);
      };
    velocity.x = approach(velocity.x, command.x, .4, .5);
    velocity.theta = approach(velocity.theta, command.theta, .8, 1.0);
    pose.x += velocity.x * std::cos(pose.theta + velocity.theta * dt / 2) * dt;
    pose.y += velocity.x * std::sin(pose.theta + velocity.theta * dt / 2) * dt;
    pose.theta = angle(pose.theta + velocity.theta * dt);
    if (distance < .30 && std::abs(command.theta) > .04) {
      if (previous_turn * command.theta < 0) {++out.reversals;}
      previous_turn = command.theta;
    }
    // Independent filled-rectangle check (not just DWB footprint perimeter).
    double clearance = 100;
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    clearance = wall - std::abs(pose.y) - std::abs(s) * half_x - std::abs(c) * half_y;
    if (person) {
      const double px = -pose.x * c - pose.y * s;
      const double py = pose.x * s - pose.y * c;
      clearance = std::min(clearance,
        std::hypot(std::max(0.0, std::abs(px) - half_x),
        std::max(0.0, std::abs(py) - half_y)) - .20);
    }
    if (blocked) {
      clearance = std::min(clearance,
        std::abs(pose.x) - .10 - std::abs(c) * half_x - std::abs(s) * half_y);
    }
    out.min_clearance = std::min(out.min_clearance, clearance);
    if (clearance < -.001) {ADD_FAILURE() << "Filled footprint collision"; break;}
    out.seconds += dt;
  }
  out.pose = pose;
  std::cout << "SCENARIO reached=" << out.reached << " seconds=" << out.seconds <<
    " failures=" << out.failures << " longest_failure_streak=" << out.longest_failure_streak <<
    " reversals=" << out.reversals <<
    " clearance=" << out.min_clearance << " final=" << pose.x << "," << pose.y <<
    "," << pose.theta << std::endl;
  for (const auto & reason : reasons) {
    std::cout << "FAILURE_REASON " << reason.second << " " << reason.first << std::endl;
  }
  planner.cleanup();
  return out;
}

class NavigationScenarios : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(NavigationScenarios, PassStandingPersonWithSeventyFiveCentimetreSideGap)
{
  const auto out = run({point(-1.8, 0), point(-.85, -.62), point(.80, -.62),
    point(1.8, 0)}, point(-1.8, 0), 0, true, .95);
  EXPECT_TRUE(out.reached);
  EXPECT_LT(out.failures, 8);
  EXPECT_GT(out.min_clearance, .02);
}

TEST_F(NavigationScenarios, TightStraightPassage)
{
  const auto out = run({point(-1.5, 0), point(1.5, 0)}, point(-1.5, 0), 0, false, .35);
  EXPECT_TRUE(out.reached);
  EXPECT_LT(out.failures, 8);
  EXPECT_GT(out.min_clearance, .02);
}

TEST_F(NavigationScenarios, FinalHeadingConvergesWithoutRepeatedReversals)
{
  const auto out = run({point(-.4, 0), point(0, 0)}, point(-.4, 0), 1.57);
  EXPECT_TRUE(out.reached);
  EXPECT_LT(out.seconds, 15);
  EXPECT_LE(out.reversals, 1);
  EXPECT_LT(out.failures, 8);
}

TEST_F(NavigationScenarios, BlockedCorridorNeverBecomesPassableByTuning)
{
  const auto out = run({point(-1.0, 0), point(1.0, 0)}, point(-1.0, 0), 0, false, .35, true);
  EXPECT_FALSE(out.reached);
  EXPECT_LT(out.pose.x, -.36);
  EXPECT_GE(out.min_clearance, 0);
}

TEST_F(NavigationScenarios, MirroredPersonDetourAndOppositeFinalHeading)
{
  const auto out = run({point(-1.8, 0), point(-.85, .62), point(.80, .62),
    point(1.8, 0)}, point(-1.8, 0), -1.57, true, .95);
  EXPECT_TRUE(out.reached);
  EXPECT_EQ(out.failures, 0);
  EXPECT_GT(out.min_clearance, .02);
  EXPECT_LE(out.reversals, 2);
}

TEST_F(NavigationScenarios, SmallFinalYawErrorDoesNotStall)
{
  const auto out = run({point(-.4, 0), point(0, 0)}, point(-.12, 0), .16);
  EXPECT_TRUE(out.reached);
  EXPECT_LT(out.seconds, 2.0);
  EXPECT_EQ(out.failures, 0);
  EXPECT_LE(out.reversals, 1);
}

TEST_F(NavigationScenarios, NearPiFinalHeadingConverges)
{
  const auto out = run({point(-.4, 0), point(0, 0)}, point(-.12, 0), 3.13);
  EXPECT_TRUE(out.reached);
  EXPECT_LT(out.seconds, 9.0);
  EXPECT_EQ(out.failures, 0);
  EXPECT_LE(out.reversals, 1);
}

TEST_F(NavigationScenarios, LargeInitialHeadingStillTurnsBeforeDriving)
{
  const auto out = run({point(-1.5, 0), point(1.5, 0)}, point(-1.5, 0, 1.5), 0);
  EXPECT_TRUE(out.reached);
  EXPECT_EQ(out.failures, 0);
}

TEST_F(NavigationScenarios, CenterGradientKeepsCollisionRejectionAndCacheParity)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("obstacle_regression", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("FollowPath.Obstacle.score_center", true)}));
  auto map = std::make_shared<SceneCostmap>(true, .95, false);
  dog_nav::CachedObstacleFootprintCritic critic;
  critic.initialize(node, "Obstacle", "FollowPath", map);
  ASSERT_TRUE(critic.prepare(Pose(), nav_2d_msgs::msg::Twist2D(), Pose(), nav_2d_msgs::msg::Path2D()));
  std::vector<dwb_msgs::msg::Trajectory2D> trajectories(4);
  trajectories[0].poses = {point(-1.2, 0)};
  trajectories[1].poses = {point(-.65, 0)};
  // Collision on the footprint perimeter AND wholly inside the footprint.
  trajectories[2].poses = {point(-.4, 0)};
  trajectories[3].poses = {point(0, 0)};
  const double far = critic.scoreTrajectory(trajectories[0]);
  const double near = critic.scoreTrajectory(trajectories[1]);
  EXPECT_LT(far, near);
  for (const auto i : {2u, 3u}) {
    EXPECT_THROW(critic.scoreTrajectory(trajectories[i]), dwb_core::IllegalTrajectoryException);
  }
  critic.prepareTrajectoryCache(trajectories);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectories[0]), far);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectories[1]), near);
  for (const auto i : {2u, 3u}) {
    EXPECT_THROW(critic.scoreTrajectory(trajectories[i]), dwb_core::IllegalTrajectoryException);
  }
  critic.prepareTrajectoryCache({});
  // An unknown cell enclosed by the footprint must also remain forbidden.
  unsigned mx, my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(-1.2, 0, mx, my));
  map->getCostmap()->setCost(mx, my, 255);
  EXPECT_THROW(critic.scoreTrajectory(trajectories[0]), dwb_core::IllegalTrajectoryException);
}

}  // namespace
