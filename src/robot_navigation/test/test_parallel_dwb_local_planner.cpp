#include <atomic>
#include <memory>
#include <vector>

#include <omp.h>
#include <sched.h>

#include "dog_nav/parallel_dwb_local_planner.hpp"
#include "dog_nav/trajectory_cache.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "gtest/gtest.h"

namespace
{
class Generator : public dwb_core::TrajectoryGenerator
{
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr &, const std::string &) override {}
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override {index_ = 0;}
  bool hasMoreTwists() override {return index_ < 4;}
  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    nav_2d_msgs::msg::Twist2D twist;
    twist.x = ++index_;
    return twist;
  }
  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const nav_2d_msgs::msg::Twist2D & command) override
  {
    dwb_msgs::msg::Trajectory2D trajectory;
    trajectory.velocity = command;
    return trajectory;
  }
  void setSpeedLimit(const double &, const bool &) override {}

private:
  int index_{0};
};

class MotionGate : public dwb_core::TrajectoryCritic
{
public:
  explicit MotionGate(bool reject_all) : reject_all_(reject_all) {scale_ = 1.0;}
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (reject_all_ || trajectory.velocity.x == 1.0) {
      throw dwb_core::IllegalTrajectoryException("MotionGate", "forbidden command");
    }
    return trajectory.velocity.x;
  }

private:
  bool reject_all_;
};

// Model expensive collision work after the motion gate. Cache preparation is
// deliberately observable: rejected / already worse commands must not reach it
// on a single CPU. No ROS publishers or robot commands are created by this test.
class CollisionCritic : public dwb_core::TrajectoryCritic, public dog_nav::TrajectoryCacheProvider
{
public:
  CollisionCritic() {scale_ = 1.0;}
  void prepareTrajectoryCache(const std::vector<dwb_msgs::msg::Trajectory2D> & trajectories)
  override {cached += trajectories.size();}
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override
  {
    ++scored;
    return 0.0;
  }
  std::atomic<int> scored{0};
  std::size_t cached{0};
};

class Planner : public dog_nav::ParallelDwbLocalPlanner
{
public:
  explicit Planner(bool reject_all = false)
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    debug_trajectory_details_ = false;
    short_circuit_trajectory_evaluation_ = true;
    traj_generator_ = std::make_shared<Generator>();
    collision = std::make_shared<CollisionCritic>();
    critics_ = {std::make_shared<MotionGate>(reject_all), collision};
  }
  dwb_msgs::msg::TrajectoryScore run()
  {
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
    return coreScoringAlgorithm(
      geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  }
  std::shared_ptr<CollisionCritic> collision;
};

class Scoring : public ::testing::Test
{
protected:
  void SetUp() override {threads_ = omp_get_max_threads();}
  void TearDown() override {omp_set_num_threads(threads_);}
  int threads_;
};

TEST_F(Scoring, SerialSkipsCollisionWorkForRejectedAndWorseCommands)
{
  omp_set_num_threads(1);
  Planner planner;
  EXPECT_DOUBLE_EQ(planner.run().traj.velocity.x, 2.0);
  EXPECT_EQ(planner.collision->cached, 0u);
  EXPECT_EQ(planner.collision->scored, 1);
}

TEST_F(Scoring, RejectedCommandsRemainRejectedAndKeepTheirReason)
{
  omp_set_num_threads(1);
  Planner planner(true);
  try {
    planner.run();
    FAIL() << "All commands should have been rejected";
  } catch (const dwb_core::NoLegalTrajectoriesException & exception) {
    const auto percentages = exception.tracker_.getPercentages();
    ASSERT_EQ(percentages.size(), 1u);
    EXPECT_EQ(percentages.begin()->first.first, "MotionGate");
    EXPECT_EQ(percentages.begin()->first.second, "forbidden command");
    EXPECT_DOUBLE_EQ(percentages.begin()->second, 1.0);
  }
  EXPECT_EQ(planner.collision->scored, 0);
}

TEST_F(Scoring, ParallelAndSerialSelectTheSameCommand)
{
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  ASSERT_EQ(sched_getaffinity(0, sizeof(allowed), &allowed), 0);
  if (CPU_COUNT(&allowed) < 2) {GTEST_SKIP() << "Requires two allowed CPUs";}
  omp_set_num_threads(2);
  Planner planner;
  EXPECT_DOUBLE_EQ(planner.run().traj.velocity.x, 2.0);
  EXPECT_EQ(planner.collision->cached, 4u);
  EXPECT_EQ(planner.collision->scored, 3);
}

TEST_F(Scoring, SingleCpuAffinityOverridesRequestedParallelWorkers)
{
  cpu_set_t original;
  CPU_ZERO(&original);
  ASSERT_EQ(sched_getaffinity(0, sizeof(original), &original), 0);
  struct RestoreAffinity
  {
    cpu_set_t allowed;
    ~RestoreAffinity() {sched_setaffinity(0, sizeof(allowed), &allowed);}
  } restore{original};
  cpu_set_t single;
  CPU_ZERO(&single);
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &original)) {CPU_SET(cpu, &single); break;}
  }
  ASSERT_EQ(sched_setaffinity(0, sizeof(single), &single), 0);
  omp_set_num_threads(2);
  Planner planner;
  EXPECT_DOUBLE_EQ(planner.run().traj.velocity.x, 2.0);
  EXPECT_EQ(planner.collision->cached, 0u);
  EXPECT_EQ(planner.collision->scored, 1);
}
}  // namespace
