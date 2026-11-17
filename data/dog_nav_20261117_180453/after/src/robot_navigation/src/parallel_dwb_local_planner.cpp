#include "dog_nav/parallel_dwb_local_planner.hpp"
#include "dog_nav/trajectory_cache.hpp"

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <omp.h>
#include <sched.h>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dog_nav
{

namespace
{
struct ScoredTrajectory
{
  dwb_msgs::msg::TrajectoryScore score;
  bool legal{false};
  std::string critic;
  std::string reason;
};
}  // namespace

dwb_msgs::msg::TrajectoryScore ParallelDwbLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  const bool single_cpu = sched_getaffinity(0, sizeof(allowed), &allowed) == 0 &&
    CPU_COUNT(&allowed) < 2;
  const bool serial = single_cpu || omp_get_max_threads() < 2;
  const auto start = std::chrono::steady_clock::now();
  const auto report_duration = [&]() {
      const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      if (milliseconds > 100.0) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Slow DWB scoring: %.1fms mode=%s velocity=(%.3f, %.3f, %.3f)",
          milliseconds, serial ? "serial" : "parallel", velocity.x, velocity.y, velocity.theta);
      }
    };
  try {
    dwb_msgs::msg::TrajectoryScore best;
    if (serial) {
      // A single pinned CPU cannot benefit from OpenMP. In particular, do not
      // precompute expensive footprints for commands rejected by earlier critics.
      // Clear caches as affinity / OpenMP settings may change between iterations.
      for (const auto & critic : critics_) {
        auto * provider = dynamic_cast<TrajectoryCacheProvider *>(critic.get());
        if (provider) {
          provider->prepareTrajectoryCache({});
        }
      }
      best = dwb_core::DWBLocalPlanner::coreScoringAlgorithm(pose, velocity, results);
    } else {
      best = scoreParallelTrajectories(pose, velocity, results);
    }
    report_duration();
    return best;
  } catch (const dwb_core::NoLegalTrajectoriesException & exception) {
    report_duration();
    // Keep the reason in rosout even when publishing full trajectory evaluations
    // is disabled. The generic "No valid trajectories" loses this information.
    for (const auto & entry : exception.tracker_.getPercentages()) {
      RCLCPP_WARN(
        logger_, "DWB rejected %.1f%%: %s: %s", entry.second * 100.0,
        entry.first.first.c_str(), entry.first.second.c_str());
    }
    throw;
  }
}

dwb_msgs::msg::TrajectoryScore ParallelDwbLocalPlanner::scoreParallelTrajectories(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  std::vector<dwb_msgs::msg::Trajectory2D> trajectories;
  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    const auto twist = traj_generator_->nextTwist();
    trajectories.push_back(traj_generator_->generateTrajectory(pose, velocity, twist));
  }

  for (const auto & critic : critics_) {
    auto * provider = dynamic_cast<TrajectoryCacheProvider *>(critic.get());
    if (provider) {
      provider->prepareTrajectoryCache(trajectories);
    }
  }

  std::vector<ScoredTrajectory> scored(trajectories.size());

  // Critics are prepared before entering this method and debriefed after it
  // returns. Their per-trajectory score calls are read-only for the configured
  // DWB critics, so only this independent scoring phase is parallelized.
#pragma omp parallel for schedule(static)
  for (std::ptrdiff_t index = 0;
    index < static_cast<std::ptrdiff_t>(trajectories.size()); ++index)
  {
    auto & output = scored[static_cast<std::size_t>(index)];
    try {
      // Passing -1 disables order-dependent short-circuiting. The final
      // reduction below preserves the original trajectory order and tie rule.
      output.score = scoreTrajectory(trajectories[static_cast<std::size_t>(index)], -1.0);
      output.legal = true;
    } catch (const dwb_core::IllegalTrajectoryException & exception) {
      output.critic = exception.getCriticName();
      output.reason = exception.what();
      dwb_msgs::msg::CriticScore critic_score;
      critic_score.name = output.critic;
      critic_score.raw_score = -1.0;
      output.score.scores.push_back(critic_score);
      output.score.total = -1.0;
    }
  }

  dwb_core::IllegalTrajectoryTracker tracker;
  dwb_msgs::msg::TrajectoryScore best;
  dwb_msgs::msg::TrajectoryScore worst;
  bool have_best = false;
  bool have_worst = false;
  for (std::size_t index = 0; index < scored.size(); ++index) {
    const auto & output = scored[index];
    if (results) {
      results->twists.push_back(output.score);
    }
    if (!output.legal) {
      tracker.addIllegalTrajectory(
        dwb_core::IllegalTrajectoryException(output.critic, output.reason));
    } else {
      tracker.addLegalTrajectory();
      if (!have_best || output.score.total < best.total) {
        best = output.score;
        have_best = true;
        if (results) {
          results->best_index = index;
        }
      }
      if (!have_worst || output.score.total > worst.total) {
        worst = output.score;
        have_worst = true;
        if (results) {
          results->worst_index = index;
        }
      }
    }
  }

  if (!have_best) {
    throw dwb_core::NoLegalTrajectoriesException(tracker);
  }
  return best;
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::ParallelDwbLocalPlanner, nav2_core::Controller)
