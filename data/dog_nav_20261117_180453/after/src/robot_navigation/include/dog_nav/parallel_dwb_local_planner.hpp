#ifndef DOG_NAV__PARALLEL_DWB_LOCAL_PLANNER_HPP_
#define DOG_NAV__PARALLEL_DWB_LOCAL_PLANNER_HPP_

#include <memory>

#include "dwb_core/dwb_local_planner.hpp"

namespace dog_nav
{

/**
 * DWB controller with parallel, deterministic trajectory scoring.
 *
 * Twist generation and critic prepare/debrief remain serial because those APIs
 * are stateful. Individual critic scores are evaluated concurrently after all
 * trajectories have been materialized. With only one allowed CPU or OpenMP
 * worker, use native DWB scoring with early rejection and score short-circuiting.
 */
class ParallelDwbLocalPlanner : public dwb_core::DWBLocalPlanner
{
protected:
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override;

private:
  dwb_msgs::msg::TrajectoryScore scoreParallelTrajectories(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results);
};

}  // namespace dog_nav

#endif  // DOG_NAV__PARALLEL_DWB_LOCAL_PLANNER_HPP_
