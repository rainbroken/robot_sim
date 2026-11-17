#include "dog_nav/cached_obstacle_footprint_critic.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dog_nav
{

void CachedObstacleFootprintCritic::onInit()
{
  dwb_critics::ObstacleFootprintCritic::onInit();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Obstacle critic node expired");
  }
  const std::string parameter = dwb_plugin_name_ + "." + name_ + ".score_center";
  nav2_util::declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(false));
  score_center_ = node->get_parameter(parameter).as_bool();
}

double CachedObstacleFootprintCritic::scoreUncachedTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  // Always validate the full, oriented footprint at EVERY trajectory pose.
  const double footprint_score = dwb_critics::ObstacleFootprintCritic::scoreTrajectory(trajectory);
  if (!score_center_) {
    return footprint_score;
  }
  // In a narrow passage both footprint edges can lie in the saturated 253
  // inscribed band. That score cannot distinguish centered from wall-hugging
  // trajectories. Rank by the center's inflation gradient after shape checking.
  // The center check also rejects lethal/unknown cells inside the perimeter.
  double score = 0.0;
  for (const auto & pose : trajectory.poses) {
    const double center_score = dwb_critics::BaseObstacleCritic::scorePose(pose);
    score += center_score;
  }
  // Mean rather than endpoint-only ranking penalizes close passes in the
  // middle of a trajectory, without depending on sample count / turn rate.
  return sum_scores_ || trajectory.poses.empty() ? score : score / trajectory.poses.size();
}

void CachedObstacleFootprintCritic::prepareTrajectoryCache(
  const std::vector<dwb_msgs::msg::Trajectory2D> & trajectories)
{
  cache_.clear();
  if (trajectories.empty()) {
    return;
  }
  cache_.reserve(trajectories.size());
  std::vector<CachedScore> scores(trajectories.size());

#pragma omp parallel for schedule(static)
  for (std::ptrdiff_t index = 0;
    index < static_cast<std::ptrdiff_t>(trajectories.size()); ++index)
  {
    auto & output = scores[static_cast<std::size_t>(index)];
    try {
      output.score = scoreUncachedTrajectory(
        trajectories[static_cast<std::size_t>(index)]);
      output.legal = true;
    } catch (const dwb_core::IllegalTrajectoryException & exception) {
      output.reason = exception.what();
    }
  }

  for (std::size_t index = 0; index < trajectories.size(); ++index) {
    cache_.emplace(&trajectories[index], std::move(scores[index]));
  }
}

double CachedObstacleFootprintCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory)
{
  const auto iterator = cache_.find(&trajectory);
  if (iterator == cache_.end()) {
    return scoreUncachedTrajectory(trajectory);
  }
  if (!iterator->second.legal) {
    throw dwb_core::IllegalTrajectoryException(name_, iterator->second.reason);
  }
  return iterator->second.score;
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::CachedObstacleFootprintCritic, dwb_core::TrajectoryCritic)
