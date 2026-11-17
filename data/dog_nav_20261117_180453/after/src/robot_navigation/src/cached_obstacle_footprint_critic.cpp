#include "dog_nav/cached_obstacle_footprint_critic.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dog_nav
{

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
      output.score = dwb_critics::ObstacleFootprintCritic::scoreTrajectory(
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
    return dwb_critics::ObstacleFootprintCritic::scoreTrajectory(trajectory);
  }
  if (!iterator->second.legal) {
    throw dwb_core::IllegalTrajectoryException(name_, iterator->second.reason);
  }
  return iterator->second.score;
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::CachedObstacleFootprintCritic, dwb_core::TrajectoryCritic)
