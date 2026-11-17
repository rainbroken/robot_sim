#ifndef DOG_NAV__CACHED_OBSTACLE_FOOTPRINT_CRITIC_HPP_
#define DOG_NAV__CACHED_OBSTACLE_FOOTPRINT_CRITIC_HPP_

#include <unordered_map>
#include <string>

#include "dog_nav/trajectory_cache.hpp"
#include "dwb_critics/obstacle_footprint.hpp"

namespace dog_nav
{

class CachedObstacleFootprintCritic
  : public dwb_critics::ObstacleFootprintCritic,
  public TrajectoryCacheProvider
{
public:
  void prepareTrajectoryCache(
    const std::vector<dwb_msgs::msg::Trajectory2D> & trajectories) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override;

private:
  struct CachedScore
  {
    bool legal{false};
    double score{0.0};
    std::string reason;
  };

  std::unordered_map<const dwb_msgs::msg::Trajectory2D *, CachedScore> cache_;
};

}  // namespace dog_nav

#endif  // DOG_NAV__CACHED_OBSTACLE_FOOTPRINT_CRITIC_HPP_
