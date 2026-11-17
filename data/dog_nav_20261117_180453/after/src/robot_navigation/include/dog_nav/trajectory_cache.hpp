#ifndef DOG_NAV__TRAJECTORY_CACHE_HPP_
#define DOG_NAV__TRAJECTORY_CACHE_HPP_

#include <vector>

#include "dwb_msgs/msg/trajectory2_d.hpp"

namespace dog_nav
{

class TrajectoryCacheProvider
{
public:
  virtual ~TrajectoryCacheProvider() = default;
  virtual void prepareTrajectoryCache(
    const std::vector<dwb_msgs::msg::Trajectory2D> & trajectories) = 0;
};

}  // namespace dog_nav

#endif  // DOG_NAV__TRAJECTORY_CACHE_HPP_
