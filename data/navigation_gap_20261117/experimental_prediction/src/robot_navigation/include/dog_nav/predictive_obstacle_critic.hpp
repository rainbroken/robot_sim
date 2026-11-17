#ifndef DOG_NAV__PREDICTIVE_OBSTACLE_CRITIC_HPP_
#define DOG_NAV__PREDICTIVE_OBSTACLE_CRITIC_HPP_

#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dog_nav
{
// Short-horizon geometric tracking of compact lethal-cell clusters in the
// costmap frame. This does not classify people or predict human intent.
class PredictiveObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  using Cluster = std::vector<geometry_msgs::msg::Point>;
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;

protected:
  void updateTracks(const std::vector<Cluster> & clusters, double now);
  struct Track
  {
    Cluster points;
    double x{0}, y{0}, vx{0}, vy{0};
    double radius{0.2};
    double last_motion{-1};
    std::vector<std::pair<double, geometry_msgs::msg::Point>> history;
    unsigned moving_updates{0};
  };
  std::vector<Track> tracks_;
  Cluster footprint_;
  double last_update_{-1}, prediction_age_{0};

private:
  rclcpp::Clock::SharedPtr clock_;
  double horizon_{1.0}, margin_{0.04}, min_speed_{0.08}, max_speed_{0.8};
  double max_cluster_extent_{0.8}, association_distance_{0.5};
};
}  // namespace dog_nav
#endif
