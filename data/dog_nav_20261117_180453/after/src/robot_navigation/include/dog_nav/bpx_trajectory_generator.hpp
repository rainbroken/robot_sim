#ifndef DOG_NAV__BPX_TRAJECTORY_GENERATOR_HPP_
#define DOG_NAV__BPX_TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dog_nav
{

bool isBpxExecutableVelocity(
  double x, double y, double theta, double startup_threshold,
  double zero_epsilon = 1.0e-4);

class BpxVelocityIterator : public dwb_plugins::XYThetaIterator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node,
    dwb_plugins::KinematicsHandler::Ptr kinematics,
    const std::string & plugin_name) override;
  void startNewIteration(
    const nav_2d_msgs::msg::Twist2D & current_velocity, double dt) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;

protected:
  bool isValidVelocity() override;

private:
  double startup_threshold_{0.21};
  std::vector<nav_2d_msgs::msg::Twist2D> startup_twists_;
  std::size_t next_startup_twist_{0};
};

class BpxTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
protected:
  void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & node) override;
};

}  // namespace dog_nav

#endif  // DOG_NAV__BPX_TRAJECTORY_GENERATOR_HPP_
