#include "dog_nav/bpx_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "dwb_core/trajectory_generator.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

namespace dog_nav
{

bool isBpxExecutableVelocity(
  const double x, const double y, const double theta,
  const double startup_threshold, const double zero_epsilon)
{
  const double magnitude = std::sqrt(x * x + y * y + theta * theta);
  return magnitude <= zero_epsilon || magnitude >= startup_threshold;
}

void BpxVelocityIterator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & node,
  dwb_plugins::KinematicsHandler::Ptr kinematics,
  const std::string & plugin_name)
{
  dwb_plugins::XYThetaIterator::initialize(node, std::move(kinematics), plugin_name);

  const std::string parameter_name = plugin_name + ".bpx_startup_threshold";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_name, rclcpp::ParameterValue(0.21));
  startup_threshold_ = node->get_parameter(parameter_name).as_double();
  if (!std::isfinite(startup_threshold_) || startup_threshold_ < 0.20) {
    throw std::runtime_error(
            parameter_name + " must be finite and at least the BPX Walk threshold 0.20");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "BPX velocity iterator enabled with combined startup threshold %.3f",
    startup_threshold_);
}

void BpxVelocityIterator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity, const double dt)
{
  dwb_plugins::XYThetaIterator::startNewIteration(current_velocity, dt);

  auto kinematics = kinematics_handler_->getKinematics();
  startup_twists_.clear();
  next_startup_twist_ = 0;
  // The acceleration window at rest is below the firmware startup threshold.
  // Offer executable pure-axis commands in all enabled directions, so lateral
  // motion and reverse do not depend on adding forward motion or yaw.
  const auto add_startup = [this](
    const double current, const double minimum, const double maximum,
    const int axis, const double direction)
    {
      const double speed = direction * startup_threshold_;
      if (direction * current >= startup_threshold_ || speed < minimum || speed > maximum) {
        return;
      }
      nav_2d_msgs::msg::Twist2D twist;
      if (axis == 0) {
        twist.x = speed;
      } else if (axis == 1) {
        twist.y = speed;
      } else {
        twist.theta = speed;
      }
      if (isValidSpeed(twist.x, twist.y, twist.theta)) {
        startup_twists_.push_back(twist);
      }
    };
  for (const double direction : {1.0, -1.0}) {
    add_startup(current_velocity.x, kinematics.getMinX(), kinematics.getMaxX(), 0, direction);
    add_startup(current_velocity.y, kinematics.getMinY(), kinematics.getMaxY(), 1, direction);
    add_startup(
      current_velocity.theta, kinematics.getMinTheta(), kinematics.getMaxTheta(), 2, direction);
  }
}

bool BpxVelocityIterator::hasMoreTwists()
{
  return next_startup_twist_ < startup_twists_.size() ||
         dwb_plugins::XYThetaIterator::hasMoreTwists();
}

nav_2d_msgs::msg::Twist2D BpxVelocityIterator::nextTwist()
{
  if (next_startup_twist_ < startup_twists_.size()) {
    return startup_twists_[next_startup_twist_++];
  }
  return dwb_plugins::XYThetaIterator::nextTwist();
}

bool BpxVelocityIterator::isValidVelocity()
{
  const double x = x_it_->getVelocity();
  const double y = y_it_->getVelocity();
  const double theta = th_it_->getVelocity();
  const double magnitude = std::sqrt(x * x + y * y + theta * theta);
  if (magnitude <= 1.0e-4) {
    return true;
  }
  return isValidSpeed(x, y, theta) &&
         magnitude >= startup_threshold_;
}

void BpxTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & node)
{
  velocity_iterator_ = std::make_shared<BpxVelocityIterator>();
  velocity_iterator_->initialize(node, kinematics_handler_, plugin_name_);
}

}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::BpxTrajectoryGenerator, dwb_core::TrajectoryGenerator)
