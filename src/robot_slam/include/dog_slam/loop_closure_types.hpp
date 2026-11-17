#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>

namespace dog_slam
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using ColoredCloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct SolidDescriptor
{
  Eigen::VectorXd range;
  Eigen::VectorXd azimuth;
};

struct Keyframe
{
  std::size_t id = 0;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Isometry3d local_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
  Cloud::Ptr cloud = std::make_shared<Cloud>();
  Cloud::Ptr map_cloud;
  ColoredCloud::Ptr colored_cloud;
  SolidDescriptor descriptor;
};

struct LoopCandidate
{
  std::size_t index = 0;
  double descriptor_score = 0.0;
  double yaw_rad = 0.0;
};

struct RegistrationResult
{
  bool accepted = false;
  bool converged = false;
  std::size_t iterations = 0;
  Eigen::Isometry3d target_from_source = Eigen::Isometry3d::Identity();
  std::size_t num_inliers = 0;
  double overlap = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
  double min_eigenvalue = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  std::string rejection_reason;
};

}  // namespace dog_slam
