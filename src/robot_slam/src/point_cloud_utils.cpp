#include "dog_slam/point_cloud_utils.hpp"

#include <algorithm>

#include <pcl/filters/voxel_grid.h>

namespace dog_slam
{

Cloud::Ptr voxelized(const Cloud::ConstPtr & input, double leaf_size)
{
  auto output = std::make_shared<Cloud>();
  if (!input || input->empty()) {
    return output;
  }
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(input);
  const float leaf = static_cast<float>(std::max(0.01, leaf_size));
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  return output;
}

}  // namespace dog_slam
