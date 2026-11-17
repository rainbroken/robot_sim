#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include "dog_slam/loop_closure_types.hpp"

namespace dog_slam
{

struct EIGEN_ALIGN16 PointXYZIRGB
{
  PCL_ADD_POINT4D;
  float intensity = 0.0F;
  PCL_ADD_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using ColoredMapCloud = pcl::PointCloud<PointXYZIRGB>;

struct ColoredMapResult
{
  ColoredMapCloud::Ptr cloud = std::make_shared<ColoredMapCloud>();
  std::size_t colored_point_count = 0;
  std::size_t colored_keyframe_count = 0;
};

ColoredMapResult buildColoredMap(
  const Cloud & geometry, const std::vector<Keyframe> & keyframes,
  double voxel_size, std::uint8_t default_red,
  std::uint8_t default_green, std::uint8_t default_blue);

}  // namespace dog_slam

POINT_CLOUD_REGISTER_POINT_STRUCT(
  dog_slam::PointXYZIRGB,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, rgb, rgb)
)
