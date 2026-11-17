#include "dog_slam/colored_map_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include <Eigen/Geometry>

namespace dog_slam
{
namespace
{
struct VoxelKey
{
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int32_t>{}(key.x);
    seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct ColorAccumulator
{
  Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_rgb_sum = Eigen::Vector3d::Zero();
  std::uint32_t count = 0;
};

VoxelKey voxelKey(const Eigen::Vector3d & point, double voxel_size)
{
  return VoxelKey{
    static_cast<std::int32_t>(std::floor(point.x() / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.y() / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.z() / voxel_size))};
}

double srgbToLinear(std::uint8_t value)
{
  static const std::array<double, 256> lookup = []() {
      std::array<double, 256> values{};
      for (std::size_t index = 0; index < values.size(); ++index) {
        const double normalized = static_cast<double>(index) / 255.0;
        values[index] = normalized <= 0.04045 ? normalized / 12.92 :
          std::pow((normalized + 0.055) / 1.055, 2.4);
      }
      return values;
    }();
  return lookup[value];
}

std::uint8_t linearToSrgb(double value)
{
  static const std::array<std::uint8_t, 65536> lookup = []() {
      std::array<std::uint8_t, 65536> values{};
      for (std::size_t index = 0; index < values.size(); ++index) {
        const double linear = static_cast<double>(index) /
          static_cast<double>(values.size() - 1);
        const double srgb = linear <= 0.0031308 ? linear * 12.92 :
          1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
        values[index] = static_cast<std::uint8_t>(
          std::clamp(std::lround(srgb * 255.0), 0L, 255L));
      }
      return values;
    }();
  const auto index = static_cast<std::size_t>(
    std::lround(std::clamp(value, 0.0, 1.0) * (lookup.size() - 1)));
  return lookup[index];
}
}  // namespace

ColoredMapResult buildColoredMap(
  const Cloud & geometry, const std::vector<Keyframe> & keyframes,
  double voxel_size, std::uint8_t default_red,
  std::uint8_t default_green, std::uint8_t default_blue)
{
  ColoredMapResult result;
  voxel_size = std::max(0.02, voxel_size);
  std::unordered_map<VoxelKey, ColorAccumulator, VoxelKeyHash> colors;
  std::size_t estimated_color_points = 0;
  for (const auto & keyframe : keyframes) {
    estimated_color_points += keyframe.colored_cloud ? keyframe.colored_cloud->size() : 0U;
  }
  colors.reserve(estimated_color_points);

  for (const auto & keyframe : keyframes) {
    if (!keyframe.colored_cloud || keyframe.colored_cloud->empty()) {
      continue;
    }
    ++result.colored_keyframe_count;
    for (const auto & point : *keyframe.colored_cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Eigen::Vector3d world = keyframe.optimized_pose *
        Eigen::Vector3d(point.x, point.y, point.z);
      auto & accumulator = colors[voxelKey(world, voxel_size)];
      accumulator.position_sum += world;
      accumulator.linear_rgb_sum += Eigen::Vector3d(
        srgbToLinear(point.r), srgbToLinear(point.g), srgbToLinear(point.b));
      ++accumulator.count;
    }
  }

  result.cloud->reserve(geometry.size());
  const double maximum_color_distance_squared = 3.0 * voxel_size * voxel_size;
  for (const auto & point : geometry) {
    PointXYZIRGB output;
    output.x = point.x;
    output.y = point.y;
    output.z = point.z;
    output.intensity = point.intensity;
    output.r = default_red;
    output.g = default_green;
    output.b = default_blue;

    const Eigen::Vector3d position(point.x, point.y, point.z);
    const VoxelKey center = voxelKey(position, voxel_size);
    const ColorAccumulator * best = nullptr;
    double best_distance_squared = std::numeric_limits<double>::infinity();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto iterator = colors.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
          if (iterator == colors.end() || iterator->second.count == 0) {
            continue;
          }
          const Eigen::Vector3d mean_position =
            iterator->second.position_sum / static_cast<double>(iterator->second.count);
          const double distance_squared = (mean_position - position).squaredNorm();
          if (distance_squared < best_distance_squared) {
            best = &iterator->second;
            best_distance_squared = distance_squared;
          }
        }
      }
    }
    if (best && best_distance_squared <= maximum_color_distance_squared) {
      const Eigen::Vector3d mean_color =
        best->linear_rgb_sum / static_cast<double>(best->count);
      output.r = linearToSrgb(mean_color.x());
      output.g = linearToSrgb(mean_color.y());
      output.b = linearToSrgb(mean_color.z());
      ++result.colored_point_count;
    }
    result.cloud->push_back(output);
  }
  return result;
}

}  // namespace dog_slam
