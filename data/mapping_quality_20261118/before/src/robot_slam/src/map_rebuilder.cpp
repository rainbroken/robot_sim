#include "dog_slam/map_rebuilder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "dog_slam/point_cloud_utils.hpp"

namespace dog_slam
{
namespace
{
struct VoxelKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

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

struct PointRange
{
  std::size_t begin;
  std::size_t end;
  Eigen::Vector3d sensor;
};

struct OccupancyEndpoint
{
  std::int32_t x;
  std::int32_t y;
  int delta;
};

struct LocalizationAccumulator
{
  Eigen::Vector4d sum = Eigen::Vector4d::Zero();
  std::size_t point_count = 0;
  std::size_t observations = 0;
  std::size_t last_keyframe = std::numeric_limits<std::size_t>::max();
};

struct VoxelAccumulator
{
  Eigen::Vector4d sum = Eigen::Vector4d::Zero();
  std::size_t count = 0;
  std::uint64_t order = 0;
};

struct DynamicEvidence
{
  std::size_t hit_keyframes = 0;
  std::size_t miss_keyframes = 0;
  double log_odds = 0.0;
  std::vector<VoxelKey> observer_positions;
};

using Clock = std::chrono::steady_clock;

double elapsedSeconds(const Clock::time_point & start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::int64_t cellKey(std::int32_t x, std::int32_t y)
{
  return static_cast<std::int64_t>(
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
    static_cast<std::uint32_t>(y));
}

VoxelKey voxelKey(const pcl::PointXYZI & point, double voxel_size)
{
  return VoxelKey{
    static_cast<std::int32_t>(std::floor(point.x / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.y / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.z / voxel_size))};
}

VoxelKey voxelKey(const Eigen::Vector3d & point, double voxel_size)
{
  return VoxelKey{
    static_cast<std::int32_t>(std::floor(point.x() / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.y() / voxel_size)),
    static_cast<std::int32_t>(std::floor(point.z() / voxel_size))};
}

Cloud::ConstPtr mapCloud(const Keyframe & keyframe)
{
  if (keyframe.map_cloud && !keyframe.map_cloud->empty()) {
    return keyframe.map_cloud;
  }
  return keyframe.cloud;
}

Eigen::Vector3d sensorOrigin(
  const Keyframe & keyframe, const MapRebuilder::Config & config)
{
  return keyframe.optimized_pose * Eigen::Vector3d(
    config.lidar_origin_x, config.lidar_origin_y, config.lidar_origin_z);
}

double probabilityToLogOdds(double probability)
{
  constexpr double epsilon = 1.0e-6;
  const double clamped = std::clamp(probability, epsilon, 1.0 - epsilon);
  return std::log(clamped / (1.0 - clamped));
}

double endpointMargin(double range, const MapRebuilder::Config & config)
{
  const double base = std::max(0.0, config.dynamic_filter_endpoint_margin_base);
  const double scaled = base +
    std::max(0.0, config.dynamic_filter_endpoint_margin_per_meter) * range;
  return std::clamp(
    scaled, base, std::max(base, config.dynamic_filter_endpoint_margin_max));
}

void traceFreeVoxels(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & endpoint,
  double voxel_size, double endpoint_margin,
  const std::unordered_map<VoxelKey, DynamicEvidence, VoxelKeyHash> & candidates,
  std::unordered_set<VoxelKey, VoxelKeyHash> & free_voxels)
{
  const Eigen::Vector3d ray = endpoint - origin;
  const double range = ray.norm();
  if (range <= endpoint_margin || range < 1.0e-9) {
    return;
  }

  const double end_t = std::clamp(1.0 - endpoint_margin / range, 0.0, 1.0);
  VoxelKey current = voxelKey(origin, voxel_size);
  const VoxelKey endpoint_key = voxelKey(endpoint, voxel_size);
  const int steps[3] = {
    ray.x() > 0.0 ? 1 : (ray.x() < 0.0 ? -1 : 0),
    ray.y() > 0.0 ? 1 : (ray.y() < 0.0 ? -1 : 0),
    ray.z() > 0.0 ? 1 : (ray.z() < 0.0 ? -1 : 0)};
  const double infinity = std::numeric_limits<double>::infinity();
  double t_max[3];
  double t_delta[3];
  const double coordinates[3] = {origin.x(), origin.y(), origin.z()};
  const double directions[3] = {ray.x(), ray.y(), ray.z()};
  const std::int32_t indices[3] = {current.x, current.y, current.z};
  for (int axis = 0; axis < 3; ++axis) {
    if (steps[axis] == 0) {
      t_max[axis] = infinity;
      t_delta[axis] = infinity;
      continue;
    }
    const double boundary = static_cast<double>(
      indices[axis] + (steps[axis] > 0 ? 1 : 0)) * voxel_size;
    t_max[axis] = (boundary - coordinates[axis]) / directions[axis];
    t_delta[axis] = voxel_size / std::abs(directions[axis]);
  }

  while (!(current == endpoint_key)) {
    const double next_t = std::min({t_max[0], t_max[1], t_max[2]});
    if (!std::isfinite(next_t) || next_t >= end_t) {
      break;
    }
    const double tolerance = 1.0e-12 * std::max(1.0, std::abs(next_t));
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(t_max[axis] - next_t) > tolerance) {
        continue;
      }
      if (axis == 0) {
        current.x += steps[axis];
      } else if (axis == 1) {
        current.y += steps[axis];
      } else {
        current.z += steps[axis];
      }
      t_max[axis] += t_delta[axis];
    }
    if (!(current == endpoint_key) && candidates.find(current) != candidates.end()) {
      free_voxels.insert(current);
    }
  }
}

bool stableDynamicVoxel(
  const DynamicEvidence & evidence, const MapRebuilder::Config & config)
{
  if (evidence.hit_keyframes < static_cast<std::size_t>(
      std::max(1, config.dynamic_filter_min_hit_keyframes)))
  {
    return false;
  }
  if (evidence.observer_positions.size() < static_cast<std::size_t>(
      std::max(1, config.dynamic_filter_min_observer_positions)))
  {
    return false;
  }
  return evidence.log_odds >= probabilityToLogOdds(
    config.dynamic_filter_occupancy_threshold);
}

Cloud::Ptr dynamicFilteredCloud(
  const Cloud & transformed, const std::vector<PointRange> & ranges,
  const MapRebuilder::Config & config, std::vector<PointRange> & filtered_ranges)
{
  const double voxel_size = std::max(0.02, config.dynamic_filter_voxel_size);
  const double max_range = std::max(0.0, config.dynamic_filter_max_range);
  const double observer_voxel_size = std::max(0.05, config.dynamic_filter_observer_voxel_size);
  const double hit_update = probabilityToLogOdds(config.dynamic_filter_prob_hit);
  const double miss_update = probabilityToLogOdds(config.dynamic_filter_prob_miss);
  const double first_clamp = probabilityToLogOdds(config.dynamic_filter_clamping_min);
  const double second_clamp = probabilityToLogOdds(config.dynamic_filter_clamping_max);
  const double clamp_min = std::min(first_clamp, second_clamp);
  const double clamp_max = std::max(first_clamp, second_clamp);
  const std::size_t required_observers = static_cast<std::size_t>(
    std::max(1, config.dynamic_filter_min_observer_positions));
  std::unordered_map<VoxelKey, DynamicEvidence, VoxelKeyHash> evidence;
  evidence.reserve(transformed.size() / 2U + 1U);

  // Only endpoint voxels can survive into the output. Pre-collect them so free-space
  // traversal does not allocate evidence for every empty voxel along every ray.
  for (const auto & range : ranges) {
    for (std::size_t index = range.begin; index < range.end; ++index) {
      const auto & point = transformed[index];
      const Eigen::Vector3d endpoint(point.x, point.y, point.z);
      const double distance = (endpoint - range.sensor).norm();
      if (std::isfinite(distance) && (max_range <= 0.0 || distance <= max_range)) {
        evidence.try_emplace(voxelKey(point, voxel_size));
      }
    }
  }

  for (const auto & range : ranges) {
    std::unordered_set<VoxelKey, VoxelKeyHash> hits;
    std::unordered_set<VoxelKey, VoxelKeyHash> misses;
    hits.reserve(range.end - range.begin);
    const VoxelKey observer = voxelKey(range.sensor, observer_voxel_size);
    for (std::size_t index = range.begin; index < range.end; ++index) {
      const auto & point = transformed[index];
      const Eigen::Vector3d endpoint(point.x, point.y, point.z);
      const double distance = (endpoint - range.sensor).norm();
      if (!std::isfinite(distance) || distance < 1.0e-9) {
        continue;
      }
      if (max_range <= 0.0 || distance <= max_range) {
        hits.insert(voxelKey(point, voxel_size));
        traceFreeVoxels(
          range.sensor, endpoint, voxel_size, endpointMargin(distance, config), evidence, misses);
      } else {
        const Eigen::Vector3d truncated =
          range.sensor + (endpoint - range.sensor) * (max_range / distance);
        traceFreeVoxels(range.sensor, truncated, voxel_size, 0.0, evidence, misses);
      }
    }
    for (const auto & hit : hits) {
      misses.erase(hit);
    }
    for (const auto & miss : misses) {
      auto & value = evidence.at(miss);
      ++value.miss_keyframes;
      value.log_odds = std::clamp(value.log_odds + miss_update, clamp_min, clamp_max);
    }
    for (const auto & hit : hits) {
      auto & value = evidence.at(hit);
      ++value.hit_keyframes;
      value.log_odds = std::clamp(value.log_odds + hit_update, clamp_min, clamp_max);
      if (value.observer_positions.size() < required_observers &&
        std::find(
          value.observer_positions.begin(), value.observer_positions.end(), observer) ==
        value.observer_positions.end())
      {
        value.observer_positions.push_back(observer);
      }
    }
  }

  auto filtered = std::make_shared<Cloud>();
  filtered->reserve(transformed.size());
  filtered_ranges.reserve(ranges.size());
  for (const auto & range : ranges) {
    const auto begin = filtered->size();
    for (std::size_t index = range.begin; index < range.end; ++index) {
      const auto & point = transformed[index];
      const double distance = (Eigen::Vector3d(point.x, point.y, point.z) - range.sensor).norm();
      if (max_range <= 0.0 || distance > max_range) {
        filtered->push_back(point);
        continue;
      }
      const auto iterator = evidence.find(voxelKey(point, voxel_size));
      if (iterator != evidence.end() && stableDynamicVoxel(iterator->second, config)) {
        filtered->push_back(point);
      }
    }
    filtered_ranges.push_back(PointRange{begin, filtered->size(), range.sensor});
  }
  return filtered;
}

Cloud::Ptr radiusOutlierFiltered(
  const Cloud::ConstPtr & input, const MapRebuilder::Config & config)
{
  if (!input || input->empty() || !config.outlier_filter_enabled) {
    return input ? std::make_shared<Cloud>(*input) : std::make_shared<Cloud>();
  }
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(std::max(0.01, config.outlier_filter_radius));
  filter.setMinNeighborsInRadius(std::max(1, config.outlier_filter_min_neighbors));
  auto output = std::make_shared<Cloud>();
  filter.filter(*output);
  return output;
}

void updateRawCell(
  std::unordered_map<std::int64_t, int> & cells, std::int32_t x, std::int32_t y, int delta)
{
  const auto key = cellKey(x, y);
  auto iterator = cells.find(key);
  if (iterator == cells.end()) {
    if (delta != 0) {
      cells.emplace(key, delta);
    }
    return;
  }
  iterator->second += delta;
  if (iterator->second == 0) {
    cells.erase(iterator);
  }
}

void traceFreeRaw(
  std::unordered_map<std::int64_t, int> & cells, std::int32_t x0, std::int32_t y0,
  std::int32_t x1, std::int32_t y1)
{
  const std::int32_t dx = std::abs(x1 - x0);
  const std::int32_t sx = x0 < x1 ? 1 : -1;
  const std::int32_t dy = -std::abs(y1 - y0);
  const std::int32_t sy = y0 < y1 ? 1 : -1;
  std::int32_t error = dx + dy;
  while (x0 != x1 || y0 != y1) {
    updateRawCell(cells, x0, y0, -1);
    const std::int32_t doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

std::int32_t keyX(std::int64_t key)
{
  return static_cast<std::int32_t>(static_cast<std::uint64_t>(key) >> 32U);
}

std::int32_t keyY(std::int64_t key)
{
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(key));
}

void updateCell(
  std::unordered_map<std::int64_t, int> & cells, std::int32_t x, std::int32_t y, int delta)
{
  auto & value = cells[cellKey(x, y)];
  value = std::clamp(value + delta, -20, 20);
}

void traceFree(
  std::unordered_map<std::int64_t, int> & cells, std::int32_t x0, std::int32_t y0,
  std::int32_t x1, std::int32_t y1)
{
  const std::int32_t dx = std::abs(x1 - x0);
  const std::int32_t sx = x0 < x1 ? 1 : -1;
  const std::int32_t dy = -std::abs(y1 - y0);
  const std::int32_t sy = y0 < y1 ? 1 : -1;
  std::int32_t error = dx + dy;
  while (x0 != x1 || y0 != y1) {
    updateCell(cells, x0, y0, -1);
    const std::int32_t doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

void updateDenseCell(
  std::vector<std::int8_t> & cells, std::size_t width, std::int32_t min_x,
  std::int32_t min_y, std::int32_t x, std::int32_t y, int delta)
{
  constexpr std::int8_t untouched = std::numeric_limits<std::int8_t>::max();
  auto & cell = cells[
    static_cast<std::size_t>(y - min_y) * width + static_cast<std::size_t>(x - min_x)];
  const int value = cell == untouched ? 0 : static_cast<int>(cell);
  cell = static_cast<std::int8_t>(std::clamp(value + delta, -20, 20));
}

void traceFreeDense(
  std::vector<std::int8_t> & cells, std::size_t width, std::int32_t min_x,
  std::int32_t min_y, std::int32_t x0, std::int32_t y0,
  std::int32_t x1, std::int32_t y1)
{
  const std::int32_t dx = std::abs(x1 - x0);
  const std::int32_t sx = x0 < x1 ? 1 : -1;
  const std::int32_t dy = -std::abs(y1 - y0);
  const std::int32_t sy = y0 < y1 ? 1 : -1;
  std::int32_t error = dx + dy;
  while (x0 != x1 || y0 != y1) {
    updateDenseCell(cells, width, min_x, min_y, x0, y0, -1);
    const std::int32_t doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

Cloud::Ptr stableLocalizationCloudFromTransformed(
  const Cloud & transformed, const std::vector<PointRange> & ranges,
  double localization_voxel_size, int localization_min_observations)
{
  const double voxel_size = std::max(0.02, localization_voxel_size);
  std::unordered_map<VoxelKey, LocalizationAccumulator, VoxelKeyHash> accumulators;
  accumulators.reserve(transformed.size() / 2U + 1U);
  for (std::size_t keyframe_index = 0; keyframe_index < ranges.size(); ++keyframe_index) {
    const auto & range = ranges[keyframe_index];
    for (std::size_t point_index = range.begin; point_index < range.end; ++point_index) {
      const auto & point = transformed.points[point_index];
      const VoxelKey key{
        static_cast<std::int32_t>(std::floor(point.x / voxel_size)),
        static_cast<std::int32_t>(std::floor(point.y / voxel_size)),
        static_cast<std::int32_t>(std::floor(point.z / voxel_size))};
      auto & accumulator = accumulators[key];
      accumulator.sum += Eigen::Vector4d(point.x, point.y, point.z, point.intensity);
      ++accumulator.point_count;
      if (accumulator.last_keyframe != keyframe_index) {
        accumulator.last_keyframe = keyframe_index;
        ++accumulator.observations;
      }
    }
  }

  auto stable = std::make_shared<Cloud>();
  stable->reserve(accumulators.size());
  const std::size_t minimum_observations = std::min<std::size_t>(
    std::max(1, localization_min_observations), ranges.size());
  for (const auto & [key, accumulator] : accumulators) {
    (void)key;
    if (accumulator.observations < minimum_observations || accumulator.point_count == 0) {
      continue;
    }
    const Eigen::Vector4d mean = accumulator.sum /
      static_cast<double>(accumulator.point_count);
    pcl::PointXYZI point;
    point.x = static_cast<float>(mean.x());
    point.y = static_cast<float>(mean.y());
    point.z = static_cast<float>(mean.z());
    point.intensity = static_cast<float>(mean.w());
    stable->push_back(point);
  }
  return stable;
}

template<typename UpdateCell, typename TraceFree>
void accumulateOccupancy(
  const Cloud & transformed, const std::vector<PointRange> & ranges,
  double resolution, double obstacle_min_height, double obstacle_max_height,
  double ground_distance_threshold, double fallback_ground_below_sensor,
  double max_range, UpdateCell update_cell, TraceFree trace_free)
{
  const double max_range_squared = max_range * max_range;
  for (const auto & range : ranges) {
    const auto origin_x = static_cast<std::int32_t>(std::floor(range.sensor.x() / resolution));
    const auto origin_y = static_cast<std::int32_t>(std::floor(range.sensor.y() / resolution));
    std::vector<OccupancyEndpoint> endpoints;
    endpoints.reserve(range.end - range.begin);
    std::unordered_map<std::int64_t, std::size_t> endpoint_indices;
    endpoint_indices.reserve(range.end - range.begin);
    for (std::size_t point_index = range.begin; point_index < range.end; ++point_index) {
      const auto & point = transformed.points[point_index];
      const double dx = point.x - range.sensor.x();
      const double dy = point.y - range.sensor.y();
      if (dx * dx + dy * dy > max_range_squared) {
        continue;
      }
      const double height = point.z -
        (range.sensor.z() - fallback_ground_below_sensor);
      int delta = 0;
      if (std::abs(height) <= ground_distance_threshold) {
        delta = -1;
      } else if (height >= obstacle_min_height && height <= obstacle_max_height) {
        delta = 3;
      } else {
        continue;
      }
      const auto x = static_cast<std::int32_t>(std::floor(point.x / resolution));
      const auto y = static_cast<std::int32_t>(std::floor(point.y / resolution));
      const auto [iterator, inserted] = endpoint_indices.try_emplace(
        cellKey(x, y), endpoints.size());
      if (inserted) {
        endpoints.push_back(OccupancyEndpoint{x, y, delta});
      } else if (delta > endpoints[iterator->second].delta) {
        endpoints[iterator->second].delta = delta;
      }
    }
    for (const auto & endpoint : endpoints) {
      trace_free(origin_x, origin_y, endpoint.x, endpoint.y);
      update_cell(endpoint.x, endpoint.y, endpoint.delta);
    }
  }
}

std::optional<nav_msgs::msg::OccupancyGrid> occupancyFromTransformed(
  const Cloud & transformed, const std::vector<PointRange> & ranges,
  const MapRebuilder::Config & config, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  std::int32_t min_x = std::numeric_limits<std::int32_t>::max();
  std::int32_t min_y = std::numeric_limits<std::int32_t>::max();
  std::int32_t max_x = std::numeric_limits<std::int32_t>::min();
  std::int32_t max_y = std::numeric_limits<std::int32_t>::min();
  bool has_occupancy_points = false;
  const double max_range_squared =
    config.occupancy_max_range * config.occupancy_max_range;
  for (const auto & range : ranges) {
    const auto origin_x = static_cast<std::int32_t>(
      std::floor(range.sensor.x() / config.occupancy_resolution));
    const auto origin_y = static_cast<std::int32_t>(
      std::floor(range.sensor.y() / config.occupancy_resolution));
    for (std::size_t point_index = range.begin; point_index < range.end; ++point_index) {
      const auto & point = transformed.points[point_index];
      const double dx = point.x - range.sensor.x();
      const double dy = point.y - range.sensor.y();
      if (dx * dx + dy * dy > max_range_squared) {
        continue;
      }
      const double height = point.z -
        (range.sensor.z() - config.fallback_ground_below_sensor);
      if (std::abs(height) > config.ground_distance_threshold &&
        (height < config.obstacle_min_height || height > config.obstacle_max_height))
      {
        continue;
      }
      const auto x = static_cast<std::int32_t>(
        std::floor(point.x / config.occupancy_resolution));
      const auto y = static_cast<std::int32_t>(
        std::floor(point.y / config.occupancy_resolution));
      min_x = std::min({min_x, origin_x, x});
      min_y = std::min({min_y, origin_y, y});
      max_x = std::max({max_x, origin_x, x});
      max_y = std::max({max_y, origin_y, y});
      has_occupancy_points = true;
    }
  }
  if (!has_occupancy_points) {
    return std::nullopt;
  }

  const auto width64 = static_cast<std::uint64_t>(
    static_cast<std::int64_t>(max_x) - static_cast<std::int64_t>(min_x) + 1);
  const auto height64 = static_cast<std::uint64_t>(
    static_cast<std::int64_t>(max_y) - static_cast<std::int64_t>(min_y) + 1);
  constexpr std::uint64_t max_dense_cells = 128ULL * 1024ULL * 1024ULL;

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.resolution = static_cast<float>(config.occupancy_resolution);
  message.info.width = static_cast<std::uint32_t>(width64);
  message.info.height = static_cast<std::uint32_t>(height64);
  message.info.origin.position.x = min_x * config.occupancy_resolution;
  message.info.origin.position.y = min_y * config.occupancy_resolution;
  message.info.origin.orientation.w = 1.0;

  if (width64 <= std::numeric_limits<std::uint32_t>::max() &&
    height64 <= std::numeric_limits<std::uint32_t>::max() &&
    width64 <= max_dense_cells / height64)
  {
    constexpr std::int8_t untouched = std::numeric_limits<std::int8_t>::max();
    const auto width = static_cast<std::size_t>(width64);
    std::vector<std::int8_t> cells(static_cast<std::size_t>(width64 * height64), untouched);
    accumulateOccupancy(
      transformed, ranges, config.occupancy_resolution, config.obstacle_min_height,
      config.obstacle_max_height, config.ground_distance_threshold,
      config.fallback_ground_below_sensor, config.occupancy_max_range,
      [&](std::int32_t x, std::int32_t y, int delta) {
        updateDenseCell(cells, width, min_x, min_y, x, y, delta);
      },
      [&](std::int32_t x0, std::int32_t y0, std::int32_t x1, std::int32_t y1) {
        traceFreeDense(cells, width, min_x, min_y, x0, y0, x1, y1);
      });
    message.data.resize(cells.size());
    std::transform(
      cells.begin(), cells.end(), message.data.begin(),
      [](std::int8_t value) -> std::int8_t {
        return value == untouched ? -1 : (value >= 3 ? 100 : 0);
      });
    return message;
  }

  std::unordered_map<std::int64_t, int> cells;
  accumulateOccupancy(
    transformed, ranges, config.occupancy_resolution, config.obstacle_min_height,
    config.obstacle_max_height, config.ground_distance_threshold,
    config.fallback_ground_below_sensor, config.occupancy_max_range,
    [&](std::int32_t x, std::int32_t y, int delta) {updateCell(cells, x, y, delta);},
    [&](std::int32_t x0, std::int32_t y0, std::int32_t x1, std::int32_t y1) {
      traceFree(cells, x0, y0, x1, y1);
    });
  message.data.assign(static_cast<std::size_t>(width64 * height64), -1);
  for (const auto & [key, value] : cells) {
    const auto x = static_cast<std::uint32_t>(keyX(key) - min_x);
    const auto y = static_cast<std::uint32_t>(keyY(key) - min_y);
    message.data[static_cast<std::size_t>(y) * message.info.width + x] = value >= 3 ? 100 : 0;
  }
  return message;
}

std::optional<nav_msgs::msg::OccupancyGrid> occupancyFromRawCells(
  const std::unordered_map<std::int64_t, int> & cells,
  const MapRebuilder::Config & config, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  if (cells.empty()) {
    return std::nullopt;
  }
  std::int32_t min_x = std::numeric_limits<std::int32_t>::max();
  std::int32_t min_y = std::numeric_limits<std::int32_t>::max();
  std::int32_t max_x = std::numeric_limits<std::int32_t>::min();
  std::int32_t max_y = std::numeric_limits<std::int32_t>::min();
  for (const auto & entry : cells) {
    min_x = std::min(min_x, keyX(entry.first));
    min_y = std::min(min_y, keyY(entry.first));
    max_x = std::max(max_x, keyX(entry.first));
    max_y = std::max(max_y, keyY(entry.first));
  }
  const auto width64 = static_cast<std::uint64_t>(
    static_cast<std::int64_t>(max_x) - static_cast<std::int64_t>(min_x) + 1);
  const auto height64 = static_cast<std::uint64_t>(
    static_cast<std::int64_t>(max_y) - static_cast<std::int64_t>(min_y) + 1);
  if (width64 > std::numeric_limits<std::uint32_t>::max() ||
    height64 > std::numeric_limits<std::uint32_t>::max() ||
    (height64 > 0 && width64 > (128ULL * 1024ULL * 1024ULL) / height64))
  {
    return std::nullopt;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.resolution = static_cast<float>(config.occupancy_resolution);
  message.info.width = static_cast<std::uint32_t>(width64);
  message.info.height = static_cast<std::uint32_t>(height64);
  message.info.origin.position.x = min_x * config.occupancy_resolution;
  message.info.origin.position.y = min_y * config.occupancy_resolution;
  message.info.origin.orientation.w = 1.0;
  message.data.assign(static_cast<std::size_t>(width64 * height64), -1);
  for (const auto & [key, value] : cells) {
    const auto x = static_cast<std::size_t>(keyX(key) - min_x);
    const auto y = static_cast<std::size_t>(keyY(key) - min_y);
    message.data[y * message.info.width + x] = value >= 3 ? 100 : 0;
  }
  return message;
}
}  // namespace

struct MapRebuilder::Cache
{
  struct KeyframeBlock
  {
    const Cloud * source = nullptr;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Cloud::Ptr transformed = std::make_shared<Cloud>();
    std::unordered_map<std::int64_t, int> occupancy_cells;
  };

  std::vector<KeyframeBlock> keyframes;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> full_map_voxels;
  std::vector<std::pair<VoxelKey, std::uint64_t>> full_map_order;
  std::uint64_t next_voxel_order = 0;
  std::unordered_map<std::int64_t, int> occupancy_cells;
  bool occupancy_initialized = false;
  std::size_t occupancy_origin = 0;
  std::int32_t occupancy_min_x = 0;
  std::int32_t occupancy_min_y = 0;
  std::int32_t occupancy_max_x = 0;
  std::int32_t occupancy_max_y = 0;
};

MapRebuilder::MapRebuilder(Config config)
: config_(std::move(config)), cache_(std::make_unique<Cache>())
{
}

MapRebuilder::~MapRebuilder() = default;

Cloud::Ptr MapRebuilder::optimizedCloud(const std::vector<Keyframe> & keyframes) const
{
  auto map = std::make_shared<Cloud>();
  std::size_t total_points = 0;
  for (const auto & keyframe : keyframes) {
    const auto source = mapCloud(keyframe);
    total_points += source ? source->size() : 0U;
  }
  map->reserve(total_points);
  for (const auto & keyframe : keyframes) {
    const auto source = mapCloud(keyframe);
    if (!source) {
      continue;
    }
    Cloud transformed;
    pcl::transformPointCloud(
      *source, transformed, keyframe.optimized_pose.matrix().cast<float>());
    *map += transformed;
  }
  return voxelized(map, config_.full_map_voxel_size);
}

Cloud::Ptr MapRebuilder::stableLocalizationCloud(
  const std::vector<Keyframe> & keyframes) const
{
  Cloud transformed;
  std::vector<PointRange> ranges;
  std::size_t total_points = 0;
  for (const auto & keyframe : keyframes) {
    const auto source = mapCloud(keyframe);
    total_points += source ? source->size() : 0U;
  }
  transformed.reserve(total_points);
  ranges.reserve(keyframes.size());
  for (const auto & keyframe : keyframes) {
    const auto begin = transformed.size();
    const auto source = mapCloud(keyframe);
    if (source) {
      Cloud cloud;
      pcl::transformPointCloud(
        *source, cloud, keyframe.optimized_pose.matrix().cast<float>());
      transformed += cloud;
    }
    ranges.push_back(PointRange{begin, transformed.size(), sensorOrigin(keyframe, config_)});
  }
  return stableLocalizationCloudFromTransformed(
    transformed, ranges, config_.localization_voxel_size,
    config_.localization_min_observations);
}

std::optional<nav_msgs::msg::OccupancyGrid> MapRebuilder::occupancy(
  const std::vector<Keyframe> & keyframes, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp) const
{
  Cloud transformed;
  std::vector<PointRange> ranges;
  std::size_t total_points = 0;
  for (const auto & keyframe : keyframes) {
    total_points += keyframe.cloud ? keyframe.cloud->size() : 0U;
  }
  transformed.reserve(total_points);
  ranges.reserve(keyframes.size());
  for (const auto & keyframe : keyframes) {
    const auto begin = transformed.size();
    if (keyframe.cloud) {
      Cloud cloud;
      pcl::transformPointCloud(
        *keyframe.cloud, cloud, keyframe.optimized_pose.matrix().cast<float>());
      transformed += cloud;
    }
    ranges.push_back(PointRange{begin, transformed.size(), keyframe.optimized_pose.translation()});
  }
  return occupancyFromTransformed(transformed, ranges, config_, frame_id, stamp);
}

void MapRebuilder::resetOccupancy(std::size_t origin)
{
  cache_->occupancy_cells.clear();
  cache_->occupancy_initialized = false;
  cache_->occupancy_origin = origin;
  cache_->occupancy_min_x = 0;
  cache_->occupancy_min_y = 0;
  cache_->occupancy_max_x = 0;
  cache_->occupancy_max_y = 0;
}

std::size_t MapRebuilder::occupancyOrigin() const
{
  return cache_->occupancy_origin;
}

MapRebuilder::RebuiltMap MapRebuilder::rebuild(
  const std::vector<Keyframe> & keyframes, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, bool include_localization,
  bool include_occupancy) const
{
  RebuiltMap result;
  auto transformed = std::make_shared<Cloud>();
  std::vector<PointRange> ranges;
  std::size_t total_points = 0;
  for (const auto & keyframe : keyframes) {
    const auto source = mapCloud(keyframe);
    total_points += source ? source->size() : 0U;
  }
  transformed->reserve(total_points);
  ranges.reserve(keyframes.size());

  auto started = Clock::now();
  for (const auto & keyframe : keyframes) {
    const auto begin = transformed->size();
    const auto source = mapCloud(keyframe);
    if (source) {
      Cloud cloud;
      pcl::transformPointCloud(
        *source, cloud, keyframe.optimized_pose.matrix().cast<float>());
      *transformed += cloud;
    }
    ranges.push_back(PointRange{begin, transformed->size(), sensorOrigin(keyframe, config_)});
  }
  result.timings.transform_sec = elapsedSeconds(started);

  started = Clock::now();
  result.raw_full_map = voxelized(transformed, config_.full_map_voxel_size);
  Cloud::Ptr output_source = transformed;
  std::vector<PointRange> output_ranges = ranges;
  if (config_.dynamic_filter_enabled) {
    output_ranges.clear();
    output_source = dynamicFilteredCloud(*transformed, ranges, config_, output_ranges);
  }
  result.full_map = config_.dynamic_filter_enabled ?
    voxelized(output_source, config_.full_map_voxel_size) : result.raw_full_map;
  if (config_.outlier_filter_enabled) {
    result.full_map = radiusOutlierFiltered(result.full_map, config_);
  }
  result.timings.full_map_sec = elapsedSeconds(started);

  if (include_localization) {
    started = Clock::now();
    result.localization_map = stableLocalizationCloudFromTransformed(
      *output_source, output_ranges, config_.localization_voxel_size,
      config_.localization_min_observations);
    if (config_.outlier_filter_enabled) {
      result.localization_map = radiusOutlierFiltered(result.localization_map, config_);
    }
    result.timings.localization_sec = elapsedSeconds(started);
  } else {
    result.localization_map = std::make_shared<Cloud>();
  }

  started = Clock::now();
  if (include_occupancy) {
    result.occupancy = occupancy(keyframes, frame_id, stamp);
  }
  result.timings.occupancy_sec = elapsedSeconds(started);
  return result;
}

MapRebuilder::RebuiltMap MapRebuilder::rebuildIncremental(
  const std::vector<Keyframe> & keyframes, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp, bool include_localization,
  bool materialize_full)
{
  RebuiltMap result;
  result.raw_full_map = std::make_shared<Cloud>();
  result.delta = std::make_shared<Cloud>();
  const double voxel_size = std::max(0.02, config_.full_map_voxel_size);
  const bool cache_was_empty = cache_->full_map_voxels.empty();
  std::unordered_set<VoxelKey, VoxelKeyHash> new_voxels;
  std::unordered_set<std::int64_t> dirty_occupancy_cells;
  const auto remove_block = [this, voxel_size, &dirty_occupancy_cells](
    const Cache::KeyframeBlock & block, std::size_t index)
    {
      for (const auto & point : *block.transformed) {
        const auto key = voxelKey(point, voxel_size);
        const auto iterator = cache_->full_map_voxels.find(key);
        if (iterator == cache_->full_map_voxels.end()) {
          continue;
        }
        iterator->second.sum -= Eigen::Vector4d(
          point.x, point.y, point.z, point.intensity);
        if (iterator->second.count <= 1) {
          cache_->full_map_voxels.erase(iterator);
        } else {
          --iterator->second.count;
        }
      }
      if (index < cache_->occupancy_origin) {
        return;
      }
      for (const auto & [key, value] : block.occupancy_cells) {
        dirty_occupancy_cells.insert(key);
        auto iterator = cache_->occupancy_cells.find(key);
        if (iterator == cache_->occupancy_cells.end()) {
          continue;
        }
        iterator->second -= value;
        if (iterator->second == 0) {
          cache_->occupancy_cells.erase(iterator);
        }
      }
    };
  const auto add_block = [this, voxel_size, &new_voxels, &dirty_occupancy_cells](
    Cache::KeyframeBlock & block, std::size_t index)
    {
      for (const auto & point : *block.transformed) {
        const auto key = voxelKey(point, voxel_size);
        auto [iterator, inserted] = cache_->full_map_voxels.try_emplace(key);
        auto & accumulator = iterator->second;
        if (inserted) {
          accumulator.order = ++cache_->next_voxel_order;
          cache_->full_map_order.emplace_back(key, accumulator.order);
          new_voxels.insert(key);
        }
        accumulator.sum += Eigen::Vector4d(point.x, point.y, point.z, point.intensity);
        ++accumulator.count;
      }
      if (index < cache_->occupancy_origin) {
        return;
      }
      for (const auto & [key, value] : block.occupancy_cells) {
        dirty_occupancy_cells.insert(key);
        cache_->occupancy_cells[key] += value;
      }
    };

  auto started = Clock::now();
  while (cache_->keyframes.size() > keyframes.size()) {
    result.reset_required = true;
    const std::size_t index = cache_->keyframes.size() - 1;
    remove_block(cache_->keyframes.back(), index);
    cache_->keyframes.pop_back();
  }
  cache_->keyframes.resize(keyframes.size());
  for (std::size_t index = 0; index < keyframes.size(); ++index) {
    const auto & keyframe = keyframes[index];
    auto & block = cache_->keyframes[index];
    const bool initialized = block.source != nullptr;
    const bool dirty = !initialized || block.source != keyframe.cloud.get() ||
      !block.pose.matrix().isApprox(keyframe.optimized_pose.matrix(), 1.0e-7);
    if (!dirty) {
      ++result.reused_keyframes;
      continue;
    }
    ++result.transformed_keyframes;
    if (initialized) {
      result.reset_required = true;
      remove_block(block, index);
    }
    block.source = keyframe.cloud.get();
    block.pose = keyframe.optimized_pose;
    block.transformed = std::make_shared<Cloud>();
    block.occupancy_cells.clear();
    if (keyframe.cloud) {
      pcl::transformPointCloud(
        *keyframe.cloud, *block.transformed,
        keyframe.optimized_pose.matrix().cast<float>());
    }
    const std::vector<PointRange> range{
      PointRange{0, block.transformed->size(), keyframe.optimized_pose.translation()}};
    accumulateOccupancy(
      *block.transformed, range, config_.occupancy_resolution,
      config_.obstacle_min_height, config_.obstacle_max_height,
      config_.ground_distance_threshold, config_.fallback_ground_below_sensor,
      config_.occupancy_max_range,
      [&block](std::int32_t x, std::int32_t y, int delta) {
        updateRawCell(block.occupancy_cells, x, y, delta);
      },
      [&block](std::int32_t x0, std::int32_t y0, std::int32_t x1, std::int32_t y1) {
        traceFreeRaw(block.occupancy_cells, x0, y0, x1, y1);
      });
    add_block(block, index);
  }
  result.timings.transform_sec = elapsedSeconds(started);
  result.reset_required = result.reset_required || cache_was_empty;

  started = Clock::now();
  result.full_map = std::make_shared<Cloud>();
  if (!result.reset_required) {
    result.delta->reserve(new_voxels.size());
    for (const auto & key : new_voxels) {
      const auto iterator = cache_->full_map_voxels.find(key);
      if (iterator == cache_->full_map_voxels.end() || iterator->second.count == 0) {
        continue;
      }
      const Eigen::Vector4d mean = iterator->second.sum /
        static_cast<double>(iterator->second.count);
      pcl::PointXYZI point;
      point.x = static_cast<float>(mean.x());
      point.y = static_cast<float>(mean.y());
      point.z = static_cast<float>(mean.z());
      point.intensity = static_cast<float>(mean.w());
      result.delta->push_back(point);
    }
  }
  const bool compact_order = cache_->full_map_order.size() >
    cache_->full_map_voxels.size() * 2U + 1024U;
  std::vector<std::pair<VoxelKey, std::uint64_t>> compacted_order;
  if (compact_order) {
    compacted_order.reserve(cache_->full_map_voxels.size());
  }
  const bool build_full_map = materialize_full || result.reset_required;
  if (build_full_map) {
    result.full_map->reserve(cache_->full_map_voxels.size());
  }
  if (build_full_map || compact_order) {
    for (const auto & [key, order] : cache_->full_map_order) {
      const auto iterator = cache_->full_map_voxels.find(key);
      if (iterator == cache_->full_map_voxels.end() ||
        iterator->second.order != order || iterator->second.count == 0)
      {
        continue;
      }
      if (compact_order) {
        compacted_order.emplace_back(key, order);
      }
      if (build_full_map) {
        const auto & accumulator = iterator->second;
        const Eigen::Vector4d mean = accumulator.sum /
          static_cast<double>(accumulator.count);
        pcl::PointXYZI point;
        point.x = static_cast<float>(mean.x());
        point.y = static_cast<float>(mean.y());
        point.z = static_cast<float>(mean.z());
        point.intensity = static_cast<float>(mean.w());
        result.full_map->push_back(point);
      }
    }
  }
  if (compact_order) {
    cache_->full_map_order.swap(compacted_order);
  }
  result.timings.full_map_sec = elapsedSeconds(started);

  if (include_localization) {
    started = Clock::now();
    result.localization_map = stableLocalizationCloud(keyframes);
    result.timings.localization_sec = elapsedSeconds(started);
  } else {
    result.localization_map = std::make_shared<Cloud>();
  }

  started = Clock::now();
  bool occupancy_requires_reset = materialize_full || result.reset_required ||
    !cache_->occupancy_initialized;
  if (!occupancy_requires_reset) {
    for (const auto key : dirty_occupancy_cells) {
      const auto iterator = cache_->occupancy_cells.find(key);
      if (iterator == cache_->occupancy_cells.end()) {
        continue;
      }
      const auto x = keyX(key);
      const auto y = keyY(key);
      if (x < cache_->occupancy_min_x || x > cache_->occupancy_max_x ||
        y < cache_->occupancy_min_y || y > cache_->occupancy_max_y)
      {
        occupancy_requires_reset = true;
        break;
      }
    }
  }
  if (occupancy_requires_reset) {
    cache_->occupancy_initialized = false;
    result.occupancy = occupancyFromRawCells(
      cache_->occupancy_cells, config_, frame_id, stamp);
    if (result.occupancy) {
      const auto & grid = *result.occupancy;
      cache_->occupancy_min_x = static_cast<std::int32_t>(std::llround(
          grid.info.origin.position.x / config_.occupancy_resolution));
      cache_->occupancy_min_y = static_cast<std::int32_t>(std::llround(
          grid.info.origin.position.y / config_.occupancy_resolution));
      cache_->occupancy_max_x = cache_->occupancy_min_x +
        static_cast<std::int32_t>(grid.info.width) - 1;
      cache_->occupancy_max_y = cache_->occupancy_min_y +
        static_cast<std::int32_t>(grid.info.height) - 1;
      cache_->occupancy_initialized = true;
    }
  } else if (!dirty_occupancy_cells.empty()) {
    std::int32_t min_x = std::numeric_limits<std::int32_t>::max();
    std::int32_t min_y = std::numeric_limits<std::int32_t>::max();
    std::int32_t max_x = std::numeric_limits<std::int32_t>::min();
    std::int32_t max_y = std::numeric_limits<std::int32_t>::min();
    for (const auto key : dirty_occupancy_cells) {
      const auto x = std::clamp(
        keyX(key), cache_->occupancy_min_x, cache_->occupancy_max_x);
      const auto y = std::clamp(
        keyY(key), cache_->occupancy_min_y, cache_->occupancy_max_y);
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
    map_msgs::msg::OccupancyGridUpdate update;
    update.header.stamp = stamp;
    update.header.frame_id = frame_id;
    update.x = static_cast<std::uint32_t>(min_x - cache_->occupancy_min_x);
    update.y = static_cast<std::uint32_t>(min_y - cache_->occupancy_min_y);
    update.width = static_cast<std::uint32_t>(max_x - min_x + 1);
    update.height = static_cast<std::uint32_t>(max_y - min_y + 1);
    update.data.assign(static_cast<std::size_t>(update.width) * update.height, -1);
    for (std::uint32_t y = 0; y < update.height; ++y) {
      for (std::uint32_t x = 0; x < update.width; ++x) {
        const auto iterator = cache_->occupancy_cells.find(
          cellKey(
            min_x + static_cast<std::int32_t>(x), min_y + static_cast<std::int32_t>(y)));
        if (iterator != cache_->occupancy_cells.end()) {
          update.data[static_cast<std::size_t>(y) * update.width + x] =
            iterator->second >= 3 ? 100 : 0;
        }
      }
    }
    result.occupancy_update = std::move(update);
  }
  result.timings.occupancy_sec = elapsedSeconds(started);
  return result;
}

}  // namespace dog_slam
