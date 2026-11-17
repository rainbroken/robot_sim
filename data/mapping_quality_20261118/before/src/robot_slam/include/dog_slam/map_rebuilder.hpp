#pragma once

#include <cstddef>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "dog_slam/loop_closure_types.hpp"

namespace dog_slam
{

class MapRebuilder
{
public:
  struct RebuildTimings
  {
    double transform_sec = 0.0;
    double full_map_sec = 0.0;
    double localization_sec = 0.0;
    double occupancy_sec = 0.0;
  };

  struct RebuiltMap
  {
    Cloud::Ptr raw_full_map;
    Cloud::Ptr full_map;
    Cloud::Ptr delta;
    Cloud::Ptr localization_map;
    std::optional<nav_msgs::msg::OccupancyGrid> occupancy;
    std::optional<map_msgs::msg::OccupancyGridUpdate> occupancy_update;
    RebuildTimings timings;
    std::size_t transformed_keyframes = 0;
    std::size_t reused_keyframes = 0;
    bool reset_required = false;
  };

  struct Config
  {
    double full_map_voxel_size = 0.08;
    double localization_voxel_size = 0.20;
    int localization_min_observations = 2;
    bool dynamic_filter_enabled = false;
    double dynamic_filter_voxel_size = 0.20;
    double dynamic_filter_max_range = 25.0;
    double dynamic_filter_endpoint_margin_base = 0.15;
    double dynamic_filter_endpoint_margin_per_meter = 0.005;
    double dynamic_filter_endpoint_margin_max = 0.35;
    int dynamic_filter_min_hit_keyframes = 2;
    double dynamic_filter_observer_voxel_size = 0.50;
    int dynamic_filter_min_observer_positions = 2;
    double dynamic_filter_prob_hit = 0.70;
    double dynamic_filter_prob_miss = 0.40;
    double dynamic_filter_clamping_min = 0.12;
    double dynamic_filter_clamping_max = 0.97;
    double dynamic_filter_occupancy_threshold = 0.50;
    double lidar_origin_x = 0.25;
    double lidar_origin_y = 0.0;
    double lidar_origin_z = 0.12;
    bool outlier_filter_enabled = false;
    double outlier_filter_radius = 0.25;
    int outlier_filter_min_neighbors = 2;
    double occupancy_resolution = 0.05;
    double obstacle_min_height = 0.08;
    double obstacle_max_height = 1.20;
    double ground_distance_threshold = 0.06;
    double fallback_ground_below_sensor = 0.35;
    double occupancy_max_range = 15.0;
  };

  explicit MapRebuilder(Config config);
  ~MapRebuilder();

  Cloud::Ptr optimizedCloud(const std::vector<Keyframe> & keyframes) const;
  Cloud::Ptr stableLocalizationCloud(const std::vector<Keyframe> & keyframes) const;
  std::optional<nav_msgs::msg::OccupancyGrid> occupancy(
    const std::vector<Keyframe> & keyframes, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;
  RebuiltMap rebuild(
    const std::vector<Keyframe> & keyframes, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp, bool include_localization = true,
    bool include_occupancy = true) const;
  RebuiltMap rebuildIncremental(
    const std::vector<Keyframe> & keyframes, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp, bool include_localization = false,
    bool materialize_full = true);
  void resetOccupancy(std::size_t origin);
  std::size_t occupancyOrigin() const;

private:
  struct Cache;
  Config config_;
  std::unique_ptr<Cache> cache_;
};

}  // namespace dog_slam
