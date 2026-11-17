#pragma once

#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ikd-Tree/ikd_Tree.h"
#include "map_update_policy.hpp"
#include "use-ikfom.hpp"

class IncrementalMap
{
public:
    using Point = pcl::PointXYZINormal;
    using Cloud = pcl::PointCloud<Point>;
    using PointVector = std::vector<Point, Eigen::aligned_allocator<Point>>;

    KD_TREE<Point> & tree();
    const KD_TREE<Point> & tree() const;
    void setUpdatePolicy(const MapUpdatePolicy::Config & config);
    void resetUpdatePolicy();

    bool updateIfNeeded(
        const Cloud & body_points, Cloud & world_points,
        const std::vector<PointVector> & nearest_points,
        const state_ikfom & state, double voxel_size, double stamp_sec,
        bool ekf_initialized, int & added_points, double & elapsed_sec);

private:
    void update(
        const Cloud & body_points, Cloud & world_points,
        const std::vector<PointVector> & nearest_points,
        const state_ikfom & state, double voxel_size,
        bool ekf_initialized, int & added_points, double & elapsed_sec);

    KD_TREE<Point> tree_;
    MapUpdatePolicy update_policy_;
};
