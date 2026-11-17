#include "incremental_map.hpp"

#include <algorithm>
#include <cmath>

#include <omp.h>

namespace
{
constexpr int kMatchPointCount = 5;

float squaredDistance(const IncrementalMap::Point & left, const IncrementalMap::Point & right)
{
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

IncrementalMap::Point toWorld(
    const IncrementalMap::Point & point, const state_ikfom & state)
{
    const Eigen::Vector3d body(point.x, point.y, point.z);
    const Eigen::Vector3d world =
        state.rot * (state.offset_R_L_I * body + state.offset_T_L_I) + state.pos;
    IncrementalMap::Point result = point;
    result.x = world.x();
    result.y = world.y();
    result.z = world.z();
    return result;
}
}  // namespace

KD_TREE<IncrementalMap::Point> & IncrementalMap::tree()
{
    return tree_;
}

const KD_TREE<IncrementalMap::Point> & IncrementalMap::tree() const
{
    return tree_;
}

void IncrementalMap::setUpdatePolicy(const MapUpdatePolicy::Config & config)
{
    update_policy_.setConfig(config);
}

void IncrementalMap::resetUpdatePolicy()
{
    update_policy_.reset();
}

bool IncrementalMap::updateIfNeeded(
    const Cloud & body_points, Cloud & world_points,
    const std::vector<PointVector> & nearest_points,
    const state_ikfom & state, double voxel_size, double stamp_sec,
    bool ekf_initialized, int & added_points, double & elapsed_sec)
{
    if (!update_policy_.shouldUpdate(
            state.pos, state.rot.toRotationMatrix(), stamp_sec, ekf_initialized))
    {
        elapsed_sec = 0.0;
        added_points = 0;
        return false;
    }
    update(
        body_points, world_points, nearest_points, state, voxel_size,
        ekf_initialized, added_points, elapsed_sec);
    update_policy_.record(state.pos, state.rot.toRotationMatrix(), stamp_sec);
    return true;
}

void IncrementalMap::update(
    const Cloud & body_points, Cloud & world_points,
    const std::vector<PointVector> & nearest_points,
    const state_ikfom & state, double voxel_size,
    bool ekf_initialized, int & added_points, double & elapsed_sec)
{
    PointVector points_to_add;
    PointVector points_without_downsampling;
    points_to_add.reserve(body_points.size());
    points_without_downsampling.reserve(body_points.size());
    world_points.resize(body_points.size());

    for (std::size_t index = 0; index < body_points.size(); ++index) {
        world_points[index] = toWorld(body_points[index], state);
        const Point & world_point = world_points[index];
        if (index >= nearest_points.size() || nearest_points[index].empty() || !ekf_initialized) {
            points_to_add.push_back(world_point);
            continue;
        }

        const PointVector & nearby = nearest_points[index];
        Point voxel_center;
        voxel_center.x = std::floor(world_point.x / voxel_size) * voxel_size + 0.5 * voxel_size;
        voxel_center.y = std::floor(world_point.y / voxel_size) * voxel_size + 0.5 * voxel_size;
        voxel_center.z = std::floor(world_point.z / voxel_size) * voxel_size + 0.5 * voxel_size;
        const float distance = squaredDistance(world_point, voxel_center);

        if (std::fabs(nearby.front().x - voxel_center.x) > 0.5 * voxel_size &&
            std::fabs(nearby.front().y - voxel_center.y) > 0.5 * voxel_size &&
            std::fabs(nearby.front().z - voxel_center.z) > 0.5 * voxel_size)
        {
            points_without_downsampling.push_back(world_point);
            continue;
        }

        bool need_add = true;
        const std::size_t count = std::min<std::size_t>(kMatchPointCount, nearby.size());
        for (std::size_t nearby_index = 0; nearby_index < count; ++nearby_index) {
            if (squaredDistance(nearby[nearby_index], voxel_center) < distance) {
                need_add = false;
                break;
            }
        }
        if (need_add) {
            points_to_add.push_back(world_point);
        }
    }

    const double start = omp_get_wtime();
    tree_.Add_Points(points_to_add, true);
    tree_.Add_Points(points_without_downsampling, false);
    elapsed_sec = omp_get_wtime() - start;
    added_points = static_cast<int>(
        points_to_add.size() + points_without_downsampling.size());
}
