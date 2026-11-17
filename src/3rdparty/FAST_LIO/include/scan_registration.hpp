#pragma once

#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>

#include "common_lib.h"
#include "incremental_map.hpp"

class ScanRegistration
{
public:
    struct Context
    {
        IncrementalMap * map = nullptr;
        PointCloudXYZI * feats_down_body = nullptr;
        PointCloudXYZI * feats_down_world = nullptr;
        PointCloudXYZI * normvec = nullptr;
        PointCloudXYZI * laser_cloud_ori = nullptr;
        PointCloudXYZI * corr_normvect = nullptr;
        std::vector<PointVector> * nearest_points = nullptr;
        bool * point_selected_surf = nullptr;
        float * residuals = nullptr;
        int * feats_down_size = nullptr;
        int * effect_feat_num = nullptr;
        double * total_residual = nullptr;
        double * residual_mean = nullptr;
        double * match_time = nullptr;
        double * solve_time = nullptr;
        bool extrinsic_estimation_enabled = true;
        int match_threads = 1;
    };

    void configure(const Context & context) { context_ = context; }
    void compute(
        state_ikfom & state, esekfom::dyn_share_datastruct<double> & ekf_data) const;

private:
    Context context_;
};
