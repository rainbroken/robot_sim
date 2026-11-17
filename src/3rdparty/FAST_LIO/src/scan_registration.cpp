#include "scan_registration.hpp"

#include <cmath>
#include <iostream>
#include <omp.h>

void ScanRegistration::compute(
    state_ikfom & state, esekfom::dyn_share_datastruct<double> & ekf_data) const
{
    auto & map = *context_.map;
    auto & feats_down_body = *context_.feats_down_body;
    auto & feats_down_world = *context_.feats_down_world;
    auto & normvec = *context_.normvec;
    auto & laser_cloud_ori = *context_.laser_cloud_ori;
    auto & corr_normvect = *context_.corr_normvect;
    auto & nearest_points = *context_.nearest_points;
    auto & point_selected_surf = context_.point_selected_surf;
    auto & residuals = context_.residuals;
    const int feats_down_size = *context_.feats_down_size;
    auto & effect_feat_num = *context_.effect_feat_num;
    auto & total_residual = *context_.total_residual;
    auto & residual_mean = *context_.residual_mean;
    auto & match_time = *context_.match_time;
    auto & solve_time = *context_.solve_time;

    const double match_start = omp_get_wtime();
    laser_cloud_ori.clear();
    corr_normvect.clear();
    total_residual = 0.0;

#ifdef _OPENMP
    // ARM uses the same independent per-point search as x86. Keep the team
    // local to this loop rather than changing the process-wide OpenMP default.
#pragma omp parallel for num_threads(context_.match_threads) if(context_.match_threads > 1)
#endif
    for (int i = 0; i < feats_down_size; ++i)
    {
        PointType & point_body = feats_down_body.points[i];
        PointType & point_world = feats_down_world.points[i];
        const V3D p_body(point_body.x, point_body.y, point_body.z);
        const V3D p_global =
            state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> point_search_sq_dis(NUM_MATCH_POINTS);
        auto & points_near = nearest_points[i];
        if (ekf_data.converge)
        {
            map.tree().Nearest_Search(
                point_world, NUM_MATCH_POINTS, points_near, point_search_sq_dis);
            point_selected_surf[i] =
                points_near.size() >= NUM_MATCH_POINTS &&
                point_search_sq_dis[NUM_MATCH_POINTS - 1] <= 5;
        }
        if (!point_selected_surf[i])
        {
            continue;
        }

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            const double pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                pabcd(2) * point_world.z + pabcd(3);
            const double weight = 1 - 0.9 * std::abs(pd2) / std::sqrt(p_body.norm());
            if (weight > 0.9)
            {
                point_selected_surf[i] = true;
                normvec.points[i].x = pabcd(0);
                normvec.points[i].y = pabcd(1);
                normvec.points[i].z = pabcd(2);
                normvec.points[i].intensity = pd2;
                residuals[i] = std::abs(pd2);
            }
        }
    }

    effect_feat_num = 0;
    for (int i = 0; i < feats_down_size; ++i)
    {
        if (point_selected_surf[i])
        {
            laser_cloud_ori.points[effect_feat_num] = feats_down_body.points[i];
            corr_normvect.points[effect_feat_num] = normvec.points[i];
            total_residual += residuals[i];
            ++effect_feat_num;
        }
    }
    if (effect_feat_num < 1)
    {
        ekf_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        return;
    }

    residual_mean = total_residual / effect_feat_num;
    match_time += omp_get_wtime() - match_start;
    const double solve_start = omp_get_wtime();
    ekf_data.h_x = MatrixXd::Zero(effect_feat_num, 12);
    ekf_data.h.resize(effect_feat_num);

    for (int i = 0; i < effect_feat_num; ++i)
    {
        const PointType & laser_p = laser_cloud_ori.points[i];
        const V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        const V3D point_this = state.offset_R_L_I * point_this_be + state.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        const PointType & norm_p = corr_normvect.points[i];
        const V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        const V3D C(state.rot.conjugate() * norm_vec);
        const V3D A(point_crossmat * C);
        if (context_.extrinsic_estimation_enabled)
        {
            const V3D B(point_be_crossmat * state.offset_R_L_I.conjugate() * C);
            ekf_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekf_data.h_x.block<1, 12>(i, 0) <<
                norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekf_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start;
}
