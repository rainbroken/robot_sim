// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <ctime>
#include <stdexcept>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include "incremental_map.hpp"
#include "mapping_output.hpp"
#include "sensor_buffer.hpp"
#include "scan_registration.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

class LaserMappingNode : public rclcpp::Node
{
private:
/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
bool   traj_save_en = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
string traj_file_path;

double res_mean_last = 0.05, total_residual = 0.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   flg_first_scan = true, flg_EKF_inited = false;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   odom_tf_en = true;
bool    is_first_lidar = true;
string world_frame = "camera_init", body_frame = "body", odom_topic = "/Odometry";

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT = vector<double>(3, 0.0);
vector<double>       extrinR = vector<double>(9, 0.0);
PointCloudXYZI::Ptr featsFromMap{new PointCloudXYZI()};
PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI()};
PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};
PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()};
PointCloudXYZI::Ptr normvec{new PointCloudXYZI(100000, 1)};
PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI(100000, 1)};
PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(100000, 1)};
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

IncrementalMap incremental_map_;
MappingOutput mapping_output_;
ScanRegistration registration_model_;

V3F XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world = V3F(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last = V3D(Zero3d);
V3D Lidar_T_wrt_IMU = V3D(Zero3d);
M3D Lidar_R_wrt_IMU = M3D(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
geometry_msgs::msg::Quaternion geoQuat;

shared_ptr<Preprocess> p_pre{new Preprocess()};
shared_ptr<ImuProcess> p_imu{new ImuProcess()};
SensorBuffer sensor_buffer_{*p_pre};

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                         // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                              // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                              // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void points_cache_collect()
{
    PointVector points_history;
    incremental_map_.tree().acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) {
        kdtree_delete_counter = incremental_map_.tree().Delete_Point_Boxes(cub_needrm);
    }
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    sensor_buffer_.pushStandard(std::move(msg));
}

void livox_pcl_cbk(livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    sensor_buffer_.pushLivox(std::move(msg));
}

void imu_cbk(sensor_msgs::msg::Imu::UniquePtr msg)
{
    ++publish_count;
    sensor_buffer_.pushImu(std::move(msg));
}

bool sync_packages(MeasureGroup & measures)
{
    return sensor_buffer_.syncPackages(measures, lidar_end_time);
}

void h_share_model(state_ikfom & state, esekfom::dyn_share_datastruct<double> & data)
{
    registration_model_.compute(state, data);
}

/*
* @brief : Save the whole trajectory to a txt file (TUM format)
*/
void save_trajectory(const std::string &traj_file) {
    std::string filename(traj_file);
    std::fstream output_fstream;

    output_fstream.open(filename, std::ios_base::out);

    if (!output_fstream.is_open()) {
        std::cerr << "Failed to open " << filename << '\n';
    }

    else {
        output_fstream << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
        for (const auto &p : path.poses) {
            output_fstream << std::setprecision(15) << p.header.stamp.sec + p.header.stamp.nanosec * 1e-9 << " "
                           << p.pose.position.x << " "
                           << p.pose.position.y << " "
                           << p.pose.position.z << " "
                           << p.pose.orientation.x << " "
                           << p.pose.orientation.y << " "
                           << p.pose.orientation.z << " "
                           << p.pose.orientation.w << std::endl;
        }
    }
}

public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        MapUpdatePolicy::Config map_update_config;
        const int match_threads = this->declare_parameter<int>("mapping.match_threads", MP_PROC_NUM);
        if (match_threads < 1 || match_threads > static_cast<int>(
                std::max(1u, std::thread::hardware_concurrency())))
            throw std::invalid_argument("mapping.match_threads must be between 1 and the CPU count");
#ifndef _OPENMP
        if (match_threads != 1)
            throw std::invalid_argument("mapping.match_threads > 1 requires OpenMP");
#endif
        RCLCPP_INFO(get_logger(), "Scan matching threads: %d", match_threads);
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        const bool map_publish_enabled = this->declare_parameter<bool>("publish.map_en", true);
        this->declare_parameter<bool>("publish.tf_en", true);
        this->declare_parameter<string>("publish.world_frame", "camera_init");
        this->declare_parameter<string>("publish.body_frame", "body");
        this->declare_parameter<string>("publish.odom_topic", "/Odometry");
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        const auto performance_path = this->declare_parameter<string>("performance_log_path", "");
        if (!performance_path.empty())
        {
            performance_log_.open(performance_path);
            if (!performance_log_)
                throw std::runtime_error("Cannot open performance_log_path: " + performance_path);
            performance_log_ << "stamp,input_age_s,total_ms,thread_cpu_ms,sync_ms,imu_ms,"
                "processing_lock_ms,segment_ms,downsample_ms,update_ms,match_ms,jacobian_ms,odom_publish_ms,"
                "map_update_ms,cloud_publish_ms,latest_preprocess_ms,points,effective_points,map_points\n";
        }
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("mapping.map_update_motion_gate_enabled", true);
        this->declare_parameter<double>("mapping.map_update_min_translation", 0.03);
        this->declare_parameter<double>("mapping.map_update_min_rotation", 0.02);
        this->declare_parameter<double>("mapping.map_update_max_interval", 0.5);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<bool>("traj_save.traj_save_en", false);
        this->declare_parameter<string>("traj_save.traj_file_path", "");

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<bool>("publish.tf_en", odom_tf_en, true);
        this->get_parameter_or<string>("publish.world_frame", world_frame, "camera_init");
        this->get_parameter_or<string>("publish.body_frame", body_frame, "body");
        this->get_parameter_or<string>("publish.odom_topic", odom_topic, "/Odometry");
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>(
            "mapping.map_update_motion_gate_enabled", map_update_config.enabled, true);
        this->get_parameter_or<double>(
            "mapping.map_update_min_translation", map_update_config.min_translation, 0.03);
        this->get_parameter_or<double>(
            "mapping.map_update_min_rotation", map_update_config.min_rotation, 0.02);
        this->get_parameter_or<double>(
            "mapping.map_update_max_interval", map_update_config.max_interval_sec, 0.5);
        incremental_map_.setUpdatePolicy(map_update_config);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<bool>("traj_save.traj_save_en", traj_save_en, false);
        this->get_parameter_or<string>("traj_save.traj_file_path", traj_file_path, "");
        sensor_buffer_.configure(time_sync_en, time_diff_lidar_to_imu);

        MappingOutput::Config output_config;
        output_config.scan_publish_enabled = scan_pub_en;
        output_config.dense_publish_enabled = dense_pub_en;
        output_config.odom_tf_enabled = odom_tf_en;
        output_config.pcd_save_enabled = pcd_save_en;
        output_config.world_frame = world_frame;
        output_config.body_frame = body_frame;
        output_config.map_file_path = map_file_path;
        mapping_output_.configure(output_config);

        registration_model_.configure({
            &incremental_map_, feats_down_body.get(), feats_down_world.get(), normvec.get(),
            laserCloudOri.get(), corr_normvect.get(), &Nearest_Points,
            point_selected_surf, res_last, &feats_down_size, &effct_feat_num,
            &total_residual, &res_mean_last, &match_time, &solve_time,
            extrinsic_est_en, match_threads});

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = world_frame;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(
            get_f, df_dx, df_dw,
            [this](state_ikfom & state, esekfom::dyn_share_datastruct<double> & data) {
                h_share_model(state, data);
            },
            NUM_MAX_ITERATIONS, epsi);

        /*** optional runtime debug record ***/
        if (runtime_pos_log)
        {
            string pos_log_dir = root_dir + "/Log/pos_log.txt";
            fp = fopen(pos_log_dir.c_str(), "w");
            fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
            fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
            if (fp == nullptr || !fout_pre || !fout_out)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open FAST-LIO runtime log files under %s", root_dir.c_str());
                runtime_pos_log = false;
                if (fp != nullptr)
                {
                    fclose(fp);
                    fp = nullptr;
                }
                fout_pre.close();
                fout_out.close();
            }
        }

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA || p_pre->lidar_type == MID360)
        {
            auto lidar_qos = rclcpp::QoS(
                rclcpp::KeepLast(SensorBuffer::kMaxPendingLidarScans));
            lidar_qos.best_effort().durability_volatile();
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, lidar_qos,
                [this](livox_ros_driver2::msg::CustomMsg::UniquePtr message) {
                    livox_pcl_cbk(std::move(message));
                });
        }
        else
        {
            auto lidar_qos = rclcpp::QoS(
                rclcpp::KeepLast(SensorBuffer::kMaxPendingLidarScans));
            lidar_qos.best_effort().durability_volatile();
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, lidar_qos,
                [this](sensor_msgs::msg::PointCloud2::UniquePtr message) {
                    standard_pcl_cbk(std::move(message));
                });
        }
        auto imu_qos = rclcpp::SensorDataQoS();
        imu_qos.keep_last(SensorBuffer::kMaxPendingImuSamples);
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos,
            [this](sensor_msgs::msg::Imu::UniquePtr message) {
                imu_cbk(std::move(message));
            });
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        if (map_publish_enabled)
        {
            auto map_period_ms = std::chrono::milliseconds(1000);
            map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms,
                std::bind(&LaserMappingNode::map_publish_callback, this));
        }

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        processing_thread_ = std::thread(&LaserMappingNode::processing_loop, this);
        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        stop_processing_.store(true);
        sensor_buffer_.notifyStop();
        if (processing_thread_.joinable()) processing_thread_.join();
        save_outputs();
        if (fout_out.is_open()) fout_out.close();
        if (fout_pre.is_open()) fout_pre.close();
        if (fp != nullptr) fclose(fp);
    }

private:
    //*** main functions ***//
    void processing_loop()
    {
        while (rclcpp::ok() && !stop_processing_.load())
        {
            sensor_buffer_.waitForData(stop_processing_);
            if (stop_processing_.load() || !rclcpp::ok()) break;

            bool processed = false;
            const double lock_start = omp_get_wtime();
            {
                std::lock_guard<std::mutex> processing_lock(processing_mutex_);
                processed = process_next_package(omp_get_wtime() - lock_start);
            }
            if (!processed)
            {
                // A LiDAR scan may be present before its closing IMU sample.
                // Release the processing lock before waiting so ROS callbacks
                // remain able to deliver the closing IMU sample.
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }

    bool process_next_package(double processing_lock_wait)
    {
        const double sync_start = omp_get_wtime();
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return true;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();
            const double input_age = (now() - rclcpp::Time(
                static_cast<int64_t>(lidar_end_time * 1e9), get_clock()->get_clock_type())).seconds();
            timespec cpu_start{};
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start);

            p_imu->Process(Measures, kf, feats_undistort);
            const double imu_end = omp_get_wtime();
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return true;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();
            const double segment_end = omp_get_wtime();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(incremental_map_.tree().Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    incremental_map_.tree().set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    incremental_map_.tree().Build(feats_down_world->points);
                }
                    return true;
            }
            int featsFromMapNum = incremental_map_.tree().validnum();
            kdtree_size_st = incremental_map_.tree().size();
            

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return true;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            if (runtime_pos_log)
            {
                fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;
            }

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(incremental_map_.tree().PCL_Storage);
                incremental_map_.tree().flatten(
                    incremental_map_.tree().Root_Node,
                    incremental_map_.tree().PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = incremental_map_.tree().PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            mapping_output_.publishOdometry(
                pubOdomAftMapped_, tf_broadcaster_, kf, state_point, geoQuat,
                lidar_end_time);
            const double odom_publish_end = omp_get_wtime();

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            incremental_map_.updateIfNeeded(
                *feats_down_body, *feats_down_world, Nearest_Points, state_point,
                filter_size_map_min, lidar_end_time, flg_EKF_inited,
                add_point_size, kdtree_incremental_time);
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)
                mapping_output_.publishPath(
                    pubPath_, path, state_point, geoQuat, lidar_end_time);
            if (scan_pub_en)
                mapping_output_.publishFrameWorld(
                    pubLaserCloudFull_, feats_undistort, feats_down_body,
                    state_point, lidar_end_time, publish_count);
            if (scan_body_pub_en)
                mapping_output_.publishFrameBody(
                    pubLaserCloudFull_body_, feats_undistort, state_point,
                    lidar_end_time, publish_count);
            const double output_end = omp_get_wtime();
            timespec cpu_end{};
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end);
            const double cpu_ms = (cpu_end.tv_sec - cpu_start.tv_sec) * 1000.0 +
                (cpu_end.tv_nsec - cpu_start.tv_nsec) * 1e-6;
            if (output_end - t0 > 0.1 || input_age > 0.4 || processing_lock_wait > 0.1)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Slow scan: input_age=%.3fs total=%.1fms worker_cpu=%.1fms "
                    "sync=%.1fms imu=%.1fms lock_wait=%.1fms segment=%.1fms downsample=%.1fms "
                    "update=%.1fms odom_pub=%.1fms map_update=%.1fms cloud_pub=%.1fms "
                    "points=%d effective=%d",
                    input_age, (output_end - t0) * 1000, cpu_ms, (t0 - sync_start) * 1000,
                    (imu_end - t0) * 1000, processing_lock_wait * 1000,
                    (segment_end - imu_end) * 1000,
                    (t1 - segment_end) * 1000, (t_update_end - t_update_start) * 1000,
                    (odom_publish_end - t_update_end) * 1000, (t5 - t3) * 1000,
                    (output_end - t5) * 1000, feats_down_size, effct_feat_num);
            }
            if (performance_log_.is_open())
            {
                performance_log_ << std::setprecision(16) << lidar_end_time << ',' << input_age << ','
                    << (output_end - t0) * 1000 << ',' << cpu_ms << ','
                    << (t0 - sync_start) * 1000 << ',' << (imu_end - t0) * 1000 << ','
                    << processing_lock_wait * 1000 << ','
                    << (segment_end - imu_end) * 1000 << ',' << (t1 - segment_end) * 1000 << ','
                    << (t_update_end - t_update_start) * 1000 << ',' << match_time * 1000 << ','
                    << solve_time * 1000 << ',' << (odom_publish_end - t_update_end) * 1000 << ','
                    << (t5 - t3) * 1000 << ',' << (output_end - t5) * 1000 << ','
                    << sensor_buffer_.lastPreprocessDuration() * 1000 << ','
                    << feats_down_size << ',' << effct_feat_num << ','
                    << incremental_map_.tree().size() << '\n';
            }

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = incremental_map_.tree().size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot11[time_log_counter] = sensor_buffer_.lastPreprocessDuration();
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
            return true;
        }
        return false;
    }

    void map_publish_callback()
    {
        // Visualization must not block the single-threaded sensor executor.
        std::unique_lock<std::mutex> processing_lock(processing_mutex_, std::try_to_lock);
        if (!processing_lock.owns_lock()) return;
        const double map_start = omp_get_wtime();
        mapping_output_.publishMap(
            pubLaserCloudMap_, feats_undistort, feats_down_body, state_point,
            lidar_end_time);
        const double elapsed_ms = (omp_get_wtime() - map_start) * 1000;
        if (elapsed_ms > 50.0)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Map visualization held the processing lock for %.1fms", elapsed_ms);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!map_pub_timer_)
        {
            res->success = false;
            res->message = "Frontend map accumulation is disabled (publish.map_en=false).";
            return;
        }
        std::unique_lock<std::mutex> processing_lock(processing_mutex_, std::try_to_lock);
        if (!processing_lock.owns_lock())
        {
            res->success = false;
            res->message = "Mapping is busy; retry saving after processing completes.";
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            res->success = mapping_output_.saveMap();
            res->message = res->success ? "Map saved." : "Map save failed.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

    void save_outputs()
    {
        if (traj_save_en)
        {
            save_trajectory(traj_file_path);
            std::cout << "Save FAST-LIO2 trajectory !!" << std::endl;
        }
        if (pcd_save_en)
        {
            cout << "current scan saved to " << map_file_path << endl;
            mapping_output_.saveAccumulatedScans();
        }
        if (runtime_pos_log)
        {
            string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
            FILE * time_log = fopen(log_dir.c_str(), "w");
            if (time_log == nullptr)
            {
                return;
            }
            fprintf(
                time_log,
                "time_stamp, total time, scan point size, incremental time, search time, "
                "delete size, delete time, tree size st, tree size end, add point size, "
                "preprocess time\n");
            for (int i = 0; i < time_log_counter; i++)
            {
                fprintf(
                    time_log,
                    "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                    T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i],
                    int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]),
                    int(s_plot10[i]), s_plot11[i]);
            }
            fclose(time_log);
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    std::thread processing_thread_;
    std::atomic_bool stop_processing_{false};
    std::mutex processing_mutex_;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp = nullptr;
    ofstream fout_pre, fout_out, performance_log_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::spin(node);

    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}
