/**
 * @file mid360_points_plugin.cpp
 * @brief Mid-360 激光雷达点云仿真插件实现
 *
 * 基于第三方方向序列的 Mid-360 风格瞬时射线仿真
 * 发布标准 ROS2 PointCloud2 消息
 */

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>

#include "mid360_simulation/mid360_points_plugin.h"
#include "mid360_simulation/csv_reader.hpp"
#include "mid360_simulation/mid360_ode_multiray_shape.h"

namespace gazebo
{

// 注册 Gazebo 传感器插件
GZ_REGISTER_SENSOR_PLUGIN(Mid360PointsPlugin)

//==============================================================================
// 构造函数和析构函数
//==============================================================================

Mid360PointsPlugin::Mid360PointsPlugin() {}

Mid360PointsPlugin::~Mid360PointsPlugin() { worldUpdateConnection_.reset(); }

//==============================================================================
// 辅助函数: 将 CSV 数据转换为旋转信息
//==============================================================================

/**
 * @brief 将 CSV 数据转换为扫描旋转信息
 * @param datas CSV 原始数据 [sample_index, azimuth_deg, zenith_deg]
 * @param infos 输出的旋转信息列表
 */
static void convertDataToRotateInfo(
    const std::vector<std::vector<double>>& datas,
    std::vector<RotateInfo>& infos)
{
    infos.reserve(datas.size());
    constexpr double deg_to_rad = M_PI / 180.0;

    for (const auto& data : datas)
    {
        if (data.size() == 3)
        {
            RotateInfo info;
            info.sample_index = data[0];
            info.azimuth = data[1] * deg_to_rad;
            // 转换为标准右手坐标系角度
            info.zenith = data[2] * deg_to_rad - M_PI_2;
            infos.push_back(info);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Mid360Plugin"),
                         "CSV 数据格式错误: 期望 3 列，实际 %zu 列", data.size());
        }
    }
}

//==============================================================================
// 插件加载
//==============================================================================

void Mid360PointsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf)
{
    // 获取 ROS2 节点
    rosNode_ = gazebo_ros::Node::Get(sdf);

    // 读取扫描模式 CSV 文件
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "加载扫描模式文件: %s", file_name.c_str());

    if (!mid360_simulation::CsvReader::ReadCsvFile(file_name, datas))
    {
        RCLCPP_ERROR(rclcpp::get_logger("Mid360Plugin"),
                     "无法读取 CSV 文件: %s", file_name.c_str());
        return;
    }

    // 保存 SDF 配置
    sdfPtr_ = sdf;
    auto rayElem = sdfPtr_->GetElement("ray");
    auto rangeElem = rayElem->GetElement("range");

    // 初始化传感器
    raySensor_ = _parent;
    auto curr_scan_topic = sdf->Get<std::string>("topic");
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "ROS 话题: %s", curr_scan_topic.c_str());

    // 获取坐标系名称
    childName_ = raySensor_->Name();
    parentName_ = raySensor_->ParentName();
    size_t delimiter_pos = parentName_.find("::");
    parentName_ = parentName_.substr(delimiter_pos + 2);

    // 初始化 Gazebo 传输节点
    gazeboNode_ = transport::NodePtr(new transport::Node());
    gazeboNode_->Init(raySensor_->WorldName());

    // 创建 ROS2 PointCloud2 发布器
    cloudPub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
        curr_scan_topic, 10);

    // 创建 Gazebo 内部扫描消息发布器
    scanPub_ = gazeboNode_->Advertise<msgs::LaserScanStamped>(
        curr_scan_topic + "laserscan", 50);

    // 转换扫描模式数据
    scanInfos_.clear();
    convertDataToRotateInfo(datas, scanInfos_);
    if (scanInfos_.empty()) {
        RCLCPP_ERROR(rosNode_->get_logger(), "Empty scan pattern; refusing to start");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "扫描点数: %zu", scanInfos_.size());
    maxPointSize_ = scanInfos_.size();

    // Do not connect RayPlugin's sensor-thread callback. These custom rays
    // must see complete physics steps, including newly spawned model bodies.
    world = physics::get_world(_parent->WorldName());
    laserMsg_.mutable_scan()->set_frame(_parent->ParentName());

    // 获取父实体
    parentEntity_ = this->world->EntityByName(_parent->ParentName());

    // 创建激光碰撞体
    auto physics = world->Physics();
    boost::recursive_mutex::scoped_lock physics_lock(*physics->GetPhysicsUpdateMutex());
    laserCollision_ = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision_->SetName("mid360_ray_collision");
    laserCollision_->SetRelativePose(_parent->Pose());
    laserCollision_->SetInitialRelativePose(_parent->Pose());

    // 创建多射线形状
    rayShape_.reset(new physics::Mid360OdeMultiRayShape(laserCollision_));
    laserCollision_->SetShape(rayShape_);

    // 读取采样参数
    samplesStep_ = sdfPtr_->Get<int>("samples");
    downSample_ = sdfPtr_->Get<int>("downsample");
    if (downSample_ < 1) downSample_ = 1;
    if (samplesStep_ <= 0 || samplesStep_ > 200000) {
        throw std::runtime_error("samples must be in 1..200000");
    }

    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"),
                "采样数: %ld, 降采样: %ld", samplesStep_, downSample_);

    // 初始化射线形状
    rayShape_->RayShapes().reserve(samplesStep_ / downSample_);
    rayShape_->Load(sdfPtr_);
    rayShape_->Init();

    // 读取测距范围
    minDist_ = rangeElem->Get<double>("min");
    maxDist_ = rangeElem->Get<double>("max");
    if (sdfPtr_->HasElement("noise_stddev"))
        noiseStddev_ = sdfPtr_->Get<double>("noise_stddev");
    if (!std::isfinite(noiseStddev_) || noiseStddev_ < 0.0)
        throw std::runtime_error("noise_stddev must be finite and non-negative");

    // 创建初始射线
    auto offset = laserCollision_->RelativePose();
    ignition::math::Vector3d start_point, end_point;

    for (int j = 0; j < samplesStep_; j += downSample_)
    {
        int index = j % maxPointSize_;
        auto& rotate_info = scanInfos_[index];

        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

        start_point = minDist_ * axis + offset.Pos();
        end_point = maxDist_ * axis + offset.Pos();
        rayShape_->AddRay(start_point, end_point);
    }

    if (_parent->UpdateRate() > 0.0) scanPeriod_ = 1.0 / _parent->UpdateRate();
    lastScanTime_ = world->SimTime();
    worldUpdateConnection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&Mid360PointsPlugin::OnNewLaserScans, this));
    RCLCPP_INFO(rclcpp::get_logger("Mid360Plugin"), "Mid-360 仿真插件加载完成");
}

//==============================================================================
// 新扫描数据回调
//==============================================================================

void Mid360PointsPlugin::OnNewLaserScans()
{
    if (!rayShape_) return;
    const auto now = world->SimTime();
    if (now < lastScanTime_) lastScanTime_ = now;
    if ((now - lastScanTime_).Double() + 1e-9 < scanPeriod_) return;
    lastScanTime_ = now;
    boost::recursive_mutex::scoped_lock physics_lock(*world->Physics()->GetPhysicsUpdateMutex());

    // 初始化射线扫描点对
    std::vector<std::pair<int, RotateInfo>> points_pair;
    InitializeRays(points_pair, rayShape_);
    rayShape_->Update();

    // 设置激光扫描消息时间戳
    msgs::Set(laserMsg_.mutable_time(), world->SimTime());
    msgs::LaserScan* scan = laserMsg_.mutable_scan();
    InitializeScan(scan);

    // 获取仿真时间 (纳秒)
    double sim_time_sec = world->SimTime().Double();

    // 创建 PointCloud2 消息 (PointXYZI + timestamp 格式)
    sensor_msgs::msg::PointCloud2 cloud2;
    cloud2.header.stamp.sec = static_cast<int32_t>(sim_time_sec);
    cloud2.header.stamp.nanosec = static_cast<uint32_t>((sim_time_sec - cloud2.header.stamp.sec) * 1e9);
    cloud2.header.frame_id = raySensor_->Name();

    // 设置点云字段 (x, y, z, intensity, timestamp)
    cloud2.fields.resize(5);
    cloud2.fields[0].name = "x";
    cloud2.fields[0].offset = 0;
    cloud2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud2.fields[0].count = 1;
    cloud2.fields[1].name = "y";
    cloud2.fields[1].offset = 4;
    cloud2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud2.fields[1].count = 1;
    cloud2.fields[2].name = "z";
    cloud2.fields[2].offset = 8;
    cloud2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud2.fields[2].count = 1;
    cloud2.fields[3].name = "intensity";
    cloud2.fields[3].offset = 12;
    cloud2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud2.fields[3].count = 1;
    cloud2.fields[4].name = "timestamp";
    cloud2.fields[4].offset = 16;
    cloud2.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT64;
    cloud2.fields[4].count = 1;

    cloud2.point_step = 24;  // 4 floats (16 bytes) + 1 double (8 bytes)
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.is_bigendian = false;

    // 收集有效点
    std::vector<uint8_t> point_data;
    point_data.reserve(points_pair.size() * cloud2.point_step);

    size_t point_count = 0;
    for (auto& pair : points_pair)
    {
        auto range = rayShape_->GetRange(pair.first);

        // 过滤超出范围的点
        if (range >= RangeMax() || range <= RangeMin())
        {
            continue;
        }
        range += noiseStddev_ * normal_(noiseRng_);
        if (range >= RangeMax() || range <= RangeMin()) continue;

        // 计算点云坐标
        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;

        // 写入 x, y, z, intensity
        float x = static_cast<float>(point.X());
        float y = static_cast<float>(point.Y());
        float z = static_cast<float>(point.Z());
        float intensity = 100.0f;
        // All rays are evaluated at this simulation instant. The CSV first
        // column contains sample indices (1, 2, ...), NOT seconds. Invented
        // sweep timestamps would corrupt downstream motion compensation.
        double timestamp = sim_time_sec;

        size_t offset = point_data.size();
        point_data.resize(offset + cloud2.point_step);
        memcpy(&point_data[offset], &x, sizeof(float));
        memcpy(&point_data[offset + 4], &y, sizeof(float));
        memcpy(&point_data[offset + 8], &z, sizeof(float));
        memcpy(&point_data[offset + 12], &intensity, sizeof(float));
        memcpy(&point_data[offset + 16], &timestamp, sizeof(double));
        point_count++;
    }

    // 设置点云数据
    cloud2.width = point_count;
    cloud2.row_step = cloud2.point_step * cloud2.width;
    cloud2.data = std::move(point_data);

    // 发布 Gazebo 内部消息
    if (scanPub_ && scanPub_->HasConnections())
    {
        scanPub_->Publish(laserMsg_);
    }

    // 发布 ROS2 PointCloud2 消息
    cloudPub_->publish(cloud2);
}

//==============================================================================
// 初始化射线
//==============================================================================

void Mid360PointsPlugin::InitializeRays(
    std::vector<std::pair<int, RotateInfo>>& points_pair,
    boost::shared_ptr<physics::Mid360OdeMultiRayShape>& ray_shape)
{
    auto& rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision_->RelativePose();

    int64_t end_index = currStartIndex_ + samplesStep_;
    size_t ray_index = 0;
    auto ray_size = rays.size();
    points_pair.reserve(rays.size());

    for (int64_t k = currStartIndex_; k < end_index; k += downSample_)
    {
        auto index = k % maxPointSize_;
        auto& rotate_info = scanInfos_[index];

        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

        start_point = minDist_ * axis + offset.Pos();
        end_point = maxDist_ * axis + offset.Pos();

        if (ray_index < ray_size)
        {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }

    currStartIndex_ = (currStartIndex_ + samplesStep_) % maxPointSize_;
}

//==============================================================================
// 初始化扫描消息
//==============================================================================

void Mid360PointsPlugin::InitializeScan(msgs::LaserScan*& scan)
{
    msgs::Set(scan->mutable_world_pose(),
              raySensor_->Pose() + parentEntity_->WorldPose());

    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j)
    {
        for (unsigned int i = 0; i < rangeCount; ++i)
        {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

//==============================================================================
// 角度和范围获取方法
//==============================================================================

ignition::math::Angle Mid360PointsPlugin::AngleMin() const
{
    return rayShape_ ? rayShape_->MinAngle() : ignition::math::Angle(-1);
}

ignition::math::Angle Mid360PointsPlugin::AngleMax() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->MaxAngle().Radian())
                     : ignition::math::Angle(-1);
}

double Mid360PointsPlugin::AngleResolution() const
{
    return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1);
}

double Mid360PointsPlugin::RangeMin() const
{
    return minDist_;
}

double Mid360PointsPlugin::RangeMax() const
{
    return maxDist_;
}

double Mid360PointsPlugin::RangeResolution() const
{
    return rayShape_ ? rayShape_->GetResRange() : -1;
}

int Mid360PointsPlugin::RayCount() const
{
    return rayShape_ ? rayShape_->GetSampleCount() : -1;
}

int Mid360PointsPlugin::RangeCount() const
{
    return rayShape_ ? rayShape_->GetSampleCount() * rayShape_->GetScanResolution() : -1;
}

int Mid360PointsPlugin::VerticalRayCount() const
{
    return rayShape_ ? rayShape_->GetVerticalSampleCount() : -1;
}

int Mid360PointsPlugin::VerticalRangeCount() const
{
    return rayShape_ ? rayShape_->GetVerticalSampleCount() * rayShape_->GetVerticalScanResolution() : -1;
}

ignition::math::Angle Mid360PointsPlugin::VerticalAngleMin() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->VerticalMinAngle().Radian())
                     : ignition::math::Angle(-1);
}

ignition::math::Angle Mid360PointsPlugin::VerticalAngleMax() const
{
    return rayShape_ ? ignition::math::Angle(rayShape_->VerticalMaxAngle().Radian())
                     : ignition::math::Angle(-1);
}

double Mid360PointsPlugin::VerticalAngleResolution() const
{
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}

} // namespace gazebo
