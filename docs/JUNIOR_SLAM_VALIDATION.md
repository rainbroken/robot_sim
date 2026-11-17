# JUNIOR SLAM 接入与 Gazebo 验证（2026-11-17）

## 实现范围

参考 `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS` 的当前源码，复用：

* FAST-LIO2 前端；
* SOLiD 候选检索、small_gicp 鲁棒配准、GTSAM iSAM2 回环建图；
* Open3D 粗点到点 / 精点到面 ICP 地图定位及质量门限。

没有导入四足驱动、腿部里程计、相机标定、参考机器人的外参/尺寸、CPU 绑核和
supervisor 平台。活跃配置、启动文件及 SLAM 核心在 `src/robot_slam`，接口在 `src/robot_interfaces`，
FAST-LIO 等上游库在 `src/3rdparty`。下述 10 包测试记录属于迁移前版本；目录合并后为 9 包。
本工程提供明确的建图、保存、定位导航模式，未实现参考平台的楼层/地图服务管理界面。

## baobao 适配

目录迁移复查：`robot_slam` 合并核心、`robot_interfaces` 接管五个接口后，
9 包 Release/symlink 编译通过，五个接口类型支持加载通过；在独立 ROS 域 68 中
启动建图及定位入口，确认保存/加载服务使用 `robot_interfaces` 类型。
本轮未重跑完整 Gazebo 导航路线，证据见 `results/package_relocation_20261117/`。

* 雷达相对 base_link `(0.11054145, 0, 0.71964036) m`、pitch `+30°`；
  仿真 IMU 与雷达同轴同原点，FAST-LIO 雷达到 IMU 外参因此为单位变换。
* 坐标转换使用 URDF 的 `base_footprint → imu_link`，没有在算法内重复补偿安装倾角。
* `lidar_type: 5` 直接读取 PointCloud2 快照，逐点偏移全部为零；
  保留原 Livox CustomMsg 模式，未伪造非重复扫描的逐点采样时间。
* Gazebo IMU 保持 m/s²。重力对齐检查使用 9.81，而不是参考工程的 1 g 数值。
* 现有轮速/IMU UKF 独占 `odom → base_footprint`；URDF 独占固定的
  `base_footprint → base_link`；建图桥或定位器独占 `map → odom`。
* Nav2 footprint、轮距、轮径、碰撞保护沿用当前机器人。
* 导航门控等待 TRACKING + pose_valid + 新鲜数据，才启动 Nav2。
  watchdog 在定位门控失联 0.8 秒后阻断底盘命令；定位丢失不会仅凭旧 TF 放行。

地图保存修复：PGM 的未知像素为 205。参考保存阈值 free_thresh=0.25 会让它在重载时
变为 FREE；现改为 0.196。最终重载后保留 35995 个未知格。
首次关键帧立即发布地图，后续保持周期发布，避免静止启动时看不到地图。

## 本机验证结果

环境：ROS 2 Humble / Gazebo Classic，house 场景，3 名行人，seed=42，
sensor_noise_scale=1，lidar_downsample=5。测试使用隔离 ROS_DOMAIN_ID=67。
先无窗口采图，再开启 Gazebo GUI 和 RViz 检查导航。

1. **编译**：使用 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
   的构建脚本，10 个包全部通过。修复了参考 CMake 的 ProcessorCount 拼写，以及
   Livox 驱动依赖上游 build.sh 才传入 DISTRO_ROS 的构建问题。
2. **静止**：初始化后约 12 秒观察，地图平面位姿误差均值约 0.0016 m，最大约 0.0041 m。
3. **移动建图**：约 120 秒观察，平面误差均值约 0.0361 m、最大约 0.1491 m。
   首条固定路线到达第一个路点后，前向扫描 0.26 m，停车超过 15 秒而退出，未完成原路线。
   后续返程路线 `(0,0) → (-2.5,0) → (0,0)` 的三个路点完成。
   建图日志累计 21 次 LOOP ACCEPT；这是后端接受次数，不等于独立人工验证的真实回环数。
4. **保存**：full_map 25746 点，localization 3782 点，栅格 316×236，分辨率 0.05 m；
   无相机输入，colored_keyframe_count=0。地图在 `maps/junior_house_20261117/`。
5. **独立定位**：关闭建图，重新启动 FAST-LIO2 + Open3D，读取保存的 PCD。
   最终约 240 秒观察，平面误差均值约 0.0144 m、最大约 0.0700 m、末值约 0.0027 m。
6. **自主导航**：修正地图阈值并重新启动后，目标 `(-2,0,0)`、`(2,0,0)`、`(0,0,0)`
   全部返回 Nav2 action status=4（SUCCEEDED），墙钟耗时约 39.2 / 50.7 / 30.4 秒。
   过程中有 collision ahead、controller patience 和恢复行为，未关闭碰撞检测。
7. **定位失联停车**：暂停定位器心跳，等待失效后持续发 0.1 m/s 命令，
   31 条 /cmd_vel_safe 样本的线速度和角速度均为零；恢复定位器后门控恢复。
8. **显示**：RViz 可见机器人、三维定位地图、原始雷达点云和二维地图，Global Status OK。
   截图见结果目录。修复集成 launch 中子入口 rviz=false 泄漏到父入口的问题。
9. **最终入口复查**：同时启动 Gazebo GUI、RViz、定位和 Nav2，生命周期激活正常。
   追加的自动短目标被 RViz 新目标抢占，原始 action status=6 保留于
   `final_entrypoint_goals.json`，该轮不作为自动到达验收。
   RViz 发出的约 `(-4.858,0.277)`、`(-6.273,-3.183)` 两个目标随后均打印 Goal succeeded，
   见最终入口日志；它们是交互操作记录，与前述三个自动测试目标分开。

误差来源是只读诊断节点抽样的最新 map→base_footprint 与 Gazebo model_states 的 XY 差值，
没有把真值送入 SLAM、UKF、地图或规划器，也没有做轨迹最优配准。
这些消息没有严格离线时间对齐，因此厘米级数字是本次冒烟观测，不是精度标定结论。

## 原始证据

`results/junior_slam_20261117/`：

* `stationary_ready.json`：初始化后的静止观察；
* `mapping_motion.json`：首段移动采图（包含受阻停止）；
* `final_navigation.json`、`final_nav_goals.json`：修正未知格阈值后的最终定位/导航；
* `localization_stop.json`：定位器失联停车检查；
* `final_mapping_startup.json`：首关键帧 /map 与建图入口检查；
* `final_sensor_smoke.json`：最终传感器、行人运动、地图和 TF 检查；
* `occupancy.png`、`rviz_navigation.png`、`gazebo.png`：栅格和界面显示；
* `logs/`：构建、建图、返程、保存、最终导航与门控日志。

`stationary.json` 是尚未完成重力初始化的早期观察，不能当作初始化成功证据。
`localization_navigation.json` 和 `nav_goals.json` 是阈值修复前的诊断，保留用于追溯，
最终结果以带 final 前缀的文件为准。

## 尚未覆盖

这是一份走廊局部地图，尚未全屋覆盖。未验证长时间运行、其他行人种子、多层/坡道、
任意位置无初值重定位、真实 MID360 运动畸变、实机 IMU/雷达外参和真实底盘安全性。
固定路线工具会受阻退出；Nav2 的短路线成功也不能保证所有动态拥堵都能自行恢复。
真实传感器不能使用快照 lidar_type=5，需要切回对应驱动消息和真实时间模型。

启动和切换命令见根目录 README。
