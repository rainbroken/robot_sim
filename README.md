# 抱抱机器人：Gazebo 与 MID360 仿真

环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。

原 Cartographer 方案已移除。现已接入参考 JUNIOR_DOG_WS 的 FAST-LIO2、
SOLiD/small_gicp/GTSAM 回环建图及独立 Open3D ICP 地图定位。
机器人模型、安装外参、底盘 UKF 和 Nav2 footprint 仍使用 baobao 的参数。
`start_sim.sh` 保持纯仿真；建图和导航使用下面的独立集成入口。

## 编译

在没有加载其他机器人同名功能包的干净终端中：

```bash
cd ~/ros_ws/baobao
bash scripts/build.sh
source scripts/env.sh
```

编译脚本使用 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`，
并限制并行度。默认构建共 10 个 ROS 包（含 `robot_web`）；SLAM 核心合并在 `robot_slam`，接口统一在 `robot_interfaces`，FAST-LIO 等上游依赖保留在 `src/3rdparty`。
Nav2 Humble 源码已拉取到 `src/3rdparty/navigation2`，暂由 `COLCON_IGNORE` 排除默认编译，
保留系统版本运行。固定提交和将来启用源码编译的方法见该目录上一级的 `README.md`。

系统依赖包括 `gazebo_ros_pkgs`、`robot_localization`、`navigation2`、`nav2_bringup`、
`nav2_smac_planner`、`dwb_core`、`dwb_plugins`、`dwb_critics`、OpenMP、`xacro`、`rviz2`。
其他机器可用 `rosdep install --from-paths src --ignore-src -r -y` 检查依赖。
SLAM 还需 Open3D、GTSAM、PCL；编译随附 Livox 驱动需安装 Livox SDK2。
仿真不会启动硬件雷达驱动。导入范围见 `src/3rdparty/README.baobao.md`。
第三方来源和插件修改见 [来源记录](docs/SOURCES.md)。

## 启动仿真

```bash
cd ~/ros_ws/baobao
bash scripts/start_sim.sh

# 办公室场景
bash scripts/start_sim.sh scene:=office

# 关闭行人
bash scripts/start_sim.sh dynamic_pedestrians:=false

# 无图形窗口
bash scripts/start_sim.sh gui:=false rviz:=false
```

每次只运行一个实例。默认 `scene:=house`，也可以用 `world:=/绝对路径/file.world` 指定 ODE 场景。
默认 3 名碰撞行人，最大速度 0.7 m/s、随机种子 42，可分别通过
`pedestrian_count`、`pedestrian_max_speed`、`pedestrian_seed` 修改。
行人是避墙且随机停顿的运动学人形，不是社会力或人体动力学模型。

在另一终端手动控制机器人：

```bash
cd ~/ros_ws/baobao
source scripts/env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Ctrl+C 停止启动命令。原模型预览仍为
`ros2 launch robot_description display.launch.py`，该入口不启动 Gazebo。

## Web 建图与导航

```bash
# 单入口启动 Gazebo 和浏览器控制台，不需要另开建图/导航终端
bash scripts/start_web_sim.sh
# 已有纯仿真时只启动 Web，避免重复启动 Gazebo
bash scripts/start_web.sh
```

打开 `http://127.0.0.1:8080`，依次操作「开始新建图」→ 等待初始化 → 按住方向遥控采图
→ 命名并保存 → 选择地图并「加载地图并启动导航」。切换保留 Gazebo 当前机器人位置，
如定位不符，先用地图上的「初始位姿」工具设置位置和朝向，再发导航目标。
地图上按下并拖动可设定位置与朝向，需点击按钮确认下发；也可以直接输入坐标。
提供实时二维栅格、激光、地图位姿和规划路径，支持取消目标、速度失联停车和后端日志。
地图默认仍保存到本工作空间 `maps/`。使用 Web 时不要同时运行其他建图、导航或速度发布器。
完整参数、远程访问边界和接口见 [robot_web](src/robot_web/README.md)。

## 命令行建图、保存与导航

```bash
# 终端 1：Gazebo + FAST-LIO2 + 回环建图 + RViz，默认 house、3 名行人
bash scripts/start_mapping_sim.sh
# 无图形窗口可追加 gui:=false rviz:=false

# 等到日志出现 Gravity alignment locked 后，在终端 2 手动采图
source scripts/env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 或运行场景专用采图路线；不能与键盘、Nav2 同时发速度
ros2 run robot_sim mapping_route --ros-args \
  --params-file src/robot_sim/config/family_house_mapping_route.yaml

# 采图完成，在另一终端保存到新目录（拒绝覆盖非空目录）
bash scripts/save_map.sh maps/my_house
```

保持静止约 5 秒用于重力初始化；底盘未稳定或有运动时可能等待更久。
固定路线有激光停车保护，行人阻挡持续 15 秒会退出，它不是自动探索或绕障规划器。
保存产生 `full_map.pcd`、`localization.pcd`、`occupancy.yaml/.pgm`。
本机器人未接相机，彩色地图文件使用灰色回退，不代表 RGB 建图。

停止上述建图仿真后，启动独立定位和导航：

```bash
bash scripts/start_navigation_sim.sh map_dir:="$PWD/maps/my_house"
```

默认从地图原点附近初始化，搬到其他位置需要在 RViz 用 **2D Pose Estimate**
给初值；然后用 **2D Goal Pose** 发导航目标。它不是任意位置无初值全局重定位。
只有定位进入 TRACKING 且数据新鲜时才启动 Nav2；定位无效或门控心跳失联，
`command_watchdog` 会阻止底盘运动。地图未覆盖的未知区域默认不允许规划。

也可保持同一个纯仿真，分别运行 `start_slam.sh` 和
`start_navigation.sh map_dir:=...`，切换前先关闭另一后端。
导航门控开启后会保留停车要求；要在同一仿真返回手动建图，先停止导航并执行
`ros2 param set /command_watchdog require_localization false`，或直接重启仿真。

本次走廊测试地图：`maps/junior_house_20261117`。这是局部采图示例，未完成全屋覆盖。
验证范围和原始结果见 [本次 SLAM 验证](docs/JUNIOR_SLAM_VALIDATION.md)。

## 数据链

```text
Gazebo 场景 + 碰撞行人
  ├─ CSV 扫描序列 + ODE 射线 → /livox/lidar (PointCloud2) → RViz
  │                            └─ cloud_to_scan → /scan
  ├─ IMU → /livox/imu ───────────────┐
  └─ 差速轮编码器 → /wheel_odom ──────┴─ UKF → /odom

雷达 + IMU → FAST-LIO2 → 重力对齐/车体转换 → 回环建图或 Open3D 地图定位
建图优化位姿或定位结果 → map→odom；UKF 保持发布 odom→base_footprint
手动/Nav2 速度 /cmd_vel → command_watchdog → /cmd_vel_safe → 差速驱动

odom → base_footprint → base_link → mid360_link → imu_link
            └─ scan_link（水平虚拟扫描坐标系，位于地面、与雷达同 XY）
```

纯仿真入口不发布 `map→odom` 和 `/map`；建图/导航入口会提供它们。UKF 配置在
`src/robot_sim/config/ukf.yaml`，只融合轮速平面线速度与经 TF 转换的 IMU 角速度，
负责局部里程计 `odom→base_footprint`，不构成 SLAM，也不融合 Gazebo 真值。
`robot_state_publisher` 发布机体/传感器 TF；差速插件不重复发布里程计 TF。
`/gazebo/model_states`、`/gazebo/link_states` 仅供诊断。

纯仿真 RViz 使用 `odom`；SLAM RViz 使用 `map`，同时显示地图、模型、原始点云和规划路径。
`cloud_to_scan` 保留为通用投影工具：保留离地 0.08–0.85 m 的点，滤除自身包络，
每个角度取最近距离；未观测方向填 NaN，不当作自由空间。

## 导航模块接口

`robot_navigation` 使用参考 JUNIOR `dog_nav` 的自定义并行 DWB、可配置的速度采样和评分器，
搭配 SmacPlanner2D 与不自动倒车的行为树。机身包络和差速约束仍以 baobao 为准。
当前差速仿真允许低速采样，关闭四足参考工程的 BPX 0.21 死区约束；
轮廓碰撞检查后按中心膨胀代价评分，改善窄间隙通行，并抑制终点转向摆动。
控制输出经过现有 watchdog。站立行人通行测试与移动行人的未通过项见
[导航间隙验证](docs/NAVIGATION_GAP_VALIDATION.md)。
Nav2 框架服务器仍来自系统安装包，自定义控制器源码在本工作空间编译。
详见 `src/robot_navigation/README.md`；通常应使用上面的 SLAM 导航集成入口。
只有自行提供定位时，才直接运行底层入口：

```bash
source scripts/env.sh
ros2 launch robot_navigation navigation.launch.py map:=/绝对路径/有效地图.yaml
```

运行底层入口前需要另行提供与栅格图一致的 `map→odom`；
仿真继续提供 `/odom`、`odom→base_footprint` 和 `/scan`。
底层入口不包含本项目的定位就绪门控，推荐使用 `start_navigation_sim.sh`。
固定路线工具 `mapping_route` 和路线数据是场景辅助工具，同样依赖外部 `map` TF，
不能在当前纯仿真入口下独立运行。

## MID360 与 IMU 的边界

保留原 800 mm 机器人模型。雷达相对 `base_link` 前移 0.110541 m、
上移 0.719640 m、前倾 30°。仿真 `imu_link` 暂与雷达同原点同轴，
这是仿真假设，不是 MID360 内部外参标定值。

插件使用固定版本第三方 CSV 方向序列和 ODE 多射线求交。
每帧跨 20000 个名义样本，10 Hz 仿真帧率；默认 `lidar_downsample:=5`，
每帧测试 4000 条射线，设为 1 则测试 20000 条。未命中/超量程点不发布。
CSV 有 800000 个方向，约 4 个名义扫描秒后循环，不代表厂家精确扫描律。

所有射线逐帧瞬时求交，点时间戳等于该帧仿真时间；没有逐点运动畸变、
网络分包、材质反射率、多回波或天气模型。输出为
`PointCloud2(x,y,z,intensity,timestamp)`，不是 Livox CustomMsg。
FAST-LIO 新增 `lidar_type: 5` 快照输入，保留零点时间偏移，已做仿真冒烟验证。
这不验证真实扫描运动畸变、FAST-LIVO2 或实机传感器适配。

物理步长 1 ms，`/clock` 100 Hz，雷达 10 Hz，IMU 200 Hz，UKF 50 Hz，均按仿真时间。
`sensor_noise_scale:=0` 关闭白噪声，默认 1。
完整射线模式 CPU 成本更高，不保证墙钟实时运行。

## 检查与文件

```bash
source scripts/env.sh
python3 scripts/check_sim.py --duration 10 --output results/live_check.json
# 关闭行人时追加 --pedestrians 0
# 后续接入地图/定位后，可追加 --require-map 检查 /map 和 map TF
```

默认检查点云、IMU、二维扫描、里程计、行人和 `odom→mid360_link`。
它是冒烟检查，不是长期定位或导航验收。

- 集成仿真入口：`src/robot_bringup/launch/bringup.launch.py`
- 仿真与行人：`src/robot_sim/launch/sim.launch.py`、`src/robot_sim/src/random_pedestrians.cpp`
- 传感器：`src/robot_description/xacro/simulation.gazebo.xacro`
- 局部里程计：`src/robot_sim/config/ukf.yaml`
- 点云投影：`src/robot_sim/src/cloud_to_scan.cpp`
- 仿真显示：`src/robot_sim/rviz/simulation.rviz`
- 通用导航参数：`src/robot_navigation/config/nav2.yaml`
- 结果说明：[验证记录](docs/VALIDATION.md)

旧 Cartographer 方案的地图和状态文件已删除。旧诊断文件仅供追查历史问题；
新方案测试结果单独存放在 `results/junior_slam_20261117/`，不要混用。
