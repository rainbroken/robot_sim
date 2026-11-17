# 来源和适配记录

查阅日期：2026-11-17。以下是实际查看的源码/官方资料；第三方说明不自动视为已验证事实。

## 本地参考工程

来源：`/home/rain/ros_ws/小车模型交付_R150_H150/small_car_carto`。
原工程未修改，也没有把它的 install 或地图作为当前机器人的运行依赖。

- 复制办公室、家庭场景、家庭场景生成器、碰撞行人、固定采图路线工具和速度看门狗。
- 复制其带有反向编码器速度修复的 Gazebo 差速插件，保留 BSD/Apache 版权头。
- 先前复制的 Cartographer、ROS 集成源码和专用 Lua/Abseil 本地依赖已移除。
  当前仅保留仿真基础设施与不绑定定位算法的 Nav2 配置。
- 未复制另一机器人的 URDF、地图、Nav2 源码 overlay、站点业务和控制器插件。

抱抱机器人特定适配：轮径 0.13 m、轮距 0.35 m；雷达安装沿用原设计；
轮子仿真碰撞用同尺寸圆柱，保留外观和原预览模式碰撞网格。
仿真关节阻尼设为 0.001，解决小惯量万向轮在原 0.05 阻尼下运动时的 ODE 发散；
质量/惯量继续作为原概念模型的估计值，不声称经过硬件辨识。
家庭路线、连通性和碰撞高度检查从原 0.27 m 改为 0.85 m，并改造餐桌前转弯段。

## MID360 扫描插件

实际采用：
https://github.com/inkccc/mid360_simulation

固定版本：`241cfbff81f851e4c15c1c7dba039ca29824b264`，Apache-2.0。
源码、许可证和原始 CSV 放在 `src/3rdparty/mid360_simulation`，原上游宏不参与机器人建模。
该插件致谢的原始 Livox 射线实现：
https://github.com/Livox-SDK/livox_laser_simulation

交叉参考：
https://github.com/LihanChen2004/pb_rm_simulation
该项目展示 ROS 2 Humble/Gazebo Classic 的 MID360、IMU、LIO 和 Nav2 接法。
本次没有引入其全向底盘、FAST-LIO 或假定 Livox CustomMsg 已兼容。

官方规格核对：
https://www.livoxtech.com/mid-360/specs
官方页面给出首回波 200000 points/s、典型帧率 10 Hz；这只作为名义采样推进速率参考。
仿真测距上限设为 30 m，适配室内场景，不把官方条件测距能力等同于仿真性能。

与上游不同之处：

1. 修复 CSV 序号被错误当成秒数的问题。实际表第一列为 1…800000，
   并非每点绝对时间。所有射线瞬时求交，点时间戳统一等于本帧的仿真时间。
2. 添加空序列和采样数检查，扫描索引循环，避免无限增长。
3. 在有效距离上实际添加可调高斯噪声；上游插件仅读配置但没有对自定义射线返回距离加噪。
4. 保留原机器人雷达外观；不加载上游传感器外壳、不采用其内置 IMU 外参猜测。
5. 默认每 5 个名义样本求交一次，降低 ODE 计算量，同时覆盖四组交错样本。
6. CSV 方向属于第三方序列；未证明它等于厂家实际固件中的精确扫描律。
7. 按 Gazebo 原生 `ODEMultiRayShape` 保留机器人 link 的碰撞空间归属，
   只把专用射线碰撞体放进 ray space。创建和更新射线均使用物理互斥锁；
   完成初始化后连接 `WorldUpdateEnd`，按传感器配置的帧率求交，
   避免独立 sensor 线程访问正在初始化/更新的物理碰撞体。
   完整射线测试曾在 ODE `dSpaceCollide2` 中崩溃，调整空间归属及加锁后仍复现，
   改为物理步结束回调后通过复测。保留失败诊断，不将其归因为已证明的单一原因。

## 其他查阅的主来源

- Gazebo Classic `MultiRayShape::GetRange/Update`：
  https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/MultiRayShape.cc
  核对 GetRange 已加最小量程，避免人为再加一次距离偏置。
- Gazebo Classic ODEJoint：
  https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEJoint.cc
  检查关节阻尼和求解机制；最终采用低阻尼，不依赖隐式阻尼与轮速电机竞争。
- Gazebo ROS clock 参数：
  https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/src/gazebo_ros_init.cpp
- 当前本机 `/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py` 与默认参数：
  按已安装版本实际接入导航生命周期和速度平滑器。

Gazebo Classic 是为兼容用户指定的现有工程而保留的技术栈；本交付不代表已迁移到 Gazebo Sim/Harmonic。
# FAST-LIO2 / JUNIOR integration (2026-11-17)

Imported from `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS/src`:
FAST_LIO/FAST_LIO, small_gicp, livox_ros_driver2, a subset of dog_slam C++ sources
and five dog_interfaces definitions. The core now builds as robot_slam, and the
definitions now generate robot_interfaces types. The internal dog_slam C++
namespace is retained. Existing upstream license files are retained.
Active robot parameters are exclusively in baobao's robot_description, robot_sim,
robot_navigation and robot_slam. See `src/3rdparty/README.baobao.md` and
`docs/JUNIOR_SLAM_VALIDATION.md` for import boundaries and local changes.

# JUNIOR dog_nav adaptation (2026-11-17)

The initial-navigation statements above predate this adaptation. Eight custom
controller/generator/critic C++ implementations and their headers, regression
tests and single-goal behavior tree are now imported from JUNIOR_DOG_WS/src/dog_nav
into src/robot_navigation. Internal dog_nav namespaces are retained. The local
shared library is robot_navigation_dwb_plugins. Smac replaces NavFn, and the
custom parallel DWB controller replaces RPP. Both navigator behavior trees avoid
automatic backup. There is no velocity smoother in the active command chain.

Baobao keeps its own footprint, differential kinematics, frame tree, /odom and
scan obstacle pipeline. The reference's stair policies, STVL/cloud filter,
hardware mux, nav_control offset and patrol management are not imported.
Nav2 servers and standard planner/DWB dependencies remain installed Humble
packages, as they also are dependencies of the reference dog_nav integration.
See src/robot_navigation/README.md and data/dog_nav_* for scope and live evidence.

