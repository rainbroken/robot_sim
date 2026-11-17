# ROS 2 使用说明

本包包含方案 B 的 URDF、Xacro、米制 STL 网格和 ROS 2 Python 启动文件。现有描述模型保持不变：总高约 800 mm、Mid360 前倾 30°，11 个 link，10 个 joint，其中 8 个可动、2 个固定。

## 安装与启动

以下命令在已经安装并 source ROS 2 的 Linux 终端执行。将本目录整个复制到 `~/ros2_ws/src/robot_description`，保持文件夹名和 `package.xml` 的包名一致。

```bash
cd ~/ros2_ws
rosdep install --from-paths src/robot_description --ignore-src -r -y
colcon build --symlink-install --packages-select robot_description --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch robot_description display.launch.py
```

默认展开 Xacro，启动 `robot_state_publisher`、关节滑块窗口和 RViz2。

改用普通 URDF：

```bash
ros2 launch robot_description display.launch.py model:=urdf/baobaowan_B_800mm.urdf
```

无图形界面加载（发布模型 TF，使用零位预览关节状态）：

```bash
ros2 launch robot_description display.launch.py gui:=false rviz:=false
```

使用硬件驱动发布的 `/joint_states` 时，关闭本包的预览状态发布器，避免两个来源同时发布：

```bash
ros2 launch robot_description display.launch.py publish_joint_states:=false
```

连接仿真时，可以加 `use_sim_time:=true`，前提是外部仿真正在发布 `/clock`。普通预览保持默认 `false`。

## URDF / Xacro 文件

- `urdf/baobaowan_B_800mm.urdf`：普通模型描述，直接加载。
- `xacro/baobaowan_B_800mm.urdf.xacro`：可参数化描述，加载前展开。
- `meshes/visual` 和 `meshes/collision`：两种描述共用的网格，单位 m，`scale="1 1 1"`。
- `launch/display.launch.py`：ROS 2 加载入口，支持两种文件。

本文件中的 display 入口负责模型、TF 和预览。工作空间现已新增独立的 Gazebo、MID360、建图和导航入口，见根目录 `README.md`。`gazebo:=true` 时 Xacro 才加载 `simulation.gazebo.xacro`；默认预览不加载仿真插件。舵机硬件接口仍未实现。

## 关节

| 分类 | 数量 | 类型 | 作用 |
|---|---:|---|---|
| 左右驱动轮 | 2 | continuous | 主动驱动 |
| 左右舵机手 | 2 | revolute | 主动驱动，暂定范围 ±0.6 rad |
| 前后万向轮回转轴 | 2 | continuous | 被动回转 |
| 前后万向轮滚动轴 | 2 | continuous | 被动滚动 |
| 地面参考连接、雷达安装 | 2 | fixed | 固定连接 |

因此模型有 **8 个可动关节，4 个预期主动驱动轴**。头部和雷达没有额外运动自由度。

## 验证范围

原始交付检查包括 Xacro 展开、普通 URDF 对照、关节树、网格路径和 Python 语法。当前工作空间已在 ROS 2 Humble / Gazebo Classic 环境完成实际编译与运行检查；本次集成的具体范围、实测数据与限制见根目录 `docs/VALIDATION.md`。

质量、惯量、舵机限位、速度与力矩仍为概念模型暂定值。安装与外参标定边界见上级目录 `README_先看这里.md`。
