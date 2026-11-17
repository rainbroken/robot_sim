# 验证记录与当前范围

日期：2026-11-17。环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2。

## 当前 SLAM 验证

FAST-LIO2 回环建图、Open3D 定位与 Nav2 已接入并完成 Gazebo 走廊测试。
当前结果、启动方式和限制见 [JUNIOR SLAM 接入与验证](JUNIOR_SLAM_VALIDATION.md)。
以下保留的是接入前的移除记录，不能据此判断当前缺少 SLAM。

## 历史：旧方案移除后的范围

原 Cartographer 建图、冻结状态定位、状态保存工具和专用地图已删除。
当时仿真只提供模型、场景、碰撞行人、MID360 风格点云、IMU、二维投影和 UKF 局部里程计。
当时 Nav2 通用配置保留，但没有全局定位提供者。

UKF 配置迁移至 `src/robot_sim/config/ukf.yaml`，
仿真与 `robot_bringup` 不再依赖被移除的 SLAM 包。
默认 RViz 固定坐标系改用 `odom`；模型预览入口保持独立。
`scripts/check_sim.py` 默认检查传感器及 odom TF，`--require-map` 是后续接入定位时的可选检查。

## 清理后的检查方式

```bash
bash scripts/build.sh
bash scripts/start_sim.sh gui:=false rviz:=false
```

另一个已加载 `scripts/env.sh` 的终端：

```bash
python3 scripts/check_sim.py --duration 10 --output results/slam_removal_smoke.json
```

清理后的编译、静态检查和在线检查结果保存在 `results/evidence/slam_removal_*`
及 `results/slam_removal_smoke.json`。具体通过情况以这些结果文件为准。

本次清理后实测：

- 清空旧 `build/`、`install/` 后按 Release + symlink-install 重新编译，5 个包通过。
  `robot_sim` 有 Gazebo CMake 的 FindPkgConfig 开发警告，无编译错误。
- 源码包清单、依赖残留、Python/Shell 语法、YAML 解析与模型检查通过。
- 独立 ROS domain 71 / Gazebo master 11371，无 GUI 运行默认家庭场景，15 秒墙钟在线检查通过。
  仿真时间频率：点云/投影扫描 10 Hz、IMU 200 Hz、UKF 50 Hz；3 名行人均观测到移动。
- 实时节点中没有 Cartographer，`/map` 发布者为 0，没有 `map→odom`，
  `odom→base_footprint→mid360_link` 完整。
- 该次没有打开图形窗口；RViz 配置仅完成 YAML/固定坐标系静态检查。
  测试仿真已关闭；未执行新的建图或导航验收。

## 历史记录的解释

`results/` 中旧 JSON、日志和截图仍保留，便于追查问题，不能作为清理后系统的验收。
旧地图与状态文件已移除，旧日志中的启动/保存路径不能再执行。

移除前曾观察到地图定位米级跳变，详见
`results/localization_motion_before.json`、`results/localization_stationary_diagnostic.json`。
收紧匹配范围后的一轮复测记录为 `results/localization_motion_after.json`，
仅代表单轮结果，不证明长期稳定，也不作为保留旧方案的依据。

传感器插件本身的历史检查包括默认/完整射线、ODE 崩溃诊断和点时间戳检查，
见 `results/full_density_smoke.json`、`results/full_density_before_fix_diagnostic.json`
及 `results/evidence/mid360_before_fix_backtrace.log`。
它使用帧级瞬时求交，没有逐点运动畸变；不是实机 MID360 或 LIO 精度验证。

旧测试使用的独立 ROS domain 和 Gazebo master 只是当时的隔离配置。
新检查只停止自身启动的进程，不按进程名全局清理其他工程。

