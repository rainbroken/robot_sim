# 2026-11-17 窄间隙与终点导航迭代记录

本轮已修改默认代码/参数并编译、测试。已通过的目标是合成场景中的站立行人绕行、窄走廊通行及终点收敛。
往返横穿行人仍未通过碰撞验收，不能把本轮结果解释为人群导航安全验收。

## 验证层次

1. `test_navigation_scenarios.cpp`：实际 DWB、插件、参数、轮廓及膨胀公式，给定路径，理想差速运动闭环；不发布底盘命令。
2. `scripts/test_navigation_synthetic.py`：私有 ROS domain 中运行系统安装的 SmacPlanner2D、controller_server、代价地图、单点行为树及恢复动作；输入合成 LaserScan、栅格地图、理想里程计/TF，用加减速受限的差速模型响应命令。
3. 未运行 Gazebo 物理、FAST-LIO/ICP 定位或实车。未验证雷达遮挡、定位误差、滑移、真实制动距离和不同人形外廓。多点行为树仅检查了相同的重规划修改，未做多点整链路验收。

测试人形几何为半径 0.20 m 的圆，站在走廊中心。侧隙指人形圆边界到单侧墙面的净宽；65 cm 侧隙对应 1.70 m 宽走廊，75 cm 对应 1.90 m。
机器人外廓保持 0.48 × 0.50 m，padding 保持每侧 0.02 m。报告同时记录真实外廓和 padded 外廓间距；padding 没有缩小。

## 问题与默认版本修改

- **速度采样**：保留参考 BPX 插件兼容性，增加 `bpx_enforce_startup_threshold`，默认插件行为仍为 true；本差速仿真 YAML 显式设 false，允许低速独立直行、旋转和停车。当前模型没有已测得的 BPX 固件死区。最大速度仍是 0.30 m/s、0.60 rad/s，仍不采样侧移或倒车。
- **窄路代价**：`CachedObstacleFootprint` 先执行原生轮廓边界碰撞检查，再检查中心栅格，按整条轨迹中心膨胀代价的均值排名。原来两侧轮廓落在 253 饱和区时，评分区分不出居中与贴墙。`score_center: true`、scale 0.20；lethal/unknown 检查保留，新增中心检查可拒绝部分完全位于轮廓内部的障碍。
- **栅格误差**：仅局部地图改为 0.025 m。0.05 m 时，完整链路 70 cm 走廊会出现墙边栅格碰撞/停滞；单纯增加前进权重的 24/40/60 对照没有解决。0.025 m 后通过。全局静态地图仍使用其自身分辨率。
- **绕行自由度**：强制原地对路径的进入角从 0.60 改到 1.20 rad，允许小偏差使用碰撞检查过的弧线；线速度/角速度采样为 16/25，仿真时长 1.7 s、线性步长 0.025 m。膨胀半径仍为 0.5 m，衰减系数 3→5；Smac 代价权重 5.5→3。
- **重规划**：两棵行为树从 1 Hz 改为 2 Hz。控制失败宽限从 0.5 s 改为 1.2 s，让短暂无轨迹有机会等到更新的路径。15 s 无进展检查、碰撞检查及恢复动作均保留。
- **终点**：位置窗口匹配为 0.15 m；在 0.30 m 内软惩罚角速度和角速度变化；角速度下限改为 0.05 rad/s。终点角度评分前瞻为 0.6 s，碰撞仍检查完整 1.7 s。插值处理角度跨 ±π 和轨迹末尾缺失重复时间戳的情况。
- **位置精度取舍**：goal checker 改为 stateful，第一次进入 XY 容差后集中完成朝向。容差仍是 0.15 m/0.15 rad，但检查针对规划路径终点，并有栅格量化和制动偏移。本轮整链路最终位置相对原始请求约有 0.17 m 误差；不代表厘米级停靠能力。

2.5 cm 局部栅格的单元数是 5 cm 的 4 倍，采样也更密，需要额外 CPU。最终三项通行整链路记录中未出现 `Slow DWB` 或 `Control loop missed`，这不代表其他机器上的实时性能保证。

## 对照与最终结果

### 给定路径的控制器闭环

旧参数使用 `before/nav2.yaml`，重规划模拟频率为 1 Hz；最终参数为默认 YAML、2 Hz。基线程序中的新增功能均保持兼容默认值。

| 场景 | 旧参数 | 最终参数 |
|---|---|---|
| 75 cm 站立行人侧隙 | 65 s 未到达 | 15.0 s 到达，控制失败 0 次 |
| 70 cm 直走廊 | 10.53 s 到达，末段转向换向 3 次 | 10.73 s 到达，换向 0 次 |
| 距目标 0.40 m、最终朝向 1.57 rad | 4.93 s，换向 4 次 | 4.73 s，换向 0 次 |
| 封死走廊 | 不通过 | 65 s 内不通过、无碰撞 |

另有镜像绕行、接近 π 的终点朝向、小角度收敛、大初始偏航、轮廓/中心碰撞及缓存一致性回归。
换向统计为距目标 0.30 m 内、角速度指令绝对值大于 0.04 rad/s 时的符号变化，不等价于全部机体摆动次数。

### 真实 Nav2 服务器与行为树的合成输入闭环

所有时长均为测试模拟时间，不是硬件执行时间；起点 (-1.8, 0)，终点 (1.8, 0)。每项是单次可复现运行，尚非多随机种子统计。

| 场景 | 到达 | 时间 | 恢复次数 | 最小真实外廓间距 | 最小 padded 间距 |
|---|---|---:|---:|---:|---:|
| 站立行人，侧隙 65 cm | 是 | 14.46 s | 0 | 3.26 cm | 1.26 cm |
| 站立行人，侧隙 75 cm，终点转到 1.57 rad | 是 | 15.46 s | 0 | 4.41 cm | 2.41 cm |
| 70 cm 整段直走廊，无行人 | 是 | 13.00 s | 0 | 8.80 cm | 6.80 cm |
| 人以最高约 0.25 m/s 往返横穿 | **否** | 5.82 s 时中止 | 0 | **发生约 0.67 mm 几何穿入** | 余量被耗尽 |

横穿测试不是成功结果：没有进入恢复并不能说明安全。当前 DWB 把每次代价地图当作静态快照，不能可靠处理行人突然折返；默认版本仍有该失败项。
本轮曾试验基于紧凑障碍簇的运动预测、圆拟合及让行停车，仍存在碰撞或恢复问题，已从默认源码、插件导出和 CMake 中撤回。
实验快照保留在 `data/navigation_gap_20261117/experimental_prediction/`，不能直接用当前构建加载；应在隔离副本中恢复对应 CMake/插件/源码后研究，不能直接部署。

## 编译、测试与原始证据

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=1 \
  colcon test --packages-select robot_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/robot_navigation --verbose

# 隔离域整链路，拒绝已被占用的域，不连接硬件图
python3 scripts/test_navigation_synthetic.py --wall .85 --output /tmp/nav_gap65.json
python3 scripts/test_navigation_synthetic.py --goal-yaw 1.57 --output /tmp/nav_gap75.json
python3 scripts/test_navigation_synthetic.py --person none --wall .35 --output /tmp/nav_corridor70.json
# 已知失败回归：预期非零退出码，不能当作通过项
python3 scripts/test_navigation_synthetic.py --person crossing --output /tmp/nav_crossing.json
```

4 个 GTest 可执行文件共 **43 个 GTest 用例全部通过**。`colcon test-result` 的 47 项汇总包含额外 4 个 CTest 包装记录。

原始证据目录 `data/navigation_gap_20261117/`：

- `before/nav2.yaml`、`before/stable_rotate_to_goal_critic.cpp`：本轮开始时的备份。
- `baseline.log/.xml`：旧参数控制器对照，2 项预期失败。
- `build_final.log`、`tests_final.log`、`test_summary.txt`、`final_test_results/`：最终构建及回归。
- `final_res025_standing65.json`、`final_res025_standing75.json`：最终默认配置站立行人整链路；同名 `.params.yaml` 保存实际输入，`.launch.log` 保存服务器日志。
- `corridor_res025.json`：70 cm 整段走廊成功记录；其 `.params.yaml` 与最终默认 YAML 的解析值一致。
- `final_res025_crossing.json`：最终默认配置移动行人失败记录。
- 其他 `iteration*`、`sweep*`、`mean*`、`progress*`、`full_stack*` 是中间实验，不能混为默认版本的验收结果。

重新启动导航后才会加载新的控制器库/参数。仿真配置已经安装；没有启动、移动或重新配置实车。
