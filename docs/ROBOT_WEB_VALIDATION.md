# robot_web 验证记录（2026-11-17，Asia/Shanghai）

## 实现范围

参考 `/home/rain/Desktop/驭灵工作记录/JUNIOR_DOG_WS/src/dog_web` 的地图管理、遥控、
状态展示、初始位姿和单目标导航交互，在本工作空间新增独立 `robot_web`。
原项目依赖 `dog_control_gateway`、`dog_visual_bridge`、机器狗动作与地图管理服务，
这里使用本地 ROS 接口重新实现，未把不适用的机器狗/巡检/相机功能当作可用功能展示。
本次为二维控制台，不是原版三维 UI 的完整移植。

## 构建与运行来源

```bash
bash scripts/build.sh --packages-select robot_web
# 实际执行包含：colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source scripts/env.sh
PYTHONPATH="$PWD/src/robot_web:$PYTHONPATH" python3 -m pytest src/robot_web/test -q
```

`robot_web` 编译完成，12 项检查通过：地图路径/符号链接/部分保存保护、旋转地图坐标及
增量更新、速度范围/owner/失联租约、进程组清理、真实 rclpy 节点构造、暂停时钟识别、
乱序遥控、非有限定位指标的 JSON 序列化、HTTP 同源/静态资源/请求大小，以及外部后端保护。
原始输出为 `data/robot_web_20261117/build.log`、`unit_tests.log`。

Nav2 官方 `humble` 源码已拉取到 `src/3rdparty/navigation2`，版本 1.1.20，固定提交
`3c3db59d6969d8ecee8e68468693d006397f4a0c`。按后续“先不卸载、拉下源码即可”的要求，
没有卸载系统包，也没有编译该源码 overlay；`COLCON_IGNORE` 避免默认构建意外切换版本。
运行时 `nav2_controller`、`nav2_planner`、`nav2_map_server`、`nav2_bt_navigator` 仍来自
`/opt/ros/humble`，自定义插件来自本工作空间的 `robot_navigation`。
可核查 `data/robot_web_20261117/package_sources.json`，后续启用源码编译步骤见
`src/3rdparty/README.md`。这些来源说明不等于源码 Nav2 已完成编译验证。

## 通过的端到端场景

```bash
# 干净终端；不要同时运行其他仿真或后端
bash scripts/start_web_sim.sh gui:=false dynamic_pedestrians:=false
# 等待服务启动；脚本也会有限重试等待仿真就绪
python3 scripts/test_robot_web_sim.py \
  --output data/robot_web_20261117/retest.json
```

配置：自带 house 场景、Gazebo Classic 无 GUI、无行人、默认传感器噪声和
`lidar_downsample:=5`。脚本通过 HTTP API 控制 Web，由 Web 调用 ROS 接口，
没有绕过 Web 直接调用 ROS 保存地图或发送导航目标。短采图路线只适用于自带出生点走廊。

最终有效记录：`data/robot_web_20261117/workflow_verified.json`。

| 检查 | 观察结果 |
| --- | --- |
| 建图启动与初始化 | Web 启动 FAST-LIO2/回环建图，初始化后允许遥控 |
| 遥控实际运动 | 建图起止地图位姿距离约 1.051 m |
| 不再发送遥控心跳 | 停止前实测线速约 0.220 m/s，1.2 s 后约 -0.00052 m/s |
| 保存地图 | `maps/web_test_20261117_193900`，PCD 与栅格文件完整并出现在可选列表 |
| 栅格展示 | 196×234，已知栅格 13,672 个；支持 `/map_updates` |
| 原仿真内切换模式 | 旧建图退出，定位导航启动；显式给定采图结束位姿后 TRACKING |
| 定位后激活 Nav2 | 定位门控和 lifecycle 均就绪后允许目标下发 |
| 前向 0.55 m 目标 | Nav2 `SUCCEEDED`（status 4），恢复次数 0，仿真导航时间约 2 s |
| 终点偏差 | 地图 XY 误差约 0.116 m，满足当前 0.15 m 位置容差 |
| 取消新的独立目标 | 核对新目标坐标后发送停车；获得该目标 `CANCELED`（status 5） |
| 停止后端 | 状态回到 idle，已拥有的后端进程组退出 |

这里的终点偏差来自地图 TF，不是 Gazebo 真值定位误差，也不是实机定位精度。
该检查只覆盖一段局部走廊，不代表完成全屋建图、动态行人避障或长期稳定性验收。
程序里配置的 Web 遥控租期为 350 ms，上表的 1.2 s 是测试取样延迟，未实测精确停车延迟。

## 未稳定通过的场景及推断边界

先前后向 0.55 m 目标曾到达，地图 XY 误差约 0.066 m，但触发 3 次恢复。
该次 `workflow_final.json` 的取消断言读到了上个目标的缓存结果，**不能用作取消成功证据**。
已修正脚本为：核对新目标坐标和 navigating 状态，再要求相同目标返回 canceled；
只有 `workflow_verified.json` 的取消结果用于验收。

再次尝试后向目标时发生大角度转向，定位进入 LOST，目标未在 100 s 墙钟上限内到达，
记录为 `data/robot_web_20261117/rear_goal_repeat.json`。ICP 日志出现
`failed_gates=fitness`，匹配比例约 0.25，低于现有 0.30 门限；在 LOST 状态，
导航门控关闭，底盘里程计线速度维持接近零。定位阈值及现有导航插件源码均未为此放宽或改写。

事实是该次局部小地图和转向组合不稳定；“采图朝向/视角覆盖不足”只是可能原因。
还可能涉及初始位姿误差、走廊结构退化、转向中 FAST-LIO 误差以及现有控制器恢复逻辑。
若要验收任意朝向的目标，应先增加多视角采图覆盖，再独立检查定位质量和控制器行为，
不能把一次前向成功外推为完整导航稳定，也不能将该失败直接归因于 Web 或单一参数。

首次 `workflow.json` 是过早设置定位初值的失败记录，服务明确拒绝等待 FAST-LIO 数据。
页面和最终测试已增加前端重力初始化就绪检查；保留早期记录用于追溯。

## 浏览器与清理

Chromium 实际加载静态页面和实时接口；检查过地图拖动设定 X/Y/朝向。
最终页面无 JavaScript page error，手机 viewport 390 px 时文档宽度 390 px。
见 `data/robot_web_20261117/browser.json` 及同目录桌面/手机截图。
`navigation_desktop.png` 是定位已 TRACKING、Nav2 尚在激活时的页面，
不是到达成功的截图；到达/取消以 action 结果 JSON 为准。

本次测试启动的仿真、Web 和后端均在结束时关闭。没有停止或接管用户已有的其他后端。
使用方式及安全边界见 `src/robot_web/README.md`。
