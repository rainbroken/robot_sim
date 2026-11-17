# robot_web

参考 JUNIOR `dog_web` 的地图管理、状态反馈、遥控和导航交互，针对本工作空间重新接线。
提供二维栅格地图、激光、机器人位姿、规划路径、地图保存和 Nav2 单目标导航。
前端为本地静态 HTML/CSS/JS，无需 npm、CDN、rosbridge 或原工作空间的 Web 网关。
不包含机器狗动作、相机、楼层、巡检工作流和原版三维点云控制台。

## 启动

```bash
cd ~/ros_ws/baobao
bash scripts/build.sh
# Gazebo + Web；默认 house 场景、有行人、不启动 RViz
bash scripts/start_web_sim.sh
# 无窗口仿真
bash scripts/start_web_sim.sh gui:=false dynamic_pedestrians:=false
```

浏览器打开 `http://127.0.0.1:8080`。
若已启动 `start_sim.sh`，另开终端运行 `bash scripts/start_web.sh`，避免重复启动 Gazebo。
不要同时启动其他建图或导航后端；Web 会拒绝接管外部后端，需先在原终端关闭。

1. 点击「开始新建图」，保持静止，直到提示建图就绪。
2. 按住方向按钮、W/A/S/D 或方向键采集环境；松开停车。键盘输入框获得焦点时不触发遥控。
3. 填写地图名并保存。保存是后台任务，以「操作结果」成功为准，不以 HTTP 202 为成功。
   保存期间暂停遥控。默认保存在当前工作空间 `maps/`，拒绝覆盖同名目录。
4. 选择保存成功的地图，点击「加载地图并启动导航」。Web 先门控停车，再关闭建图并启动
   `robot_slam/navigation.launch.py`。Gazebo 不重启，因此当前机器人可能不在原点。
5. 若当前地图位姿不符，用「初始位姿」工具在地图上按下并拖动设置机器人位置和朝向，
   再点击「设置初始位姿」。也可输入 X/Y/角度。等待 TRACKING、定位门控和 Nav2 就绪。
6. 使用「导航目标」工具按下并拖动设定目标位置和朝向，再点击「发送导航目标」。
   自由栅格预检不等于路径可达，包络、障碍和路径仍由 Nav2 检查。
7. 「停车 / 取消导航」会发零速度并请求取消当前 Web 目标，观察取消结果。
   「停止后端」停止 Web 自己启动的后端，保留 Gazebo 并保持底盘定位门控。
   无地图定位初值的任意位置全局重定位不在本功能范围。

地图切换时保持仿真场景与所选地图一致。地图不能混用不同场景或不同坐标原点。
复用旧地图时 `localization.pcd`、`full_map.pcd`、栅格 YAML 和图像必须完整。
失败或超时保存的目录保留供检查，换名重试，避免覆盖未确认结果。

## 参数与边界

```bash
bash scripts/start_web_sim.sh scene:=office port:=8090 map_root:=/绝对路径/maps
# 远程访问，仅用于可信局域网
bash scripts/start_web_sim.sh host:=0.0.0.0
```

默认仅绑定回环地址；启用外网卡后没有用户认证或 TLS，不适合直接暴露公网。
端口仅使用一个 HTTP 端口，状态 5 Hz 轮询，遥控 10 Hz 续租；浏览器失焦、隐藏或松键释放。
服务端遥控租期 350 ms，底盘仍经过 `/cmd_vel -> command_watchdog -> /cmd_vel_safe`。
遥控与 Web 活动导航互斥，多浏览器以短期 owner 租约互斥，断联或暂停仿真停止续租。
这些机制不能仲裁外部 teleop/其他客户端的 `/cmd_vel`，使用 Web 时须关闭其他速度源。
这不是实机急停系统。

建图启动：`robot_slam/slam.launch.py`；保存：`/mapping/save_pcd_map`。
定位初值：`/localization/set_initial_pose`；导航：`/navigate_to_pose` action。
订阅 `/map`（Transient Local）和 `/map_updates`，保留栅格原点旋转；
地图位姿来自 TF，激光按扫描时间转换到地图坐标，路径来自 `/plan`。
Nav2 激活与否从 `/bt_navigator/get_state` 查询，导航还要求新鲜 TRACKING 和门控心跳。
进程只管理本服务创建的独立进程组，Ctrl+C 时门控停车并清理后端。
日志位于 `~/.local/share/baobao/web_logs/`，页面可查看末尾 8 KB。

## 检查

```bash
source scripts/env.sh
PYTHONPATH="$PWD/src/robot_web:$PYTHONPATH" python3 -m pytest src/robot_web/test -q
```

Gazebo 的端到端检查脚本与本次实测边界见工作空间 `docs/ROBOT_WEB_VALIDATION.md`。
