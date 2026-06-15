# core/navigation — 5e Nav2 自主导航

Gz Harmonic 重建栈第 5e 阶段（最小打通）。在 5c FAST-LIO 里程计 + 5d GICP 先验图定位之上，
打通最小 Nav2：给一个目标位姿，差速小车在工厂先验图里用 **Smac Hybrid-A\***（全局）+ **MPPI**（局部）
自主规划并开到目标。定位靠 GICP 的 `map→camera_init` TF，**无 AMCL**。

## TF 链
```
map →[GICP]→ camera_init →[FAST-LIO]→ body →[本包静态焊接]→ base_footprint →[URDF]→ base_link/...
```
焊接：parent=body→child=base_footprint，单位旋转，z=-0.556（=-(base_height/2+wheel_radius+sensor_z)）。

## 节点职责（详见 spec）
- map_server：发先验 2D 图 `/map`(latched) 给全局 costmap。
- planner_server(+全局 costmap，map 系)：Smac Hybrid-A\* 出全局路径。
- controller_server(+局部 costmap，camera_init 系)：MPPI 出速度 → `/cmd_vel_nav`(Twist)。
- behavior_server：spin/backup/wait 恢复。
- bt_navigator：行为树大脑，编排 planner/controller/behavior。
- lifecycle_manager：autostart 上述五节点。
- twist_stamper：`/cmd_vel_nav`(Twist) → `/cmd_vel`(TwistStamped) → diff_drive。

## 关键设计点
- **双 frame**：全局 costmap=map（含 GICP 校正、会跳变，全局重规划无害）；局部 costmap+behavior=camera_init（FAST-LIO 连续、不跳变，MPPI 高频环要平滑）。
- **障碍源** `/cloud_registered`(sensor_frame=body)，**不是** `/points_raw`（后者经 velodyne URDF 链转歪刷假障碍）。
- **局部 voxel `origin_z=-1.0`**：含传感器原点；否则报 `Sensor origin out of map bounds`、清不掉障碍假卡死。
- **odom_topic `/base_controller/odom`**：diff_drive 真实 twist；FAST-LIO `/Odometry` twist 恒零，MPPI 不能用。
- **换底盘只改 CHASSIS-DEPENDENT 参数**（`nav2_params.yaml` 顶部注释块列清单：转弯半径/运动模型/footprint/限速）。

## 构建机：验证流程
前提：**构建根 `core/`**；5c/5d 已 build；`apt install ros-humble-navigation2 ros-humble-nav2-bringup`（含 smac-planner、mppi-controller）；**先验图 5b 已在 `~/result/GlobalMap.pcd`**。

```bash
# 0) 离线先验图 2D 化(需 open3d)
cd core && colcon build --packages-select robot_navigation && source install/setup.bash
pip install open3d
ros2 run robot_navigation pcd_to_occupancy --pcd ~/result/GlobalMap.pcd \
    --out ~/result/factory_map.yaml --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2

# 1-4) 起现有栈(各一终端，均 cd core && source install/setup.bash)
ros2 launch robot_gz_bringup robot_gz.launch.py
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
ros2 launch gicp_localization localization.launch.py
ros2 run robot_gz_bringup sticky_teleop.py        # 开到已知起点附近

# 锁定 GICP：RViz「2D Pose Estimate」(/initialpose) 设到机器人真实 map 位姿，等 /localization 稳定

# 5) 起导航
ros2 launch robot_navigation navigation.launch.py
#   先验图非默认路径则传 map:=/path/to/factory_map.yaml
```

## 验收判据（PASS → 5e 完成）
1. 五个 nav 生命周期节点全 active（`ros2 lifecycle get /bt_navigator` 等）；启动无报错。
2. TF `map→camera_init→body→base_footprint` 全链通；`tf2_echo map base_footprint` 随车动、**z≈0**（焊接符号对；若≈+1.1 翻 weld_z 符号）、旋转≈单位。
3. frame 校验(global_frame 在内嵌 costmap 子节点上,不在 server 节点本身——查 `/controller_server`/`/planner_server` 会回 "Parameter not set",正常):
   `ros2 param get /behavior_server global_frame`=`camera_init`；
   `ros2 param get /local_costmap/local_costmap global_frame`=`camera_init`；
   `ros2 param get /global_costmap/global_costmap global_frame`=`map`。
4. RViz 给「Nav2 Goal」→ Smac Hybrid-A\* 规划出**平滑可行路径**（无原地转尖角）；`/plan` 有路径。
5. **车实际开到目标**（不反向、不贴墙——走通道中央）；`/cmd_vel_nav` 有 Twist、`/cmd_vel` 有 TwistStamped、底盘响应。
6. 局部 costmap 不刷 `Sensor origin out of map bounds`；障碍正常 mark+clear；MPPI 不报控制超时。

## FAIL 排查
- **车朝目标反向跑/不拐弯** → 焊接旋转非单位（必须单位；勿用 pitch=π）。
- **车贴墙走** → 调大局部 `inflation_radius`、`cost_scaling_factor` 调缓、MPPI `ObstaclesCritic.critical_weight` 提高。
- **MPPI 控制超时/断续 / RTF 砸到个位数**（WSL 软渲 gpu_lidar 吃 CPU,算力紧）→ 降 `batch_size`、`time_steps`、`controller_frequency`(现已 1000/40/15;再紧到 500/10);治本是 WSLg GPU 直通。
- **转弯特别慢 / 弯道像停下来转**（紧弯 vx 被 `vx=wz·r` 钳死）→ 提 `wz_max`(现 1.8,≤底盘 2.0)、`vx_max`(现 1.0,≤底盘 1.5)、`wz_std`(现 0.6);仍嫌弯太碎可调大 `minimum_turning_radius`(0.2→0.4,牺牲窄道机动)。
- **局部 costmap 刷 origin out of bounds / 障碍清不掉** → 确认 voxel `origin_z=-1.0`、源 `/cloud_registered`。
- **behavior 报 odom/帧不存在** → 确认 behavior global_frame=camera_init、odom_topic=/base_controller/odom。
- **改 launch/config 不生效** → 必须重新 `colcon build`（launch 跑 install/ 副本）。
- **ament_python 测试 colcon 不发现** → `python3 -m pytest core/navigation/robot_navigation/test` 兜底。

## 后续路线图（不在 5e）
- **穿点导航**：启用 `waypoint_follower` + `navigate_through_poses`（参数/BT 已内置默认）。
- **动态障碍**：移植旧 `src/sim_obstacles`（关重力防翻倒）+ MPPI 调避让（差速 + MPPI 预测式，目标超越旧栈 DWB 的 stop-and-wait）。
- **手动接管**：加 `twist_mux` 多路复用 teleop + nav（都走 TwistStamped）。
- **STVL / velocity_smoother / collision_monitor**：按需。
