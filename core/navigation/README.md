# core/navigation — 5e Nav2 自主导航

Gz Harmonic 重建栈第 5e 阶段（最小打通）。在 5c FAST-LIO 里程计 + 5d GICP 先验图定位之上，
打通最小 Nav2：给一个目标位姿，差速小车在工厂先验图里用 **Smac Hybrid-A\***（全局）+ **MPPI**（局部）
自主规划并开到目标。定位靠 GICP 的 `map→camera_init` TF，**无 AMCL**。

## TF 链
```
map →[GICP]→ camera_init →[FAST-LIO]→ body →[本包静态焊接]→ base_footprint →[URDF]→ base_link/...
```
焊接：仿真默认 `body→base_footprint=[0,0,-0.556]`；真机由
`system_bringup/config/bringup.yaml:real_geometry` 派生，当前为 `[-0.443,0,-0.905]`、单位旋转。

## 节点职责（详见 spec）
- map_server：发先验 2D 图 `/map`(latched) 给全局 costmap。
- planner_server(+全局 costmap，map 系)：Smac Hybrid-A\* 出全局路径。
- controller_server(+局部 costmap，camera_init 系)：MPPI 出速度 → `/cmd_vel_nav`(Twist)。
- behavior_server：spin/backup/wait 恢复。
- bt_navigator：行为树大脑，编排 planner/controller/behavior。
- lifecycle_manager：autostart 上述五节点。
- twist_stamper：`/cmd_vel_nav`(Twist) → 可配置输出；独立启动默认 `/cmd_vel`，
  完整 bringup 改为 `/cmd_vel_auto`，再经 `cmd_vel_gate` 输出 `/cmd_vel`。

## 关键设计点
- **双 frame**：全局 costmap=map（含 GICP 校正、会跳变，全局重规划无害）；局部 costmap+behavior=camera_init（FAST-LIO 连续、不跳变，MPPI 高频环要平滑）。
- **障碍源** `/cloud_registered`(sensor_frame=body)，**不是** `/points_raw`（后者经 velodyne URDF 链转歪刷假障碍）。
- **全局+局部障碍层统一 STVL**(`spatio_temporal_voxel_layer`)：视锥+时间衰减清除(非 VoxelLayer raytrace)；`combination_method:1`(max)不覆盖 static_layer；为未来运动障碍预留(届时调 `voxel_decay`/`decay_acceleration`)。STVL 的 3D voxel 层结构性规避了旧 VoxelLayer 的 `Sensor origin out of map bounds`(`origin_z:0.0` 即可,无需旧的 `-1.0` hack)。
- **odom_topic `/base_controller/odom`**：diff_drive 真实 twist；FAST-LIO `/Odometry` twist 恒零，MPPI 不能用。
- **真机几何只改一处**：`bringup.yaml:real_geometry` 运行时生成 footprint 并下发 URDF/TF/控制器；规划器运动学和限速等调参仍在 Nav2 profile。
- **人工接管**：Web 只切换 gate 接受的速度源，不取消当前 Nav2 goal；点击
  “恢复自动导航”后 Nav2 继续输出。manual 模式浏览器断连时，0.5 秒源超时停车。

## 构建机：验证流程
前提：**构建根 `core/`**；5c/5d 已 build；`apt install ros-humble-navigation2 ros-humble-nav2-bringup`（含 smac-planner、mppi-controller）；**先验图 5b 已在 `~/result/GlobalMap.pcd`**。

```bash
# 0) 离线先验图 2D 化(需 open3d)
cd core && colcon build --packages-select robot_navigation && source install/setup.bash
pip install open3d
ros2 run robot_navigation pcd_to_occupancy --pcd ~/result/GlobalMap.pcd \
    --out ~/result/factory_map.yaml --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2

# 1) config/bringup.yaml 设 platform: sim、mode: navigation，然后起完整栈
ros2 launch system_bringup bringup.launch.py

# 2) 锁定 GICP：RViz「2D Pose Estimate」(/initialpose) 设到机器人真实 map 位姿，等 /localization 稳定
```

## 真机配置与验收边界

真机车体、车轮和雷达安装位置已按人工测量写入 `bringup.yaml:real_geometry`；完整 bringup
会生成实际 footprint、轮参和 TF。雷达坐标原点目前按外壳中线估计，垂直 FOV、STVL
障碍高度带及雷达/内置 IMU 精确外参仍待厂家资料和现场验证，因此这些仍不是最终标定值。

真机**静态**验收使用 `platform: real`、`mode: navigation` 后启动完整 bringup，并检查：

```bash
ros2 launch system_bringup bringup.launch.py
timeout 10 ros2 topic hz /Odometry
timeout 10 ros2 topic hz /cloud_registered
timeout 10 ros2 topic hz /localization
timeout 10 ros2 topic hz /base_controller/odom
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

静态判据：Vanjee → FAST-LIO → GICP/base odom → Nav2 按序放行；`map → camera_init → body
→ base_footprint → base_link → velodyne/imu_link` 连通；五个 Nav2 lifecycle 节点均为
`active`；先验点云、地图和 costmap 能加载。小车保持静止，不发送 Nav2 goal。这些
FAST-LIO/GICP/Nav2 的新一体化真机运行检查目前仍为 **待完成**。

**动态行驶验收另行进行，不能从静态检查推断 PASS。** 它包括最终外参与 IMU 正负轴验证、
FAST-LIO 去畸变和行驶轨迹、GICP fitness/错误初值恢复与 `/initialpose` 重定位、Nav2
路径跟踪、障碍 mark/clear 和控制输出实际驱动车轮。仅在小车具备安全运动条件后执行。

手机访问 `http://<机器人或仿真主机IP>:8080`

- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

完整 bringup 中只有 `cmd_vel_gate` 发布 `/cmd_vel`，Web 和 Nav2 分别只发布
`/cmd_vel_manual`、`/cmd_vel_auto`。Web 不会失能硬件，不能替代物理急停。

## 验收判据（PASS → 5e 完成）
1. 五个 nav 生命周期节点全 active（`ros2 lifecycle get /bt_navigator` 等）；启动无报错。
2. TF `map→camera_init→body→base_footprint` 全链通；`tf2_echo body base_footprint`：仿真约 `[0,0,-0.556]`，当前真机约 `[-0.443,0,-0.905]`，旋转均为单位阵。
3. frame 校验(global_frame 在内嵌 costmap 子节点上,不在 server 节点本身——查 `/controller_server`/`/planner_server` 会回 "Parameter not set",正常):
   `ros2 param get /behavior_server global_frame`=`camera_init`；
   `ros2 param get /local_costmap/local_costmap global_frame`=`camera_init`；
   `ros2 param get /global_costmap/global_costmap global_frame`=`map`。
4. RViz 给「Nav2 Goal」→ Smac Hybrid-A\* 规划出**平滑可行路径**（无原地转尖角）；`/plan` 有路径。
5. **车实际开到目标**（不反向、不贴墙——走通道中央）；`/cmd_vel_nav` 有 Twist、
   `/cmd_vel_auto` 和 gate 输出 `/cmd_vel` 有 TwistStamped、底盘响应。
6. 局部 costmap 不刷 `Sensor origin out of map bounds`；障碍正常 mark+clear；MPPI 不报控制超时。

## FAIL 排查
- **车朝目标反向跑/不拐弯** → 焊接旋转非单位（必须单位；勿用 pitch=π）。
- **车贴墙走** → 调大局部 `inflation_radius`、`cost_scaling_factor` 调缓、MPPI `ObstaclesCritic.critical_weight` 提高。
- **MPPI 控制超时/断续 / RTF 砸到个位数**（WSL 软渲 gpu_lidar 吃 CPU,算力紧）→ 降 `batch_size`、`time_steps`、`controller_frequency`(现 1000/30/10;再紧降 `batch_size→500`);治本是 WSLg GPU 直通。
  - ⚠️ **MPPI 强约束:`1/controller_frequency ≤ model_dt`**(否则 configure 报 "Controller period more then model dt")。**降 `controller_frequency` 必须同步把 `model_dt` 抬到 = 周期**(现 10Hz↔model_dt 0.1)。`horizon = time_steps × model_dt`。
- **转弯特别慢 / 弯道像停下来转**（紧弯 vx 被 `vx=wz·r` 钳死）→ 提 `wz_max`(现 1.8,≤底盘 2.0)、`vx_max`(现 1.0,≤底盘 1.5)、`wz_std`(现 0.6);仍嫌弯太碎可调大 `minimum_turning_radius`(0.2→0.4,牺牲窄道机动)。
- **局部 costmap 障碍清不掉 / 刷错** → 确认 STVL 源 `/cloud_registered`、`sensor_frame:body`、清除源 `model_type:1`(3D lidar)与 FOV 角度;STVL 视锥清除无旧 `origin_z` 坑(若换回 VoxelLayer 才需 `origin_z=-1.0`)。
- **behavior 报 odom/帧不存在** → 确认 behavior global_frame=camera_init、odom_topic=/base_controller/odom。
- **改 launch/config 不生效** → 必须重新 `colcon build`（launch 跑 install/ 副本）。
- **ament_python 测试 colcon 不发现** → `python3 -m pytest core/navigation/robot_navigation/test` 兜底。

## 已知限制（最小范围,构建机实测确认;均留后续阶段）
- **堵路新障碍已能绕(C1 已解)**:全局+局部 costmap 现均用 STVL 实时障碍层(`spatio_temporal_voxel_layer`,源 `/cloud_registered`,视锥+时间衰减清除),全局 Smac 规划器看到新障碍即重规划绕行。**残留**:运动障碍(行人)避让为独立里程碑;起步点近场盲区(FAST-LIO `blind=1.0`)障碍仍不可见,留后续。
- **Pause/Resume 不续行**:Nav2 面板 Pause=lifecycle deactivate(取消当前目标),Resume=activate(无"续上原目标"语义)→ 车闲置、RViz 残留上次 `/plan`。要继续=重发目标(Nav2 机制,非 bug)。
- **Waypoint 模式不可用**:`waypoint_follower` 未启用(见路线图);穿点请用 RViz「Nav Through Poses」(`navigate_through_poses`,已可用)。
- **U 形死路绕行撞墙(近场盲区擦已知障碍)**:动态新增的长条障碍 A、B(全局静态图无)堵出一条 U 形死路,且足够长(车须驶入一段才靠局部 costmap 发现实死路);绕行转向时车贴该障碍更近,其一端落入雷达近场盲区(FAST-LIO `blind=1.0`),该段 voxel 被清除 → 局部 costmap 出现假缺口 → MPPI 钻缺口撞墙。根因=近场盲区 + STVL 视锥清除在贴近时擦除已观测障碍(绕行/重规划本身正确)。留后续:缩 `blind` / 盲区保形(已知 voxel 不随视锥擦) / `collision_monitor` 兜底。

## 后续路线图（不在 5e）
- **穿点导航**：启用 `waypoint_follower` + `navigate_through_poses`（参数/BT 已内置默认）。
- **动态障碍**：新建动态障碍仿真包（重新设计，不复用旧实现）+ MPPI 调避让（差速 + MPPI 预测式，目标超越旧栈 DWB 的 stop-and-wait）。
- **STVL / velocity_smoother / collision_monitor**：按需。
