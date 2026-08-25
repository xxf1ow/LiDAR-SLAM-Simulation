# Navigation：Nav2 自主导航

本模块把 GICP 全局定位、FAST-LIO 连续局部坐标和轮式里程计接入 Nav2。全局规划器使用
Smac Hybrid-A*，局部控制器使用 MPPI，不运行 AMCL。

## 坐标和数据流

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ──> base_footprint

map_server ──> global costmap ──> Smac planner
/cloud_registered_body ──> local/global STVL ──> MPPI ──> /cmd_vel_nav
```

- global costmap 使用 `map`，允许 GICP 校正后全局重规划。
- local costmap 和 behavior server 使用连续的 `camera_init`。
- 障碍源是 `/cloud_registered_body`、sensor frame=`body`，不是原始 `/points_raw`。
- 轮式速度来自 `/base_controller/odom`；FAST-LIO `/Odometry` 不提供可用 twist。
- 完整 bringup 把 Nav2 `Twist` 转为 `/cmd_vel_auto`，再由 `cmd_vel_gate` 输出
  `TwistStamped /cmd_vel`。

## 配置所有权

正式 Nav2 原生配置模板位于：

```text
core/bringup/system_bringup/config/templates/nav2.yaml
```

runtime compiler 从 Profile 注入 footprint 和运动限制，生成 `nav2.generated.yaml`。
`templates/nav2.yaml` 保持算法配置所有权；平台几何只改 `profiles/sim.yaml` 或
`profiles/real.yaml`，不要在多个 Nav2 文件重复维护车体尺寸。

MPPI 必须满足：

```text
1 / controller_frequency <= model_dt
```

改变控制频率时同步检查 `model_dt` 和总预测时域。

## 准备二维地图

```bash
cd core
source install/setup.bash
ros2 run robot_navigation pcd_to_occupancy \
  --pcd ~/result/GlobalMap.pcd \
  --out ~/result/factory_map.yaml \
  --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2
```

该工具需要 Open3D。`map_artifacts.nav2_map` 是运行时地图覆盖；不要把路径写死到 launch 源码。

## 正式运行

将 `bringup.yaml` 设置为 `mode: navigation` 并选择 `platform: sim|real`：

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

Navigation 模式先启动 RViz、FAST-LIO 和 GICP，但暂不启动 Nav2。RViz 同时显示
`/gicp_localization/prior_map` 与 `/cloud_registered_body`：初始位姿正确时 GICP 会自动接受第一次
配准；不正确时使用“2D Pose Estimate”发布 `/initialpose`。首次 accepted 后 `/localization`
出现，现有 gate 才启动 Nav2。确认点云贴合后再下发“Nav2 Goal”。手机 Web 可人工接管；恢复
自动不会自动取消或重建原 Nav2 goal。

## 验收

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /cmd_vel -v
```

应满足：

- 五个 Nav2 lifecycle 节点均为 active。
- `map -> camera_init -> body -> base_footprint` 连通且每条 TF 边只有一个发布者。
- 全局/局部 costmap 正常加载地图和 STVL 障碍层。
- 目标产生可行路径，MPPI 持续输出，gate 是唯一 `/cmd_vel` 发布者。
- 实际动态验收中车辆安全到达目标，障碍能够 mark/clear，控制器没有持续超时。

## 真机边界

真机静态检查可以验证节点、话题、TF、地图和 lifecycle，但不能推断动态导航通过。动态测试
必须有人看护，并准备物理急停；需分别验证传感器轴向、外参、轮速反馈、GICP、局部障碍、
路径跟踪和失联停车。

## 已知限制

- 运动障碍预测式避让尚未形成完整场景验收。
- FAST-LIO 近场盲区与 STVL 清除可能在贴近障碍时形成假缺口，需 collision monitor 兜底。
- waypoint follower 尚未启用；`navigate_through_poses` 可用于基础穿点。
- Nav2 lifecycle Pause 会取消当前目标，Resume 不会续上原目标。
- 全局重定位和首次有效 GICP 配准 readiness 尚未完成。

## 排错

- 车反向或不转：检查 `body -> base_footprint` 旋转和轮方向，不要用补偿性 Nav2 参数掩盖 TF。
- MPPI 无法 configure：检查 `controller_frequency` 与 `model_dt` 约束。
- costmap 障碍异常：检查 `/cloud_registered_body`、`sensor_frame=body`、STVL 视锥和高度范围。
- behavior 报 frame/odom 缺失：确认 `camera_init`、`/base_controller/odom` 和完整 TF 链。
- 配置修改不生效：Profile/`bringup.yaml` 不需 rebuild；安装到 package share 的 template/launch
  修改后必须重新构建。
- WSL RTF 过低：减少重复 GUI/RViz，确认 GPU 加速并清理遗留进程，再考虑降低 MPPI 负载。
