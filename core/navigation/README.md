# Navigation：Nav2 自主导航

本模块把 GICP 全局定位、FAST-LIO 连续局部坐标和轮式速度接入 Nav2。全局规划器使用 Smac Hybrid-A*，局部控制器使用 MPPI，不运行 AMCL。

## 坐标和数据流

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ──> base_footprint

map_server ──> global costmap ──> Smac planner
/cloud_registered_body ──> local/global STVL ──> MPPI ──> /cmd_vel_auto
```

global costmap 使用 `map`，local costmap 和 behavior server 使用连续的 `camera_init`。障碍源是 `/cloud_registered_body`、sensor frame=`body`；轮式速度来自 `/base_controller/odom`。完整 bringup 把 Nav2 `Twist` 转为 `/cmd_vel_auto`，再由 `cmd_vel_gate` 输出 `TwistStamped /cmd_vel`。

系统级 TF、启动和控制所有权见[系统架构](../../docs/architecture.md)。

## 配置所有权

正式 Nav2 原生模板为 `core/bringup/system_bringup/config/templates/nav2.yaml`。runtime compiler 从 Profile 注入 footprint 和运动限制，生成 `nav2.generated.yaml`；平台几何只在 `profiles/sim.yaml` 或 `profiles/real.yaml` 维护。

MPPI 必须满足：

```text
1 / controller_frequency <= model_dt
```

改变控制频率时同步检查 `model_dt` 和总预测时域。

## 准备地图并运行

从先验 PCD 生成二维地图：

```bash
cd core
source install/setup.bash
ros2 run robot_navigation pcd_to_occupancy \
  --pcd ~/result/GlobalMap.pcd \
  --out ~/result/factory_map.yaml \
  --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2
```

该工具依赖 Open3D。地图路径通过 `bringup.yaml` 的 `map_artifacts.nav2_map` 提供，不写死到 launch。

把 mode 设为 `navigation` 并选择 platform，再通过正式入口启动。RViz 会在 Nav2 前显示 prior map 与 registered cloud；初值错误时发布 `/initialpose`。首次 accepted 后 `/localization` 出现，ready gate 才启动 Nav2。确认点云贴合后再下发 goal。

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

- 五个 Nav2 lifecycle 节点均为 active。
- `map -> camera_init -> body -> base_footprint` 连通且各 TF 边只有一个发布者。
- global/local costmap 正常加载地图和 STVL 障碍层。
- 目标产生可行路径，MPPI 持续输出，gate 是唯一 `/cmd_vel` 发布者。
- 动态验收中车辆安全到达目标，障碍能够 mark/clear，控制器没有持续超时。

真机静态检查不能推断动态导航通过。动态测试必须有人看护并准备物理急停，按当前目标、场地和安全条件定义观察项与停止条件。

## 限制与排错

- 运动障碍预测、waypoint follower、全局重定位和首次有效 GICP readiness 尚未形成完整验收。
- FAST-LIO 近场盲区与 STVL 清除可能形成贴障假缺口，需要独立 collision monitor 兜底。
- Nav2 lifecycle Pause 会取消当前目标，Resume 不会续上原目标。
- 车反向或不转时检查 `body -> base_footprint` 和轮方向，不用 Nav2 参数掩盖 TF 错误。
- MPPI 无法 configure 时检查控制频率与 `model_dt`；costmap 异常时检查 registered cloud、sensor frame、STVL 视锥和高度。
- Profile/`bringup.yaml` 修改不需重建；package share 中的 template/launch 修改后需要重建。
