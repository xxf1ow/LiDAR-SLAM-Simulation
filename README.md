# LiDAR-SLAM-Simulation

差速机器人在 Gazebo **Sim Harmonic** 工厂世界里的 LiDAR-SLAM 仿真栈,运行于 **ROS 2 Humble**。
运行时目标:**FAST-LIO2 里程计 + 在 LIO-SAM 先验图里做 GICP 定位 + Nav2 自主导航**。

> 本仓库是旧 Gazebo-Classic 栈(已删除的 `src/` 工作区)的**自底向上重建**,以官方参考
> (`ros2_control_demos` 建机器人、Nav2 官方示例建导航)与干净的 ros2_control `sim/mock/real`
> 三态切换为准,不照搬旧实现。

---

## 架构总览

**colcon 工作区根 = `core/`**(自成一体;上游克隆落在各模块名下,不在单独的 `src/`)。五个解耦模块:

| 模块 | 角色 | 详细文档 |
|---|---|---|
| `core/robot/` | 差速机器人描述 + ros2_control(`robot_description` 单一 xacro 的 gz/mock/real 三态、`robot_hardware` 真机 C++ 硬件接口、`robot_bringup` 控制器 + launch) | `core/robot/` 各包 |
| `core/simulation/` | Gz Harmonic 世界 + 传感器桥接(`robot_gz_bringup` 起世界/spawn/桥接/控制器、`lidar_pointcloud_adapter` 把 Gz 组织化点云转成 Velodyne 风格 `/points_raw`、`spike` 雷达冒烟) | [`core/simulation/robot_gz_bringup/README.md`](core/simulation/robot_gz_bringup/README.md) |
| `core/mapping/` | **LIO-SAM** 建图、保存先验图 `~/result/GlobalMap.pcd` | [`core/mapping/README.md`](core/mapping/README.md) |
| `core/localization/` | **FAST-LIO2** 里程计 + **GICP** 先验图定位(`gicp_localization` 自研包) | [`core/localization/README.md`](core/localization/README.md) |
| `core/navigation/` | **Nav2** 自主导航(Smac Hybrid-A\* 全局 + MPPI 局部) | [`core/navigation/README.md`](core/navigation/README.md) |

每个模块的 README 含各自的 clone + 编译 + 运行 + 验收步骤。本文件只做总览与环境前置。

### 运行时 TF 链(完整栈)
```
map ─[gicp_localization]→ camera_init ─[FAST-LIO]→ body ─[静态焊接]→ base_footprint ─[URDF]→ base_link → velodyne/imu_link/轮
```
先验图在 LIO-SAM 的 `map` 帧;FAST-LIO 的 `camera_init`/`body` 仅在导航阶段经 `body→base_footprint`
焊接(单位旋转、z=-0.556)接入机器人 URDF 树。无 AMCL —— `map→camera_init` 由 GICP 提供。

### 关键话题
- Gz 侧:`/lidar/points`(组织化点云 ~10 Hz)、`/imu`、`/clock`,经 `ros_gz_bridge` 桥接。
- `lidar_pointcloud_adapter`:`/lidar/points` → `/points_raw`(Velodyne 风格,含 `ring`+合成 `time`,frame=`velodyne`)。
- `/imu_plugin/out`(Imu, 200 Hz,SLAM 契约话题名不变)。
- `/cmd_vel`:**TwistStamped**(Humble `diff_drive_controller` 默认类型);Nav2 发 `Twist` 到 `/cmd_vel_nav`,经 `twist_stamper` 转 TwistStamped 出 `/cmd_vel`。
- `/base_controller/odom`:轮式里程计(真实 twist,供 Nav2);`enable_odom_tf:false`,`odom→base_footprint` 由 SLAM 独占。
- FAST-LIO:`/Odometry`、`/cloud_registered`(`camera_init` 帧)、TF `camera_init→body`。
- GICP:TF `map→camera_init`、`/localization`。

---

## 环境前置(构建机)

> [!CAUTION]
> - **系统**:Ubuntu 22.04(支持 WSL2)
> - **ROS**:ROS 2 Humble(LTS)
> - **Gazebo**:Gazebo **Sim Harmonic**(经 `ros-humble-ros-gz` 集成,非 Classic / 非 Fortress)

```bash
# ROS 2 Humble 桌面版(参考 https://docs.ros.org/en/humble/Installation.html)
sudo apt install ros-humble-desktop ros-dev-tools

# Gz Harmonic + ros_gz 集成 + ros2_control(仿真控制器经 gz_ros2_control 插件提供)
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers

# SLAM / 定位依赖
sudo apt install ros-humble-perception-pcl libomp-dev libgtsam-dev libgtsam-unstable-dev

# Nav2(导航阶段)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

> WSL2 + NVIDIA 显卡加速(D3D12 / Vulkan mesa)按官方文档配置即可;Gz Harmonic 软渲下
> RTF 可能 <1,`ros2 topic hz` 读到的频率会按 RTF 折算,属正常。

### 上游克隆 + 资产
上游依赖**不入库**(gitignore),按各模块 README 的 pinned SHA clone 到其所属模块目录、再 `git apply` 该模块跟踪的补丁:
- `core/mapping/LIO-SAM`(+ `core/mapping/lio-sam.patch`)
- `core/localization/FAST_LIO`(+ `core/localization/fast-lio2.patch`)
- `core/localization/small_gicp`

工厂世界 `factory.sdf` 的 mesh 视觉引用 Classic 资产库(`models/factory_model/`,~30MB,gitignore),
需备到构建机;`robot_gz.launch.py` 的 `factory_models_path` 默认指向 `~/LiDAR-SLAM-Simulation/models/factory_model`。

---

## 快速运行(每个终端先 `cd core && source install/setup.bash`)

```bash
# 1) 仿真(Gz Harmonic 工厂世界 + 机器人 + 传感器桥接)
ros2 launch robot_gz_bringup robot_gz.launch.py

# 2) 建图:LIO-SAM 跑一圈、存先验图(产物 ~/result/GlobalMap.pcd)—— 仅需做一次
ros2 launch lio_sam run.launch.py
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: /result}"

# 3) 定位栈:FAST-LIO 里程计 + GICP 先验图定位
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
ros2 launch gicp_localization localization.launch.py
#   启动后在 RViz「2D Pose Estimate」给机器人真实 map 位姿,等 /localization 锁定(fitness≥0.9)

# 4) 导航:先把先验图 PCD 离线投影成 2D 占据栅格,再起 Nav2
ros2 run robot_navigation pcd_to_occupancy --pcd ~/result/GlobalMap.pcd \
  --out ~/result/factory_map.yaml --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2
ros2 launch robot_navigation navigation.launch.py
#   RViz 给「Nav2 Goal」即自主导航。完整流程/验收见 core/navigation/README.md
```

> 建图与定位/导航**不同时跑**:建图用 LIO-SAM 产先验图;之后定位栈(FAST-LIO+GICP)在该图上跑。

---

## 已知限制

- **本仿真无可演示的漂移**:特征丰富的工厂里 FAST-LIO 受 LiDAR 扫描匹配强约束、近乎无漂移
  (即便 IMU 噪声调到极大也诱发不出);故 GICP "纠正累积漂移" 的收益在本仿真**无法直观演示**——
  属仿真环境性质、非 GICP 缺陷,真实/退化场景(长走廊、空旷)才显现。
- **GICP 是局部配准 + 工厂 90° 伪对称假解**:大偏差 / ~90° 不自动恢复;工厂的 90° 旋转伪对称
  会形成 `fitness≈0.8` 的强假解,默认 `fitness_threshold=0.9` 即用于拒绝它。bootstrap 须给正确
  `/initialpose`。全自动从任意位姿恢复属后续全局重定位(Quatro 前端)阶段。
- **仿真点云无原生逐点 `time`**:由 `lidar_pointcloud_adapter` 按方位角合成,FAST-LIO 据此去畸变
  (Gz 是瞬时快照、无真实帧内畸变,故为合成时间);LIO-SAM 对无 time 输入自动关去畸变,存图不受影响。
- **Nav2 最小栈的当前限制**(详见 `core/navigation/README.md`):堵死路的新障碍会撞不会绕
  (全局 costmap 无实时障碍层)、Pause/Resume 不续行、waypoint 模式未启用。均留后续阶段。
- **MPPI 强约束**:`1/controller_frequency ≤ model_dt`,降控制频率须同步抬 `model_dt`。

---

## 数据流参考
仿真侧消息/帧/TF 的逐项注解见 [`docs/sim-dataflow-lio-sam.md`](docs/sim-dataflow-lio-sam.md)。
