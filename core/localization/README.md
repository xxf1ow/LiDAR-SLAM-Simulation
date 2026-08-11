# Localization：FAST-LIO2 + GICP

本模块由两部分组成：

- FAST-LIO2 消费 LiDAR/IMU，发布 `camera_init -> body`、`/Odometry` 和
  `/cloud_registered`。
- `gicp_localization` 把 `/cloud_registered` 配准到 LIO-SAM 先验图，发布
  `map -> camera_init` 和 `/localization`。

完整定位链：

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ── weld ──> base_footprint
```

## FAST-LIO 集成

FAST-LIO 克隆到 `core/localization/FAST_LIO`，项目修改由
`core/localization/fast-lio2.patch` 交付：

```bash
git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 --single-branch --depth 1 \
  --filter=blob:none core/localization/FAST_LIO
cd core/localization/FAST_LIO
git fetch origin a4743b095409588842a5b30ddfa27e29d2f99164 --depth 1
git checkout a4743b095409588842a5b30ddfa27e29d2f99164
git apply ../fast-lio2.patch
```

补丁包含 Gazebo 与 Vanjee 两套配置，并使 IMU 订阅兼容 BEST_EFFORT。配置中的算法外参是
当前兼容基线，不等同于 URDF 的独立 LiDAR/IMU mount；真实六自由度外参仍需动态标定。

`livox_ros_driver2` 是仓库内的消息桩，只满足 FAST-LIO 编译期类型依赖，不需要 Livox SDK。

## GICP 集成

`gicp_localization/` 是本仓库代码。它使用 small_gicp，以低频配准更新校正，同时高频发布
最后一次接受的 `map -> camera_init`。接受条件是 fitness 达到阈值；被拒绝时保持上一次有效
校正。

small_gicp 克隆到 `core/localization/small_gicp`：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/koide3/small_gicp.git --depth 1 \
  --filter=blob:none core/localization/small_gicp
cd core/localization/small_gicp
git fetch origin 78f2e7a221720625eb95271ad9da21a04fb77f86 --depth 1
git checkout 78f2e7a221720625eb95271ad9da21a04fb77f86
```

编译依赖 `libomp-dev`。诊断发布是编译期开关：

```bash
colcon build --packages-up-to gicp_localization \
  --cmake-args -DGICP_DIAGNOSTICS=ON
```

## 正式运行

准备 `~/result/GlobalMap.pcd`，把 `bringup.yaml` 设置为 `mode: navigation`，并选择
`platform: sim|real`：

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

shared sensor gate 通过后启动 FAST-LIO；随后 GICP 和底盘里程计 gate 放行 Nav2。平台对应的
FAST-LIO、GICP 配置和地图路径仍由 `bringup.yaml` 选择。

## Vanjee 逐点时间检查

Vanjee 配置中的 `timestamp_unit: 0` 表示每点 `time` 以秒为单位。已验证的一帧应约覆盖
0–0.1 s；更换固件或驱动后应重新检查。若范围不符，先停下定位验收并修正驱动时间语义，
不要通过修改扫描频率掩盖。

## 独立诊断

以下命令只用于分离 FAST-LIO/GICP 问题，不替代正式 bringup：

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=gazebo_velodyne.yaml use_sim_time:=true
ros2 launch gicp_localization localization.launch.py
```

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /localization
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init body
```

## 验收

- `/Odometry`、`/cloud_registered` 持续发布，注册点云结构稳定。
- GICP 成功加载 `GlobalMap.pcd`，`~/prior_map` 可显示。
- `map -> camera_init -> body` 连通；完整栈还应连到 `base_footprint`。
- 实时注册点云与先验图贴合，错误 90° 假解被当前 fitness 阈值拒绝。
- `/initialpose` 能在合理初值范围内重新引导配准。

## 已知边界

- GICP 是局部配准，不能从任意大偏差自动恢复。
- `/initialpose` 的机器人基准帧与 GICP `body` 初值换算仍需最终统一。
- bringup 当前等待 GICP 图接口和底盘里程计，不等价于“首次有效配准已完成”。下发导航目标
  前仍应确认点云与先验图贴合。
- 仿真点云 `time` 是适配器合成方位时间；旋转拖影应先检查该假设，而不是盲目修改 FAST-LIO。

## 排错

- FAST-LIO 配置或类型缺失：确认补丁已 apply，使用 `--packages-up-to fast_lio` 构建依赖。
- 没有里程计：检查 `/points_raw`、`/imu/data`、QoS、时钟和 frame。
- small_gicp 找不到：确认 clone 路径、固定提交和 `libomp-dev`。
- 先验图加载失败：确认 `~/result/GlobalMap.pcd` 存在且当前用户可读。
- fitness 持续偏低：检查初值、外参、点云时间、地图坐标和 90° 对称假解。
