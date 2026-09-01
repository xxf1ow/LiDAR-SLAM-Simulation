# Localization：FAST-LIO2 与 GICP

FAST-LIO2 消费 LiDAR/IMU，发布 `camera_init -> body`、`/Odometry` 和 `/cloud_registered_body`。`gicp_localization` 把注册点云配准到 LIO-SAM 先验图，发布 `map -> camera_init`；只有首次配准被接受后才发布 `/localization`。

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ── slam_stack ──> base_footprint
```

系统级 TF 和启动所有权见[系统架构](../../docs/architecture.md#传感器和坐标流)。

## FAST-LIO 集成

FAST-LIO 克隆到忽略目录 `core/localization/FAST_LIO`，项目修改由 `core/localization/fast-lio2.patch` 交付：

```bash
git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 --single-branch --depth 1 \
  --filter=blob:none core/localization/FAST_LIO
cd core/localization/FAST_LIO
git fetch origin a4743b095409588842a5b30ddfa27e29d2f99164 --depth 1
git checkout a4743b095409588842a5b30ddfa27e29d2f99164
git apply ../fast-lio2.patch
```

补丁把 IMU 订阅改为 `SensorDataQoS`，并使 world/body 点云发布开关独立。正式参数由 `system_bringup` 生成的 `fast_lio.generated.yaml` 提供。算法外参不等同于 URDF 的 LiDAR/IMU mount；真实六自由度外参需要动态标定。仓库内 `livox_ros_driver2` 只提供 FAST-LIO 编译期消息类型，不依赖 Livox SDK。

## GICP 集成

`gicp_localization/` 是项目代码，使用 small_gicp 低频更新校正并高频发布当前预览或最后一次接受的 `map -> camera_init`。fitness 未达阈值时保留上一次校正；首次 accepted 前不发布 `/localization`，避免把进程存活当作定位就绪。

small_gicp 克隆到忽略目录：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/koide3/small_gicp.git --depth 1 \
  --filter=blob:none core/localization/small_gicp
cd core/localization/small_gicp
git fetch origin 78f2e7a221720625eb95271ad9da21a04fb77f86 --depth 1
git checkout 78f2e7a221720625eb95271ad9da21a04fb77f86
```

构建依赖 `libomp-dev`。诊断发布使用 `-DGICP_DIAGNOSTICS=ON` 编译期开关。

## 正式运行与配置

准备 `~/result/GlobalMap.pcd`，把 `bringup.yaml` 设为 `mode: navigation` 并选择 platform，然后通过正式入口启动。manifest 提供 `fast_lio.generated.yaml` 和 `gicp.generated.yaml`；`map_artifacts.prior_pcd` 提供本次先验地图覆盖。

RViz 提前显示 prior map 与注册点云。初值错误时使用 “2D Pose Estimate” 发布 `/initialpose`；确认 GICP accepted 且点云贴合后再下发 Nav2 goal。

Vanjee `timestamp_unit: 0` 表示逐点 `time` 使用秒。更换固件或驱动后应复核一帧约覆盖 0–0.1 s；范围异常时先修正驱动时间语义，不得用扫描频率掩盖。

## 验收与诊断

- `/Odometry`、`/cloud_registered_body` 持续发布，注册点云结构稳定。
- GICP 成功加载 prior PCD，`map -> camera_init -> body -> base_footprint` 连通且各边所有权唯一。
- 实时注册点云与先验图贴合，错误对称假解被 fitness 阈值拒绝。
- `/initialpose` 能在合理初值范围内重新引导配准。

包级独立 launch 只用于分离 FAST-LIO/GICP 问题；必须先通过 runtime compiler 生成本次参数目录。常用观察为 `/Odometry`、`/cloud_registered_body`、`/localization`、`map -> camera_init` 和 `camera_init -> body`。

## 已知边界

- GICP 是局部配准，不能从任意大偏差自动恢复。
- `/initialpose` 的机器人基准帧与 GICP `body` 初值换算仍需统一。
- 下发目标前仍需确认首次有效配准；进程和 topic 存在不等价于地图贴合。
- 仿真点云 `time` 是 adapter 合成的方位时间，旋转拖影应先检查该假设。
