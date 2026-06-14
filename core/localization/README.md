# core/localization — FAST-LIO 里程计(+ 后续 GICP 定位)

本模块负责**激光-惯性里程计**与**先验图定位**。分两步落地:

- **5c(本次)— FAST-LIO 里程计**:用 FAST-LIO2 在 Gz Harmonic 工厂世界里跑 LiDAR-IMU 里程计,
  发布 `camera_init→body` TF、`/Odometry`、`/cloud_registered`。这是定位栈的地基。
- **5d(后续)— GICP 定位**:`gicp_localization` 把 FAST-LIO 的 `/cloud_registered` 在 5b 先验图
  (`GlobalMap.pcd`)上做 scan-to-map 配准,发布校正 `map→camera_init`。需 `small_gicp`,留到 5d 接入。

## 集成方式:clone + patch,落在本模块名下(core 自成一体)
FAST-LIO 源码 **clone 到本模块目录 `core/localization/FAST_LIO`**(被 `.gitignore` 排除、不入库),
再 `git apply` 本模块跟踪的两个补丁。**不放在 src 下**——core 是自成一体的完整 colcon 工作区。

- `fast-lio2.patch`(**中性**,对真实无害):新增 `config/gazebo_velodyne.yaml`——话题
  `points_raw` / `/imu_plugin/out`、velodyne `lidar_type:2`、`scan_line:16`、blind 1.0、
  **lidar-IMU 外参归零** `extrinsic_T:[0,0,0]`(新模型 velodyne 与 imu_link 同位置,与 mapping 一致)。

> **去畸变说明**:`lidar_pointcloud_adapter` 已给点云补 `time` 字段(按列号合成 `(i%width)/width*0.1`),
> FAST-LIO 检测到非零 time 即自动 `given_offset_time=true`、用该 time 去畸变。注意这是**合成**的方位角
> 时间(Gz 是瞬时快照、无真实帧内畸变),FAST-LIO 仍会按它做一次去畸变。若构建机原地旋转测试(判据 25)
> 出现拖影/发散,治本是让 adapter 发**常量 time**(令去畸变成恒等),而非在 FAST-LIO 侧打补丁。

`livox_ros_driver2`(本目录内,**入库**)是仅含 `CustomMsg`/`CustomPoint` 的**消息桩包**,只为满足
FAST-LIO 在 velodyne-only 配置下的编译期类型依赖——无驱动、不需 Livox-SDK。

clone + apply(从仓库根):
```bash
git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 --single-branch --depth 1 \
  --filter=blob:none core/localization/FAST_LIO
cd core/localization/FAST_LIO
git fetch origin a4743b095409588842a5b30ddfa27e29d2f99164 --depth 1
git checkout a4743b095409588842a5b30ddfa27e29d2f99164
git apply ../fast-lio2.patch
```

**改 FAST-LIO 配置的正确姿势**:改 `core/localization/FAST_LIO` working tree → `git diff > ../fast-lio2.patch`
重生成 → 提交补丁。构建机重新 `git apply`。

## TF 约定:FAST-LIO 是**平行子树**,不接进 URDF 树
```
camera_init ─(FAST-LIO 激光-惯性里程计)→ body          ← FAST-LIO 自有里程计子树
map/odom ─…→ base_footprint ─(URDF)→ base_link ─→ velodyne/imu_link/轮   ← 机器人 URDF 树
```
- FAST-LIO 的 `camera_init`/`body` **不焊接进机器人 URDF 树**(把 `body` 焊到 `base_footprint`
  是 nav2 阶段的事,5c 不做)。5c 里验证 FAST-LIO 看自己的子树即可。
- 5c 运行时 **LIO-SAM 不跑**,轮式里程计 TF 仍关(`enable_odom_tf:false`),故没有 `odom→base_footprint`;
  机器人 URDF 树暂时"悬空",**RViz 关于 `base_footprint` 无父帧的告警是预期的、非 bug**(与旧栈一致)。
- 5d 起 GICP 后,`map→camera_init` 补上,定位子树变 `map→camera_init→body`。

## 构建机:FAST-LIO 里程计验证流程
前置:**构建根 = `core/`**;`core/localization/FAST_LIO` 已 clone + apply 两个补丁;
Phase 4 的 `models/factory_model` + `GZ_SIM_RESOURCE_PATH` 就位(launch 默认路径已内化)。

```bash
# 构建(从 core 工作区根;livox 桩先建,再 fast_lio)
cd core && colcon build --packages-up-to fast_lio robot_gz_bringup
source install/setup.bash

# 终端 1：起仿真(工厂世界 + 机器人 + 传感器)
cd core && source install/setup.bash
ros2 launch robot_gz_bringup robot_gz.launch.py

# 终端 2：起 FAST-LIO 里程计(velodyne 仿真配置 + sim 时钟)
cd core && source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true

# 终端 3：sticky 键盘遥控,缓慢遍历工厂(use_sim_time 已默认 true)
ros2 run robot_gz_bringup sticky_teleop.py
#   i/, = 前进/后退   j/l = 左/右转   k或空格 = 停   s = 回正   q = 退出
```

## 验收判据(PASS → 进 5d GICP 定位)
22. 终端 2 起 FAST-LIO 后无 `extrinsic`/`frame`/话题报错;`ros2 topic hz /Odometry` 持续发布。
23. `ros2 run tf2_ros tf2_echo camera_init body` 有输出且随车移动;RViz 里 `/cloud_registered`
    勾勒出工厂结构、随行驶累积一致(不重影、不发散)。
24. 里程计贴合真实运动:开一段已知路线(直线 + 转弯),FAST-LIO 轨迹与实际路径大致吻合、尺度正确
    (本仿真特征丰富,LiDAR 充分约束、基本不漂)。
25. **原地旋转点云保持清晰**:原地转一圈,`/cloud_registered`/地图无明显拖影、发散
    ——若发散,根因是 FAST-LIO 按 adapter 合成的逐点 time 对快照云去畸变(见上"去畸变说明"),
    治本在 adapter 侧(发常量 time)。

## FAIL 排查
- FAST-LIO 起来即报 `extrinsic`/`frame` 或点云方向错乱 → 确认 `fast-lio2.patch` 已 apply、
  `config_file:=gazebo_velodyne.yaml` 传对,且构建机重新 `git apply` + `colcon build fast_lio`。
- 编译报 `livox_ros_driver2` 类型缺失 → 桩包未建:`--packages-up-to fast_lio` 会先建 `livox_ros_driver2`,
  确认它在 `core/localization/` 下且被 colcon 发现。
- `/Odometry` 不发 / 节点静默 → 查 `points_raw`(经 lidar_pointcloud_adapter 补 ring/time)与
  `/imu_plugin/out` 是否在发(`ros2 topic hz`),use_sim_time 是否 true(否则等不到 /clock)。
- 旋转拖影/发散 → FAST-LIO 对 Gz 快照云按 adapter 合成 time 去畸变所致;治本=让 lidar_pointcloud_adapter
  发常量 time(快照云无帧内畸变,令去畸变成恒等),而非改 FAST-LIO。
- 车不动 → 见 mapping/README 同条(teleop 终端焦点 / 默认 stamped+sim_time)。
