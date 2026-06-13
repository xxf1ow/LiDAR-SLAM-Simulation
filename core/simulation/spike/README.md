# Phase 0 雷达 Spike — build 机执行协议

**目的:** 坐实"Gz Harmonic 的 `gpu_lidar` → `ros_gz_bridge` → 补 ring/time → 现有 FAST-LIO 能消费并出 `/Odometry`"。这是整个 Gz 重建的**硬闸门**:过了进 Phase 1,不过回决策桌。

**前置:** build 机 Ubuntu 22.04 + ROS 2 Humble + Gazebo **Harmonic** + `ros-humble-ros-gz`(或对应 gz 供应包);本仓库 `core/simulation/`(`lidar_pointcloud_adapter` 包 + `spike/`)已拷到 build 机工作区。

## 1. 构建并源化

```bash
cd <ws> && colcon build --packages-select lidar_pointcloud_adapter && source install/setup.bash
ros2 pkg executables lidar_pointcloud_adapter   # 应列出: lidar_pointcloud_adapter adapter_node
```

## 2. 起 spike(终端一)

```bash
ros2 launch <绝对路径>/core/simulation/spike/launch/spike_bringup.launch.py
```
预期:Gz 起来,能看到墙/柱 + 雷达可视化射线;无致命插件加载错误。
- 若报 `gz-sim-*-system` 找不到:`gz sim` 看大版本(Harmonic=8),插件名应为 `gz-sim-sensors-system` 等;Fortress 则为 `ignition-gazebo-sensors-system`,据此改 `worlds/spike_lidar.sdf`。

## 3. 核验 Gz 话题与点云组织(终端二)— **关键学习点**

```bash
gz topic -l | grep -E "lidar|imu"        # Gz 侧话题名(点云常为 /gz_lidar/points)
ros2 topic list | grep -E "points|imu|clock"
ros2 topic echo /gz_lidar/points --once --field height
ros2 topic echo /gz_lidar/points --once --field width
ros2 topic echo /gz_lidar/points --once --field fields
```
判读:
- `height==16` 且 `width≈1800` → **组织化成立,适配节点的 `ring=i//width` 正确**。
- `height==1`(非组织化)→ 看 `fields` 是否已含 `ring`:含则可绕过适配直接喂;不含则需按实际改适配(见末尾"不过分支")。
- 记录 `fields` 原生是否已带 `ring`/`intensity`。
- 若 Gz 点云话题名不是 `/gz_lidar/points`,改 `config/bridge_spike.yaml` 与 launch 里的 `input_topic`。

## 4. 核验适配输出(终端二)

```bash
ros2 topic echo /points_raw --once --field fields
ros2 topic hz /points_raw
```
预期:`fields` 含 `x y z intensity ring time`;`/points_raw` 稳定约 10Hz(sim 时钟下 `hz` 可能显示约一半,正常)。

## 5. 喂 FAST-LIO(终端三 + 终端四观察)

```bash
# 现有 velodyne 配置:lid_topic=points_raw、imu_topic=/imu_plugin/out、lidar_type:2、scan_line:16
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
```
```bash
ros2 topic hz /Odometry
ros2 topic list | grep -E "Odometry|cloud_registered"
```
（可选,验证跟踪)Gz GUI 拖动 `sensor_rig`,或:
```bash
gz service -s /world/spike/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 300 \
  --req 'name: "sensor_rig", position: {x: 0.5, y: 0.0, z: 0.5}'
```

## 6. 裁决

**PASS(进入 Phase 1)需全部满足:**
1. 拿得到 Gz 点云,且能确定每点 `ring`(组织化 `height==16`,或原生已带 `ring` 字段)。
2. `/points_raw` 带 `x y z intensity ring time`,稳定发布。
3. FAST-LIO 启动**无 preprocess 崩溃/报错**,持续发 `/Odometry`(静态下位姿近恒定即可;拖动 rig 时随动为加分)。

**FAIL(回决策桌,不进 Phase 1):**
- 点云既非组织化、原生也无 `ring`,无法定线号 → 记录实际 `fields`/`height`/`width`;备选:① 调 `gpu_lidar` 配置/换带 ring 的输出;② 换 **RGLGazeboPlugin**(Harmonic)看其点云字段;③ 仍无解则重评 C2 可行性。
- FAST-LIO 因点云结构持续报错且适配无法弥合 → 同上,记录错误原文回报。

## 已知注意
- 适配节点用纯 Python 逐点 `create_cloud`,16×1800≈28800 点/10Hz 可能偏慢——spike 只需跑通;若拖垮 sim,记为 Phase 3 优化(numpy 向量打包 / C++ 节点)。
- `gz_frame_id`、桥接类型串取自 ros_gz/Nav2 官方;版本差异处已在上文标注核验命令。
