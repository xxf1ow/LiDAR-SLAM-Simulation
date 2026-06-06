# FAST-LIO2 Gazebo 接入（阶段 1 · 纯里程计）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让干净的 upstream FAST-LIO2 消费本仓库 Gazebo 仿真的 velodyne + IMU 话题，跑通纯 LIO 里程计（旁路、不加载先验地图）。

**Architecture:** 新增一个 `livox_ros_driver2` **消息桩包**满足 FAST-LIO 的编译期依赖（FAST-LIO 源码/CMake 零改动）；新增一份 `gazebo_velodyne.yaml` 配置（仿真话题/16线/外参），以补丁形式沉淀到 `src/fast-lio2-patch/`。FAST-LIO 用原生 frame `camera_init`/`body` 旁路运行，不接入机器人 TF 树。

**Tech Stack:** ROS 2 Humble、colcon、rosidl（消息生成）、Gazebo（仿真）、PCL/Eigen（FAST-LIO 既有依赖）、RViz2（验证）。环境为 WSL Ubuntu 22.04。

设计依据：`docs/superpowers/specs/2026-06-06-fast-lio2-gazebo-bringup-design.md`；仿真数据流：`docs/sim-dataflow-lio-sam.md`。

---

## 执行环境说明（重要）

- **建文件类任务（Task 1–2、4）**：在 Windows 文件系统操作，subagent 可直接做。
- **构建与运行类任务（Task 3、5）**：必须在 **WSL 的 ROS2 工作区**执行（`source /opt/ros/humble/setup.bash`）。Task 5 的 RViz 视觉判断为**人工检查**。
- 所有 `colcon`/`ros2` 命令默认在**工作区根目录**（含 `src/` 的那层）运行。
- 基准：FAST_LIO @ `a4743b0`（ROS2 分支），仿真参数取自已验证的 LIO-SAM 配置。

## 文件结构（本计划新增/改动）

```
src/
├── livox_ros_driver2/                 # 新增：消息桩包（一等公民包，非补丁）
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── msg/
│       ├── CustomPoint.msg
│       └── CustomMsg.msg
├── FAST_LIO/
│   └── config/gazebo_velodyne.yaml    # 新增：仿真配置（物理存在以供构建；同时沉淀为补丁）
└── fast-lio2-patch/
    └── 01-add-gazebo-velodyne-config.patch   # 新增：上面那份配置的补丁记录
```

- `livox_ros_driver2`：唯一职责 = 提供与官方字段一致的两个消息类型，使 FAST-LIO 通过编译；运行期不被使用。
- `gazebo_velodyne.yaml`：唯一职责 = 把 FAST-LIO 接到本仿真的话题/线数/外参。
- 补丁文件夹：保持 `src/FAST_LIO` upstream 干净、改动可复现。

---

### Task 1: 创建 `livox_ros_driver2` 消息桩包

**Files:**
- Create: `src/livox_ros_driver2/package.xml`
- Create: `src/livox_ros_driver2/CMakeLists.txt`
- Create: `src/livox_ros_driver2/msg/CustomPoint.msg`
- Create: `src/livox_ros_driver2/msg/CustomMsg.msg`

- [ ] **Step 1: 写 `CustomPoint.msg`（字段照抄官方）**

`src/livox_ros_driver2/msg/CustomPoint.msg`：
```
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

- [ ] **Step 2: 写 `CustomMsg.msg`（字段照抄官方）**

`src/livox_ros_driver2/msg/CustomMsg.msg`：
```
std_msgs/Header header  # ROS standard message header
uint64 timebase         # The time of first point
uint32 point_num        # Total number of pointclouds
uint8 lidar_id          # Lidar device id number
uint8[3] rsvd           # Reserved use
CustomPoint[] points    # Pointcloud data
```

- [ ] **Step 3: 写 `package.xml`**

`src/livox_ros_driver2/package.xml`：
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>livox_ros_driver2</name>
  <version>1.0.0</version>
  <description>
    Minimal messages-only stub of livox_ros_driver2 (CustomMsg/CustomPoint),
    provided only to satisfy FAST-LIO's compile-time type dependency in a
    Velodyne-only setup. Contains no driver and requires no Livox-SDK.
  </description>
  <maintainer email="20twenty.degree@gmail.com">xxf1ow</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: 写 `CMakeLists.txt`**

`src/livox_ros_driver2/CMakeLists.txt`：
```cmake
cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 5: 构建桩包验证（WSL）**

Run（工作区根目录）:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
```
Expected: `Finished <<< livox_ros_driver2`，无报错；`install/livox_ros_driver2/include/.../msg/custom_msg.hpp` 生成。

- [ ] **Step 6: 提交**

```bash
git add src/livox_ros_driver2
git commit -m "feat: 新增 livox_ros_driver2 消息桩包以满足 FAST-LIO 编译依赖(velodyne-only)"
```

---

### Task 2: 创建 FAST-LIO 的 Gazebo 配置 `gazebo_velodyne.yaml`

**Files:**
- Create: `src/FAST_LIO/config/gazebo_velodyne.yaml`

- [ ] **Step 1: 写配置文件**

`src/FAST_LIO/config/gazebo_velodyne.yaml`（值取自 spec §5/§6.1）：
```yaml
/**:
    ros__parameters:
        feature_extract_enable: false
        point_filter_num: 4
        max_iteration: 3
        filter_size_surf: 0.5
        filter_size_map: 0.5
        cube_side_length: 1000.0
        runtime_pos_log_enable: false
        map_file_path: "./test.pcd"

        common:
            lid_topic:  "points_raw"          # 仿真 velodyne 插件话题
            imu_topic:  "/imu_plugin/out"     # 仿真 IMU 插件话题
            time_sync_en: false
            time_offset_lidar_to_imu: 0.0

        preprocess:
            lidar_type: 2                     # 2 = Velodyne
            scan_line: 16                     # VLP-16
            scan_rate: 10                     # Hz，决定缺逐点 time 时方位角去畸变的重建尺度
            timestamp_unit: 2                 # 仿真无 time 字段→走回退分支，本项实际不起效
            blind: 1.0                        # 对齐仿真最小距离

        mapping:
            acc_cov: 0.1
            gyr_cov: 0.1
            b_acc_cov: 0.0001
            b_gyr_cov: 0.0001
            fov_degree:    360.0
            det_range:     100.0
            extrinsic_est_en:  true           # 在线微调 lidar-imu 外参，为仿真兜底
            extrinsic_T: [ 0.0, 0.0, -0.0103 ]   # 取自 LIO-SAM
            extrinsic_R: [ 1., 0., 0.,
                           0., 1., 0.,
                           0., 0., 1.]

        publish:
            path_en:  false
            scan_publish_en:  true            # 输出 /cloud_registered 供 RViz
            dense_publish_en: true
            scan_bodyframe_pub_en: true

        pcd_save:
            pcd_save_en: false                # 阶段 1 不存图
            interval: -1
```

- [ ] **Step 2: 自检（无构建）**

确认文件存在且缩进为空格（YAML 不允许 Tab）。Run:
```bash
python -c "import yaml,sys; yaml.safe_load(open('src/FAST_LIO/config/gazebo_velodyne.yaml')); print('YAML OK')"
```
Expected: `YAML OK`。

> 本任务先不单独提交，与 Task 3 的构建验证一并提交（见 Task 3 Step 3）。

---

### Task 3: 构建 FAST-LIO，验证桩包满足依赖（WSL）

**Files:** 无新增（仅构建既有 `src/FAST_LIO` + Task 1/2 产物）。

- [ ] **Step 1: 构建 fast_lio**

Run（工作区根目录）:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fast_lio
```
Expected: `Finished <<< fast_lio`，**无 livox 相关报错**（证明桩包满足了 `find_package(livox_ros_driver2)` 与编译期类型）。

- [ ] **Step 2: 确认配置已安装**

Run:
```bash
ls install/fast_lio/share/fast_lio/config/gazebo_velodyne.yaml
```
Expected: 路径存在（FAST-LIO 的 `install(DIRECTORY config ...)` 已把它装进 share）。

- [ ] **Step 3: 提交配置文件**

```bash
git add src/FAST_LIO/config/gazebo_velodyne.yaml
git commit -m "feat: 新增 FAST-LIO Gazebo 仿真配置 gazebo_velodyne.yaml(velodyne16/话题/外参/旁路)"
```
> 注：`src/FAST_LIO` 在主仓库中为未跟踪目录（同 LIO-SAM）；若该路径被忽略导致 `git add` 无效，跳过本步，改由 Task 4 的补丁作为该配置的唯一版本载体。

---

### Task 4: 生成 FAST-LIO 配置补丁到 `src/fast-lio2-patch/`

**Files:**
- Create: `src/fast-lio2-patch/01-add-gazebo-velodyne-config.patch`

- [ ] **Step 1: 生成原始 diff**

Run:
```bash
cd src/FAST_LIO
git add -N config/gazebo_velodyne.yaml
git diff -- config/gazebo_velodyne.yaml > /tmp/01-body.patch
git reset -- config/gazebo_velodyne.yaml
cd ../..
```
Expected: `/tmp/01-body.patch` 含 `diff --git a/config/gazebo_velodyne.yaml ...` 的新增内容。

- [ ] **Step 2: 写带注释头的补丁文件**

创建 `src/fast-lio2-patch/01-add-gazebo-velodyne-config.patch`，内容为「注释头 + Step 1 的 diff 正文」：
```
# Purpose: 为 Gazebo 仿真新增 FAST-LIO 配置 gazebo_velodyne.yaml
#          (velodyne 16线 / 仿真话题 points_raw + /imu_plugin/out / lidar-imu 外参 / 旁路纯里程计)
# Base:    FAST_LIO @ a4743b0 (ROS2 branch)
# Apply:   cd src/FAST_LIO && git apply ../fast-lio2-patch/01-add-gazebo-velodyne-config.patch
# Depends: 需配合工作区内 livox_ros_driver2 消息桩包方可编译
#
<在此粘贴 /tmp/01-body.patch 的完整内容>
```

- [ ] **Step 3: 校验补丁可应用**

Run（先临时移走真实文件，验证补丁能重建它）:
```bash
mv src/FAST_LIO/config/gazebo_velodyne.yaml /tmp/keep.yaml
cd src/FAST_LIO && git apply --check ../fast-lio2-patch/01-add-gazebo-velodyne-config.patch && echo "APPLY OK"; cd ../..
mv /tmp/keep.yaml src/FAST_LIO/config/gazebo_velodyne.yaml
```
Expected: `APPLY OK`（注释头不影响 `git apply`）。

- [ ] **Step 4: 提交补丁**

```bash
git add src/fast-lio2-patch/01-add-gazebo-velodyne-config.patch
git commit -m "chore: 沉淀 FAST-LIO gazebo_velodyne 配置为补丁(带注释头)"
```

---

### Task 5: 运行时验证（WSL + Gazebo + RViz，人工检查）

**Files:** 无。这是阶段 1 的验收门，按 spec §2 定性判据。

- [ ] **Step 1: 起仿真（终端 A）**

Run:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py
```
Expected: Gazebo 起、机器人加载；弹出 teleop 的 xterm 窗口。

- [ ] **Step 2: 确认话题在发布**

Run（终端 B）:
```bash
source install/setup.bash
ros2 topic hz points_raw
ros2 topic hz /imu_plugin/out
```
Expected: `points_raw` ≈ 10 Hz，`/imu_plugin/out` ≈ 200 Hz。若话题名不符，以 `ros2 topic list` 实测为准并回填到配置（Task 2）。

- [ ] **Step 3: 保持机器人静止几秒后，起 FAST-LIO（终端 C）**

Run:
```bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
```
Expected: FAST-LIO 节点运行，终端打印 IMU 初始化完成；RViz 自动打开（fastlio.rviz）。

- [ ] **Step 4: RViz 检查（fixed frame = `camera_init`）**

操作：RViz 左上 Fixed Frame 设为 `camera_init`；若无显示项，Add → By topic 添加 `/cloud_registered`(PointCloud2) 与 `/Odometry`(Odometry)。
Expected（静止时）：点云稳定成形、不乱飞；`/Odometry` 基本不动。

- [ ] **Step 5: 遥控走一圈，按定性判据验收**

操作：切到 teleop 的 xterm 窗口，用键盘驱动机器人走一圈并回到起点附近。
Expected（对照 spec §2 验收表）：
- 地图清晰：`/cloud_registered` 墙/物体单层、不重影、不发散；
- 轨迹形状对：`/Odometry` 直线直、转弯对、绕圈回起点附近；
- 漂移缓慢有界：长时间缓慢漂移可接受，**无突跳/发散**。
- **判据不是**估计/真实坐标长期完全重合（那是阶段 2 GICP 的目标）。

- [ ] **Step 6: 记录结果**

若通过：阶段 1 完成。若不通过，按 spec §8 风险表定位（首查抖动→IMU：观察 `/imu_plugin/out` 是否抖动剧烈；必要时调高配置里的 `gyr_cov/acc_cov` 重跑 Task 3）。

---

## Self-Review（作者自查）

- **Spec 覆盖**：§3 旁路（Task 5 用 camera_init 原生 frame）✓；§5 参数（Task 2 配置）✓；§6.1 配置项 ✓；§6.3 livox 桩包（Task 1）✓；§4 补丁规范（Task 4 分文件+注释头+`src/fast-lio2-patch/`）✓；§2 定性验收（Task 5）✓；§6.2 启动方式 ✓。
- **不在范围**：先验地图/GICP/`map→odom`/接入 TF 树——本计划均未涉及，符合阶段 1 边界。
- **类型一致**：包名 `livox_ros_driver2`（Task 1）= FAST-LIO `find_package`/`<depend>` 名称一致；消息字段（Task 1）覆盖 `avia_handler` 实际访问的 `point_num/points[].{x,y,z,reflectivity,tag,line,offset_time}` 与 `header`。
- **占位符**：无 TBD/TODO；所有文件内容与命令均为完整实体。
- **已知前置**：构建/运行需 WSL ROS2 Humble 环境；Task 5 RViz 判断为人工。
