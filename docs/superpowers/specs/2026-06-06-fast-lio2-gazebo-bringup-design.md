# 设计文档：FAST-LIO2 接入 Gazebo 仿真（阶段 1 · 纯里程计）

- 日期：2026-06-06
- 状态：待用户评审
- 范围：仅**阶段 1**（让干净的 upstream FAST-LIO2 在本仓库 Gazebo 仿真里跑通纯 LIO 里程计）

---

## 1. 背景与总体路线

最终目标是 **FAST-LIO2 + GICP 在 LIO-SAM 先验地图中重定位**。整体分三个**运行时叠加、但开发/验证解耦**的阶段：

1. **阶段 1（本文）**：FAST-LIO2 适配 Gazebo，跑通纯里程计。
2. **阶段 2**：外挂一个轻量 ROS2 **GICP 定位节点**，手动给初值，输出 `map→odom` 漂移校正（A 模式：纯定位/位姿跟踪）。
3. **阶段 3（可选）**：在 GICP 前加全局配准前端，实现自动全局重定位（B 模式）。

关键定性结论（已确认）：
- 本仓库的 `FAST_LIO` 是 **FAST-LIO2**（含 `ikd-Tree` + `IKFoM` + `use-ikfom`）。
- 原版 FAST-LIO2 **无"加载先验地图并定位"功能**，该能力由阶段 2/3 以**纯新增节点**方式实现，**不回改**阶段 1 产物（算法层零返工，最多 TF 接线层微调）。

## 2. 阶段 1 目标与验收

**目标**：upstream FAST-LIO2 消费本仿真的雷达 + IMU 话题，稳定输出纯 LIO 里程计。

**本阶段是 SLAM（边建图边定位），不加载任何先验地图**：FAST-LIO 从空 ikd-tree 起步、实时自建地图并在其中定位，参照物是自建地图、无外部锚点，故**缓慢漂移属正常**。加载 LIO-SAM 先验 PCD、消除漂移是**阶段 2（GICP）**的事。

**验收标准（定性）**——机器人在 Gazebo 遥控走一圈，RViz（fixed frame=`camera_init`）中：

| 判据 | 期望 | 含义 |
|---|---|---|
| 地图清晰度（首要） | `/cloud_registered` 墙/物体单层、不重影、不发散 | 配准/去畸变健康 |
| 轨迹形状 | `/Odometry` 直线直、转弯对、**绕圈回到起点附近** | 无系统性错误 |
| 短中期贴合 | 估计位姿短期跟真实运动基本一致 | 局部精度 OK |
| 漂移有界 | 长时间**缓慢**漂移可接受，**不可突跳/发散** | 纯 LIO 的正常表现 |

- 另需：`colcon build` 通过；一条 launch 起节点 + RViz。
- **不做定量真值对比**（不加 p3d 真值插件）；定量对比留到阶段 2 用于证明 GICP 拉回漂移。
- **判据不是"估计坐标与真实坐标长期完全重合"**——那是阶段 2 的目标，不可提前要求到阶段 1。

**不在范围内**：先验地图加载、GICP、`map→odom`、接入机器人 TF 树（均属阶段 2+）。

## 3. 架构决策：旁路独立运行（方案一）

FAST-LIO 作为**独立节点**运行，使用其**原生 frame**（`camera_init` = 里程计/世界系、`body` = 本体系），订阅仿真话题、自建里程计。

- **不改硬编码 frame、不接入机器人 TF 树**：FAST-LIO 发布自己的 TF 孤岛 `camera_init → body`，与机器人本身的树（`odom → base_footprint → …`）互不相连、零冲突。
- **理由**：阶段 1 只验证"FAST-LIO 在这套仿真数据上能否跑通"，旁路最干净、最快见效。接入统一 REP-105 树（`map→odom→base_footprint`）需要改硬编码 frame + 协调 TF 发布权，留到真正需要时（阶段 2 GICP 产出的正是标准 `map→odom`，届时一并处理）。
- FAST-LIO **不依赖机器人 TF 树**即可运行：lidar↔imu 外参通过 config 的 `extrinsic_T/R` 提供，不读 TF。

> 参照系：现状 LIO-SAM 配置见 `docs/sim-dataflow-lio-sam.md`。其中 §9-7 已指出：换 FAST-LIO 跑时，`odom→base_footprint` 无发布者——旁路方案下用 `camera_init→body` 孤岛规避，不受影响。

## 4. 交付方式：patch 补丁模式（分文件 + 注释头）

比照 LIO-SAM，FAST-LIO 的**全部改动以补丁形式交付**，保持 `src/FAST_LIO/` upstream 仓库**干净**（基准 commit `a4743b0`）。补丁规范：

- **按改动目的拆成独立补丁**，不把多个无关改动揉进一个文件。
- **每个补丁开头用注释写明改动目的**（一两句说明这个补丁解决什么）。
- **统一放到 `src/fast-lio2-patch/` 文件夹**（LIO-SAM 的补丁相应归到 `src/lio-sam-patch/`；现有的单文件 `src/lio-sam.patch` 后续迁入该文件夹）。
- 每个补丁是对基准 commit 的 `git diff`，可复现、可对照、无冲突。

阶段 1 预计产生的补丁：
- `src/fast-lio2-patch/01-add-gazebo-velodyne-config.patch`：新增 `config/gazebo_velodyne.yaml`（阶段 1 对 FAST_LIO 的**唯一**改动）。

> livox 依赖改为新增桩包 `src/livox_ros_driver2/`（一等公民包，**不走补丁**）；见 §6.3。

## 5. 仿真侧关键参数（来自已验证的 LIO-SAM 配置）

| 项 | 本仿真实际值 | 来源 |
|---|---|---|
| 雷达话题 | `points_raw` | robot.sdf velodyne 插件 |
| IMU 话题 | `/imu_plugin/out` | robot.sdf imu 插件（`<topicName>imu_raw</topicName>` 不生效） |
| 雷达线数 | **16**（VLP-16） | robot.sdf vertical samples=16 |
| 雷达频率 | 10 Hz | robot.sdf update_rate=10 |
| 点字段 | x/y/z/intensity/ring，**无逐点 time** | velodyne 插件 |
| lidar↔imu 外参 | 旋转单位阵、平移 `[0,0,-0.0103]` | LIO-SAM params（插件已对齐两者） |
| 点云 frame_id | `velodyne` | 插件 frameName |
| IMU frame_id | `base_link` | 插件 frameName |

## 6. 具体改动清单

### 6.1 新建 `config/gazebo_velodyne.yaml`（FAST_LIO 内，经补丁交付）

基于 FAST-LIO 自带 `velodyne.yaml` 修改，关键项：

| 参数 | 值 | 相对默认的改动 / 说明 |
|---|---|---|
| `common.lid_topic` | `points_raw` | 改（默认 `/velodyne_points`）；运行时 `ros2 topic list` 复核 |
| `common.imu_topic` | `/imu_plugin/out` | 改（默认 `/imu/data`） |
| `preprocess.lidar_type` | `2` | Velodyne |
| `preprocess.scan_line` | **16** | 改（默认 32）⚠️ |
| `preprocess.scan_rate` | `10` | 决定缺时间戳时方位角去畸变的重建尺度，务必对 |
| `preprocess.timestamp_unit` | `2` | 缺 time 字段时走回退分支，本项实际不起效，保留默认 |
| `preprocess.blind` | `1.0` | 改（默认 2.0），对齐仿真最小距离 |
| `mapping.extrinsic_T` | `[0,0,-0.0103]` | 取自 LIO-SAM；量级 1cm |
| `mapping.extrinsic_R` | 单位阵 | 插件已对齐 lidar/imu |
| `mapping.extrinsic_est_en` | `true` | 在线微调外参，为仿真兜底 |
| `mapping.acc_cov`/`gyr_cov` | 默认 0.1 | 若抖动致里程计抖/漂，调高以更信 LiDAR |
| `publish.scan_publish_en` | `true` | 输出 `/cloud_registered` 供 RViz |
| `pcd_save.pcd_save_en` | `false` | 阶段 1 不需存图 |

### 6.2 启动方式

复用 FAST-LIO 自带 `mapping.launch.py`：
```bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
```
（可选）后续提供一个固定上述参数的薄 wrapper launch；阶段 1 非必需。

### 6.3 解决 livox 依赖（编译阻塞）——采用方案 A′：消息桩包

FAST-LIO ROS2 分支在**编译期无条件引用** `livox_ros_driver2::msg::CustomMsg`——雷达选择虽是运行期 `if (lidar_type==AVIA)`，但该消息**类型**在 `laserMapping.cpp` 与 `preprocess.{h,cpp}` 中被无条件 include 并编译（C++ 中被引用的类型即使运行期不执行也要编译），故缺 `livox_ros_driver2` 包无法 `colcon build`。该包**不可 apt 安装**，官方需先编译 Livox-SDK2。

**决策：方案 A′（消息桩包），FAST-LIO 源码与 CMake 全部零改动。**

- 新增一个工作区一等公民包，**包名必须为 `livox_ros_driver2`**，仅提供与官方**字段一致**的 `CustomMsg.msg` + `CustomPoint.msg`（照抄官方，保证 `avia_handler` 字段访问能编译）。
- colcon 按包名先构建它；FAST-LIO 现有的 `find_package(livox_ros_driver2)` 与 `<depend>` 自动解析到它即可编译。
- 运行期 `lidar_type=2`（velodyne）永不进入 livox 分支，桩类型从不实例化，**无需任何 Livox 驱动/SDK**。

理由：velodyne-only 永久场景下，桩包让 FAST-LIO 保持原样（跟随上游最省心）、风险最低，是该场景的 ROS 社区惯用法（messages-only 接口包）。

> 该桩包是一等公民新包（与 `robot_control` 等并列），**不走补丁**；补丁仅用于对 upstream 的修改。

### 6.4 可选调优（不在阶段 1 必做）
- `IMU_Processing.hpp` 的 `MAX_INI_COUNT` 10→20：静止零偏估计更准。列为调优项，若初始化不稳再做（并入补丁）。

## 7. 运行与验证流程

1. 起仿真：`ros2 launch robot_gazebo robot_sim.launch.py`
2. `ros2 topic list` 确认 `points_raw`、`/imu_plugin/out` 在发布
3. **保持机器人静止几秒**（IMU 初始化估零偏/重力的前提）
4. 起 FAST-LIO（§6.2）
5. RViz（fixed frame=`camera_init`）看 `/cloud_registered` + `/Odometry`
6. 遥控走一圈，按 §2 验收标准判断

## 8. 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| 模型高频抖动 → IMU 脏 | 最大风险；里程计抖/漂 | 源头压抖动（接触/求解器）；必要时调高 `gyr_cov/acc_cov` |
| `scan_rate` 与实际不符 | 去畸变尺度错 | 确认仿真雷达 10 Hz；`ros2 topic hz /points_raw` 实测 |
| 外参符号约定差异 | ~1–2cm 偏差 | 量级小且开 `extrinsic_est_en`，可忽略 |
| 话题名不符 | 收不到数据 | 运行时 `ros2 topic list` 实测为准 |
| 启动时未静止 | 零偏/重力估歪 | 起步前静止数秒 |
| livox 依赖缺失 | 无法编译 | 见 §6.3 |

## 9. 与后续阶段的衔接

- 阶段 1 产出 `/Odometry` + `/cloud_registered`（`camera_init` 系）。
- 阶段 2 的 GICP 节点**纯订阅**这两个话题 + 先验地图，输出 `map→camera_init`(或标准 `map→odom`)，**不回改本阶段**。
- 唯一可能的后续调整是 TF frame 命名/接线（若届时要并入 REP-105 统一树），属管道层小改，非算法返工。

---

*事实来源：`src/FAST_LIO/`（CMakeLists.txt、package.xml、src/laserMapping.cpp、src/preprocess.cpp、src/IMU_Processing.hpp、config/velodyne.yaml、launch/mapping.launch.py）、`docs/sim-dataflow-lio-sam.md`、`src/lio-sam.patch`。基准 commit a4743b0。*
