# LiDAR-SLAM-Simulation

面向差速移动机器人的 ROS 2 全栈：在 Gazebo Sim Harmonic 中仿真，也可切换到
ZL-8030D 底盘与 Vanjee 722 雷达真机。定位主线为 FAST-LIO2 里程计、GICP
先验图定位与 Nav2 自主导航；LIO-SAM 负责制作先验地图。

## 仓库结构

`core/` 是独立的 colcon 工作区，不使用额外的 `src/` 层。

| 模块 | 职责 | 文档 |
|---|---|---|
| `core/robot/` | URDF、ros2_control、命令仲裁和真实设备驱动 | [Robot](core/robot/README.md) |
| `core/simulation/` | Gazebo Harmonic 世界、传感器桥接和点云适配 | [Simulation](core/simulation/robot_gz_bringup/README.md) |
| `core/mapping/` | LIO-SAM 建图与地图保存 | [Mapping](core/mapping/README.md) |
| `core/localization/` | FAST-LIO2 里程计和 GICP 先验图定位 | [Localization](core/localization/README.md) |
| `core/navigation/` | Nav2 规划、控制和代价地图 | [Navigation](core/navigation/README.md) |
| `core/bringup/` | Profile 编译、运行时一致性检查和全栈启动 | [System bringup](core/bringup/system_bringup/README.md) |

正式运行入口只有：

```bash
ros2 launch system_bringup bringup.launch.py
```

它读取 `core/bringup/system_bringup/config/bringup.yaml`，选择 `sim|real` 和
`mapping|navigation`，把 Profile、共享完整 native templates 和 runtime-only 输入编译为
generated YAML，再写入 manifest；一致性检查通过后才按 manifest 启动节点。每次编译产生
八份模块 YAML，以及单独的 `effective_profile.generated.yaml` 完成报告。

## 运行时拓扑

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ── slam_stack bridge ──> base_footprint
base_footprint ── URDF ──> base_link ──> velodyne / imu_link / wheels

Nav2 ──> /cmd_vel_auto ─┐
                        ├──> cmd_vel_gate ──> /cmd_vel ──> diff_drive_controller
Web  ──> /cmd_vel_manual┘
```

建图模式运行 LIO-SAM，定位/导航模式运行 FAST-LIO、GICP 和 Nav2；两种 SLAM
模式互斥。完整 bringup 中 `cmd_vel_gate` 是唯一的 `/cmd_vel` 发布者。

## 环境准备

支持 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Sim Harmonic。当前验证环境是 Windows
主机上的 WSL2 Ubuntu 22.04，发行版名为 `slam`。以下命令假定 ROS 2 Humble 已安装，
并已执行 `source /opt/ros/humble/setup.bash`，使 `ROS_DISTRO=humble` 生效。

```bash
sudo apt update

# URDF/Xacro 展开及 robot_description、robot_bringup 测试
sudo apt install ros-${ROS_DISTRO}-xacro

# 用于 robot_hardware
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers

# PPA 方式安装 GTSAM 依赖库，用于 LIO-SAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev

# PCL 依赖库，用于 FAST-LIO2
sudo apt install ros-${ROS_DISTRO}-perception-pcl

# OpenMP 依赖库，用于 GICP（Eigen/PCL 已安装）
sudo apt install libomp-dev

# 导航模块 Nav2
sudo apt install ros-${ROS_DISTRO}-navigation2

# Open3D 用于把 PCD 3D 点云转换为 Nav2 2D 代价地图
sudo apt install python3-pip
python3 -m pip install open3d
```

`ros-humble-xacro` 是 URDF 展开和真机链路测试的运行时依赖。缺少它不会阻止
CMake 编译，但会使 `robot_description` 和 `robot_bringup` 测试失败。

Gazebo Harmonic 使用源码构建的 `gz_ros2_control` Humble 分支；当前 apt 版与
Harmonic 不匹配。一次性安装步骤见
[仿真模块文档](core/simulation/robot_gz_bringup/README.md)。

FAST-LIO、LIO-SAM 和 small_gicp 的上游源码不入库。它们应按 mapping/localization
文档的固定提交克隆到所属模块，并应用仓库跟踪的补丁。

## 构建与测试

所有命令从 `core/` 执行：

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --all --verbose
```

`core/colcon_defaults.yaml` 会在默认测试中跳过上游 clone 和 aarch64 厂商包；这些包
需要时单独测试。验收以最新 WSL acceptance report 的零 errors、failures 和 unexpected
skips 为准。

## 选择并启动运行模式

编辑 `core/bringup/system_bringup/config/bringup.yaml` 顶部两项：

```yaml
platform: real      # sim | real
mode: navigation    # navigation | mapping
```

导航默认使用 Web 页面且不启动 RViz；调试时可把同一文件中的
`slam_stack.rviz` 临时改为 `true`。

Profile 和该选择文件从源码位置读取，修改后不需要重建；新增或修改安装到 package share
的 launch/template 文件仍需要重新 `colcon build`。

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

- `sim + mapping`：Gazebo、传感器契约 gate、LIO-SAM 和 Web 手动驾驶。
- `sim + navigation`：Gazebo、FAST-LIO、GICP、Nav2 和 Web 接管。
- `real + mapping`：8030D、Vanjee、真机传感器 gate 和 LIO-SAM。
- `real + navigation`：8030D、Vanjee、FAST-LIO、GICP 和 Nav2。

手机访问 `http://<主机IP>:8080`。Web 控制不能替代物理急停或断电手段。

## 地图产物

建图完成且 LIO-SAM 仍在运行时：

```bash
cd core
bash mapping/save_map.sh
```

脚本把地图保存到 `~/result/loam/`，并生成供定位使用的
`~/result/GlobalMap.pcd`。不要把 LIO-SAM 的保存目标直接设为 `/result`，其保存实现会
先删除并重建目标目录。

## 已知边界

- GICP 是局部配准，大位姿偏差和工厂 90° 对称假解仍需正确 `/initialpose` 引导。
- 仿真点云的逐点 `time` 是适配器合成值；Gazebo 快照本身没有真实帧内畸变。
- 真机静态链路已经接入，动态行驶、最终外参和安全保护仍需有人看护验收。
- WSL 中 controller manager 可能提示无法启用 FIFO 实时调度；当前测试允许该警告，
  真实部署仍应按 ros2_control 指南配置实时权限。

## 项目状态与路线图

当前算法主线已经端到端打通，处于“可运行，待完成真机动态验收和产品级加固”的阶段。
本节只说明当前还缺哪些模块或能力；实现细节仍以各模块 README 为准。

### 一、产品愿景

给定目标位姿，机器人能够在工厂先验图中自主规划并安全到达；沿途识别和避让障碍；遇到
定位丢失、传感器掉线、碰撞风险或控制失联时安全减速或停车；运行状态可观测、故障可
排查、数据可回放；同一套代码和配置可切换到真机交付。

产品级完成标准包括：

- **安全**：独立避撞、急停、速度限制和禁行区。
- **可靠**：失效告警、安全停车、有序启动和停机。
- **可运维**：健康指标、日志、录包和故障定位。
- **可落地**：真机动态 mapping/navigation 通过现场验收。
- **可交付**：任务接口、自动回归和可部署运行环境。

### 二、当前模块进度

| 模块 | 仿真侧 | 真机侧 | 当前仍缺 |
|---|---|---|---|
| `robot/` | gz/mock/real URDF、ros2_control 和控制 gate 已完成 | 8030D、Web、轮速反馈和静态链路已接入 | 物理急停状态、速度平滑、失联/故障安全策略和完整动态验收 |
| `simulation/` | 工厂世界、LiDAR/IMU、桥接、点云适配和控制器已完成 | 不适用 | 动态障碍、退化场景和故障注入场景 |
| `mapping/` | LIO-SAM 配置集中化、建图、保存 PCD 和二维地图转换已完成 | Vanjee real 配置和正式入口已接入 | 最终 LiDAR/IMU 外参、真机动态建图与地图质量验收 |
| `localization/` | FAST-LIO 与 GICP 配置集中化；FAST-LIO + GICP 先验图定位已完成 | real 参数和静态链路已接入 | 首次有效配准 readiness、`/initialpose` 帧语义、动态标定、退化检测和全局重定位 |
| `navigation/` | Nav2 配置集中化，Smac Hybrid-A*、MPPI、STVL 和控制仲裁可运行 | real Profile 与静态 Nav2 链已接入 | 真机动态导航、近场盲区防撞、运动障碍、waypoint 和恢复策略加固 |
| `bringup/` | sim mapping/navigation 统一入口、runtime gate 和八份模块生成产物已完成 | real mapping/navigation 四组合已接线 | lifecycle、优雅停机和全局 diagnostics |

**当前结论**：不存在尚未创建的主链功能目录；缺口集中在安全模块、定位增强、动态场景、
真机动态闭环和运维交付能力。

### 三、待完成事项

#### A. 安全与失效保护

- [ ] 增加 `collision_monitor`，独立于规划器执行减速和停车。
- [ ] 增加 `velocity_smoother` 或等价模块，统一速度、加速度和 jerk 限制。
- [ ] 接入物理急停状态和软件停机链，明确自动/人工/急停优先级。
- [ ] 增加禁行区、限速区和安全距离验收。
- [ ] 对定位丢失、传感器掉线、控制链中断建立强制停车策略。

#### B. 系统可靠性与可运维性

- [ ] 用 lifecycle/readiness 替代固定启动延迟，支持可控启动和优雅停机。
- [ ] 建立全局 diagnostics：定位、传感器、TF、控制器和地图状态。
- [ ] 持续记录 RTF、CPU/内存、话题频率和 TF 延迟。
- [ ] 标准化 rosbag2 录制、回放和故障复现流程。
- [ ] 提供运行状态、有效配置和错误原因的统一查询入口。

#### C. 导航能力补全

- [ ] 修复 FAST-LIO 近场盲区与 STVL 清除共同产生的贴障假缺口。
- [ ] 增加行人/移动设备场景并完成运动障碍避让验收。
- [ ] 启用 waypoint follower；保留 `navigate_through_poses` 基础能力。
- [ ] 加固 spin/backup/wait 等恢复行为，确保不突破统一运动限制。
- [ ] 验证堵路、窄道、U 形死路和目标不可达时的安全退出。

> Nav2 lifecycle Pause 会取消当前目标，Resume 不会续上原目标，这是现有机制，不作为 bug。

#### D. 定位鲁棒性

- [ ] 让 bringup 等待首次达到 fitness 阈值的有效 GICP 配准。
- [ ] 把 RViz `/initialpose` 的 `base_footprint` 语义正确转换为 GICP `body` 初值。
- [ ] 增加长走廊、空旷区和旋转对称场景的退化检测与告警。
- [ ] 输出定位协方差或等价不确定度，供导航和安全决策使用。
- [ ] 增加大偏差和 90° 对称场景下的全局重定位能力。

#### E. 真机落地

- [ ] 获取并固化 LiDAR、内置 IMU、车体和轮系的权威外参与方向定义。
- [ ] 完成有人看护的真机 mapping 动态验收和地图质量评估。
- [ ] 完成有人看护的真机 navigation 动态验收和路径跟踪评估。
- [ ] 验证急停、失联停车、传感器掉线、定位丢失和长时间运行。
- [ ] 将后续实机标定或动态调参确认的参数写回 real Profile。

#### F. 交付与集成

- [ ] 提供任务下发、状态查询、日志导出和 HMI 接口。
- [ ] 建立自动 build/test 和端到端场景回归。
- [ ] 提供 headless 仿真运行方式，并在主链稳定后评估容器化。
- [ ] 定义发布、部署、配置升级和回滚流程。

#### G. 开发支撑

- [ ] 建立长走廊、空旷大厅和对称回环退化测试场景。
- [ ] 建立动态障碍物场景和安全停车场景。
- [ ] 自动计算 ATE/RPE、地图一致性、路径完成率和停车距离。
- [ ] 支持 IMU 偏置、雷达丢帧、网络抖动、时钟冻结和 TF 断链故障注入。

### 四、里程碑

- **M1 产品级最低线**：完成 A 与 B 的核心安全、停机和诊断能力。
- **M2 能力补全**：完成 C 与 D 的动态避障和定位鲁棒性。
- **M3 真机闭环**：完成 E 的 mapping/navigation 动态验收。
- **M4 交付闭环**：完成 F，并以 G 的自动验收作为发布门槛。
