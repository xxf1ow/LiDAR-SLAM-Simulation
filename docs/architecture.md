# 系统架构

本文是 LiDAR-SLAM-Simulation 当前系统组成、运行流、主要边界和扩展点的权威参考。模块内部参数、诊断命令和限制由各模块 README 维护。

## 系统组成

`core/` 是独立的 ROS 2 Humble colcon 工作区。项目代码按运行职责分为六个模块：

| 模块 | 系统职责 | 详细文档 |
|---|---|---|
| `robot` | 机器人描述、ros2_control、速度源仲裁和真实设备驱动 | [Robot](../core/robot/README.md) |
| `simulation` | Gazebo Harmonic 世界、控制插件、ROS 桥接和点云适配 | [Simulation](../core/simulation/robot_gz_bringup/README.md) |
| `mapping` | LIO-SAM 建图、先验点云和二维地图产出 | [Mapping](../core/mapping/README.md) |
| `localization` | FAST-LIO 连续里程计与 GICP 先验图校正 | [Localization](../core/localization/README.md) |
| `navigation` | Nav2 地图、规划、控制、行为和速度转换 | [Navigation](../core/navigation/README.md) |
| `bringup` | Profile 编译、一致性和传感器闸门、Web UI、全栈编排 | [System bringup](../core/bringup/system_bringup/README.md) |

上游 FAST-LIO、LIO-SAM 和 small_gicp 源码不进入 Git；模块 README 固定其提交，项目修改由跟踪的 patch 交付。ZL-8030D 与 Vanjee 厂商代码位于 `core/robot/drivers/`，项目适配层与厂商实现保持分离。

## 运行时配置流

正式入口 `ros2 launch system_bringup bringup.launch.py` 固定读取源码树中的 `core/bringup/system_bringup/config/bringup.yaml`。该文件选择 `platform: sim|real`、`mode: mapping|navigation`、资源路径和编排选项。

运行时编译器读取本次选择、`config/profiles/{sim,real}.yaml` 中的平台事实、`config/templates/*.yaml` 中的完整原生配置，以及先验地图等 runtime-only 输入。每次运行在唯一的 `/tmp/system_bringup-runtime-*` 目录生成八份模块 YAML 和一份 `effective_profile.generated.yaml` 完成报告；进程内 manifest 记录绝对路径。

正式 launch 只把 manifest 指定的生成配置交给消费者。一致性闸门在创建节点前检查 manifest、生成产物、完成报告、输入选择和实际加载的 source/install 运行文件；失败时不启动节点。源码 Profile 和 template 是配置权威，安装副本只提供打包与静态验收证据。

## 运行模式

底层平台与上层算法由两个正交选择组成：

| 选择 | 底层 | 上层 |
|---|---|---|
| `sim + mapping` | Gazebo、ros2_control、桥接、LiDAR adapter | LIO-SAM |
| `sim + navigation` | Gazebo、ros2_control、桥接、LiDAR adapter | FAST-LIO、GICP、Nav2 |
| `real + mapping` | ZL-8030D、Vanjee 722、ros2_control | LIO-SAM |
| `real + navigation` | ZL-8030D、Vanjee 722、ros2_control | FAST-LIO、GICP、Nav2 |

两种平台都必须先通过共享传感器契约闸门。mapping 与 navigation 的 SLAM 链互斥；直接启动包级 launch 只用于隔离诊断。

## 传感器和坐标流

共享传感器接口固定为 `/points_raw` 与 `/imu/data`，坐标帧为 `velodyne` 与 `imu_link`。点云字段为 `x/y/z/intensity/ring/time`。sim adapter 和 real Vanjee 驱动各自产生该接口，传感器闸门按当前 Profile 检查 frame、字段、点数、频率和时间戳新鲜度。

navigation 坐标链为：

```text
map ── GICP ──> camera_init ── FAST-LIO ──> body ── slam_stack ──> base_footprint ── URDF ──> base_link
```

GICP 首次接受配准后才发布 `/localization`，随后启动 Nav2。FAST-LIO 提供连续局部坐标，GICP 提供 `map -> camera_init` 校正，`slam_stack` 永久拥有 `body -> base_footprint` bridge。

mapping 坐标链为：

```text
map ── static ──> odom ── LIO-SAM ──> base_footprint ── URDF ──> base_link
```

轮式里程计 TF 被关闭，LIO-SAM 在 mapping 模式独占 `odom -> base_footprint`。

## 控制流

```text
Nav2 ──> /cmd_vel_auto ─┐
                        ├──> cmd_vel_gate ──> /cmd_vel ──> diff_drive_controller
Web  ──> /cmd_vel_manual┘                               │
                                                       v
                                 robot_hardware ──> can_driver ──> 8030D
                                       ^                    │
                                       └── /current_speed ──┘
```

完整 bringup 中 `cmd_vel_gate` 是唯一的 `/cmd_vel` 发布者。Web 人工接管只改变 gate 接受的速度源，不取消现有 Nav2 goal。manual 命令超时会输出零速，但不会使硬件失能。

## 时钟和启动边界

功能等待使用节点自身的 ROS 时钟。仿真 graph discovery 可用墙钟限制必需 topic 始终未出现的情况；topic 出现后的 settling、传感器稳定窗口和功能 timeout 使用 ROS 时钟。仿真暂停、低 RTF 或 `/clock` 冻结时保持等待，时钟恢复后继续。

当前 WSL2 环境只承担 CPU build/test 和静态 launch/config/xacro 验证。Gazebo 动态仿真需要另行准备的 GPU-capable 主机，真机动态验收需要现场人员和物理急停。验收分层和证据规则见[测试指南](testing.md)。

## 扩展点

- 新平台在 `config/profiles/` 增加完整平台事实，并提供满足共享传感器和控制接口的底层实现；不得通过分散的 launch 覆盖绕过编译器。
- 新传感器后端提供完整 native template，由运行时编译器选择并写入 manifest；共享算法只消费标准接口。
- 新算法参数归对应 template，跨模块平台事实归 Profile，运行选择和资源路径归 `bringup.yaml`。
- 模块级独立 launch 可以保留诊断入口，但正式编排、TF 所有权和控制出口仍由 `system_bringup` 统一管理。
