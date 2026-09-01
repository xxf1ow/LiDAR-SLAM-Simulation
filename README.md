# LiDAR-SLAM-Simulation

LiDAR-SLAM-Simulation 是面向差速移动机器人的 ROS 2 全栈，支持 Gazebo Sim Harmonic 仿真，以及 ZL-8030D 底盘与 Vanjee 722 雷达真机。定位链由 FAST-LIO2、GICP 和 Nav2 组成，LIO-SAM 用于制作先验地图。

## 运行入口

`core/` 是 colcon 工作区，不使用额外的 `src/` 层。正式全栈入口只有：

```bash
cd core
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

启动前在 `core/bringup/system_bringup/config/bringup.yaml` 选择 `platform: sim|real` 和 `mode: mapping|navigation`。Profile 与配置模板的编译、manifest 和启动闸门见[系统架构](docs/architecture.md#运行时配置流)。开发环境和构建步骤见[开发指南](docs/development.md)，测试选择与验收层级见[测试指南](docs/testing.md)。

| 组合 | 主要运行链 |
|---|---|
| `sim + mapping` | Gazebo、传感器契约闸门、LIO-SAM、Web 手动驾驶 |
| `sim + navigation` | Gazebo、FAST-LIO、GICP、Nav2、Web 接管 |
| `real + mapping` | 8030D、Vanjee、传感器契约闸门、LIO-SAM |
| `real + navigation` | 8030D、Vanjee、FAST-LIO、GICP、Nav2 |

手机访问 `http://<主机IP>:8080`。Web 控制不能替代物理急停或断电手段；真机动态测试必须有人看护。

## 模块

| 模块 | 职责 | 文档 |
|---|---|---|
| `core/robot/` | URDF、ros2_control、命令仲裁和真实设备驱动 | [Robot](core/robot/README.md) |
| `core/simulation/` | Gazebo Harmonic 世界、桥接和点云适配 | [Simulation](core/simulation/robot_gz_bringup/README.md) |
| `core/mapping/` | LIO-SAM 建图与地图保存 | [Mapping](core/mapping/README.md) |
| `core/localization/` | FAST-LIO2 里程计和 GICP 先验图定位 | [Localization](core/localization/README.md) |
| `core/navigation/` | Nav2 规划、控制和代价地图 | [Navigation](core/navigation/README.md) |
| `core/bringup/` | Profile 编译、运行时检查、Web UI 和全栈编排 | [System bringup](core/bringup/system_bringup/README.md) |

## 当前边界

- GICP 是局部配准，大位姿偏差和工厂 90° 对称假解需要正确初值；首次有效配准 readiness 和 `/initialpose` 基准帧换算仍需完善。
- 仿真点云的逐点 `time` 是适配器合成值，Gazebo 快照不包含真实帧内畸变。
- 真机静态链路已经接入；动态建图、动态导航、最终外参和物理安全链仍需有人看护验收。
- 独立碰撞保护、失效停车、动态真机验收和产品级运维仍未完成。

## 文档

- [系统架构](docs/architecture.md)：当前组成、运行流、边界和扩展点。
- [开发指南](docs/development.md)：环境准备、日常构建和配置修改。
- [测试指南](docs/testing.md)：测试层级、命令归属和验收规则。
- [地图制作与保存](core/mapping/README.md#建图与保存)：先验 PCD 和 Nav2 二维地图产出。
- `docs/agent-notes/`：架构、流程与产品提案的持久决策记录。
