# Gazebo Harmonic 仿真模块

`robot_gz_bringup` 加载工厂世界、生成机器人描述、spawn 机器人、启动 `gz_ros2_control`、桥接 `/clock`/LiDAR/IMU，并运行 `lidar_pointcloud_adapter`。`robot_gz.launch.py` 是 `system_bringup` 的内部 include，需要 runtime compiler 提供 controller、adapter、几何、传感器和时钟参数；它不是零参数完整栈入口。

系统级运行流见[系统架构](../../../docs/architecture.md)，正式入口和配置选择见 [System bringup](../../bringup/system_bringup/README.md)。

## 运行主机边界

当前 `slam` WSL 只负责 CPU build/test，不安装 Gazebo、`ros_gz` 或 `gz_ros2_control`。本包在该环境只执行 launch/config/xacro 静态测试；这些测试不证明动态仿真通过。

GPU-capable Ubuntu 22.04 主机使用 ROS 2 Humble 与 Gazebo Sim Harmonic。该非默认组合应安装 Harmonic 专用 ROS 包，而不是 Humble 默认的 Fortress `ros-gz`：

```bash
sudo apt install \
  gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```

Humble apt 的 `gz_ros2_control` 与当前 Harmonic 组合不匹配。本项目从 Humble 分支源码构建并显式选择 Harmonic：

```bash
mkdir -p ~/res2_ws/src
cd ~/res2_ws/src
git clone -b humble https://github.com/ros-controls/gz_ros2_control.git
export GZ_VERSION=harmonic
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y \
  --skip-keys="ros_gz_bridge ros_gz_sim"
cd ~/res2_ws
colcon build --symlink-install
source install/setup.bash
```

该外部工作区提供 `libgz_ros2_control-system.so` 和仿真 controller manager；每个仿真终端都要 source 它。

## 工厂资产

`worlds/factory.sdf` 是当前 Harmonic 工厂世界。其 `model://` 视觉资源位于仓库外，默认目录为 `~/LiDAR-SLAM-Simulation/models/factory_model`；路径变化时修改 `bringup.yaml` 的仿真资源配置。运行时把模型和纹理目录追加到 `GZ_SIM_RESOURCE_PATH`。

`core/simulation/spike/` 仅保留底层传感器诊断资产，不属于正式运行或验收路径。

## 接口合同

- Gazebo 原始点云为 `/lidar/points`；适配后为 `/points_raw`，frame=`velodyne`，字段为 `x/y/z/intensity/ring/time`。
- IMU 为 `/imu/data`，frame=`imu_link`；LiDAR 与 IMU mount 分别来自 Profile。
- 控制器输入为 `TwistStamped /cmd_vel`；controller manager 由 URDF 的 `gz_ros2_control` 插件提供，不启动独立 `ros2_control_node`。
- 完整 bringup 顺序为关节状态 discovery/settling、共享传感器契约闸门、SLAM；功能等待使用 ROS 时钟。

runtime compiler 生成 controller、adapter 和 sensor gate 配置，并把 sim Profile 的车体、轮系、传感器 mount 和扫描事实传给本模块。

## 构建、运行与验收

CPU-only WSL 的静态测试：

```bash
cd core
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_gz_bringup system_bringup
source install/setup.bash
colcon test --packages-select robot_description lidar_pointcloud_adapter robot_gz_bringup system_bringup
colcon test-result --all --verbose
```

动态主机将 `bringup.yaml` 设为 `platform: sim` 和所需 mode，然后通过正式入口启动。验收观察 `/clock`、`/joint_states`、`/points_raw`、`/imu/data`、controller 状态和 LiDAR/IMU TF；两个 controller 必须 active，传感器通过当前 Profile 闸门，完整栈只有 `cmd_vel_gate` 发布 `/cmd_vel`。

## 排错

- 找不到 `gz_ros2_control-system`：确认已 source 外部 `res2_ws`，且源码分支与 ROS/Gazebo 组合一致。
- `model://` 无法解析：检查 `factory_model`、纹理文件和 `GZ_SIM_RESOURCE_PATH`。
- 控制器存在但车不动：确认 controller active，并从 Web/Nav2 经 gate 发送 `TwistStamped`；独立底层诊断需要持续发布。
- 传感器闸门不放行：检查 frame、字段、点数、频率和时间戳，不得用 launch 参数绕过 Profile。
- RTF 很低：关闭重复 Gazebo/RViz，确认 GPU 加速并清理孤儿进程。
