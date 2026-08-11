# Gazebo Harmonic 仿真模块

`robot_gz_bringup` 负责加载工厂世界、生成机器人描述、spawn 机器人、启动
`gz_ros2_control`、桥接 `/clock`/LiDAR/IMU，并运行 `lidar_pointcloud_adapter`。

正式入口是 `system_bringup`。`robot_gz.launch.py` 需要 runtime compiler 传入 controller、
adapter、几何、传感器和时钟参数，是内部 include，不是零参数完整栈入口。

## 当前 WSL 边界

当前 `slam` WSL 只负责 CPU build/test，不安装 Gazebo、`ros_gz` 或 `gz_ros2_control`，也不
复制工厂模型和启动仿真。这里的 `robot_gz_bringup` 测试是 launch/config/xacro 静态测试；
通过这些测试不等于完成动态仿真验收。

## 仿真运行主机依赖

以下只适用于将来具备足够 CPU/GPU 资源的仿真运行主机，不在当前 WSL 执行。Ubuntu 22.04、
ROS 2 Humble 与 Gazebo Sim Harmonic 的非默认组合应按 Gazebo 官方 Humble + Harmonic
安装说明配置，使用 Harmonic 专用包，而不是 Humble 默认的 Fortress `ros-gz`：

```bash
sudo apt install \
  gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
```

Humble 的 `gz_ros2_control` apt 包与当前 Harmonic 组合不匹配。本项目使用源码构建的
Humble 分支，并显式选择 Harmonic：

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

在每个运行终端 source `~/res2_ws/install/setup.bash`。该工作区提供
`libgz_ros2_control-system.so` 和仿真内的 controller manager；删除它会使仿真控制器
无法加载。

## 工厂资产

`worlds/factory.sdf` 是已经完成 Classic 到 Harmonic 转换和性能优化的最终文件，不需要
再次生成。其 `model://` 视觉资源位于仓库外，默认路径为：

```text
~/LiDAR-SLAM-Simulation/models/factory_model
```

路径不同时修改 `bringup.yaml` 的仿真资源配置。运行时会把模型目录及纹理目录追加到
`GZ_SIM_RESOURCE_PATH`。

## 构建与测试

当前 CPU-only WSL 使用以下静态验证命令，不需要 `res2_ws`：

```bash
cd core
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_gz_bringup system_bringup
source install/setup.bash
colcon test --packages-select \
  robot_description lidar_pointcloud_adapter robot_gz_bringup system_bringup
colcon test-result --all --verbose
```

## 启动

以下步骤只在另行准备的 GPU-capable 仿真运行主机执行，不在当前 WSL 执行。

把 `core/bringup/system_bringup/config/bringup.yaml` 设置为：

```yaml
platform: sim
mode: navigation  # 或 mapping
```

然后：

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

runtime compiler 会生成 controller、LiDAR adapter 和 sensor gate 配置，并把 sim Profile
中的车体、车轮、LiDAR、IMU 和传感器扫描事实传给本模块。

## 传感器与控制契约

- Gazebo 原始点云：`/lidar/points`。
- 适配后点云：`/points_raw`，frame=`velodyne`，字段
  `x/y/z/intensity/ring/time`。
- IMU：`/imu/data`，frame=`imu_link`。
- LiDAR 与 IMU mount 分别来自 Profile；数值相同也不表示共用一份外参。
- 控制器输入：`/cmd_vel`，消息类型为 `TwistStamped`。
- controller manager 由 URDF 中的 `gz_ros2_control` 插件提供，不启动独立
  `ros2_control_node`。

完整 bringup 的顺序是关节状态 discovery/settling、shared sensor contract gate、SLAM。
这些功能等待使用 ROS 时钟；仿真暂停或低 RTF 时会等待，时钟恢复后继续。

## 验收

```bash
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /points_raw
ros2 topic hz /imu/data
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_link velodyne
ros2 run tf2_ros tf2_echo base_link imu_link
```

应满足：

- `joint_state_broadcaster` 和 `base_controller` 为 active。
- `/points_raw`、`/imu/data` 通过当前 Profile 的 sensor gate。
- TF 中 LiDAR 与 IMU 各有唯一父边。
- navigation 模式最终启动 FAST-LIO、GICP 和 Nav2；mapping 模式启动 LIO-SAM。
- 完整 bringup 中只有 `cmd_vel_gate` 发布 `/cmd_vel`。

## 排错

- `xacro not found`：安装 `ros-humble-xacro`，并确认 source ROS 环境。
- 找不到 `gz_ros2_control-system`：source `~/res2_ws/install/setup.bash`，确认源码工作区
  与当前 ROS/Gazebo 版本一致。
- `model://...` 无法解析：检查 `factory_model` 路径、模型目录和纹理文件。
- 控制器存在但车不动：确认 controller active，且完整栈通过 Web/Nav2 进入
  `cmd_vel_gate`；独立底层诊断必须持续发送 `TwistStamped`。
- 传感器 gate 不放行：检查点云 frame/fields/形状/频率以及 IMU frame/频率，不要通过
  放宽 launch 参数绕过 Profile 契约。
- RTF 很低：确认 WSLg/GPU 加速可用，关闭重复 Gazebo/RViz 实例并清理孤儿进程。
- WSL 报 FIFO 实时调度权限警告：当前测试环境允许；真实部署需配置实时权限。

`core/simulation/spike/` 保留底层诊断资产，但不属于当前正式运行或验收路径。
