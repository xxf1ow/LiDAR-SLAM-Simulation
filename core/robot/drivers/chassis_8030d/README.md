# ZL-8030D 底盘驱动与统一控制链

本设备组位于 `core/robot/drivers/chassis_8030d/`，包含厂商 ROS 2 包：

| 目录 | ROS 包名 | 职责 |
|---|---|---|
| `can_driver_8030D_sdk/` | `can_driver` | 厂商预编译节点，负责 ZLG USBCAN2 与 ZL-8030D 通信 |

正式网页入口在 `core/bringup/robot_web_ui/`，通过 `cmd_vel_gate` 和
`diff_drive_controller` 复用仿真/真机控制器路径。厂商输入仍只由 8030D
ros2_control 话题适配器 `robot_hardware` 发布。

## 平台与硬件

- Ubuntu 22.04、ROS 2 Humble
- ARM aarch64（Jetson Orin/Nano + JetPack 6）
- ZLG USBCAN2、ZL-8030D 和正确的 24–48 V 供电

厂商节点和 `libcontrolcan.so` 都是 aarch64 预编译文件，不能在 x86_64
主机上运行。CAN 接线、终端电阻和 udev 配置见
`can_driver_8030D_sdk/doc/HARDWARE_SETUP.md`。

原始交付包只在本地 `vendor_archives/` 留档并被 Git 忽略；仓库跟踪的是
解压后的可构建 ROS 包。v1.0.0 原始包 SHA256：

```text
DB445AFD1742E85C817D958285181D4867902A46532087B9A71141E44927F9CD
```

## 在项目工作区构建

```bash
cd ~/LiDAR-SLAM-Simulation/core
source /opt/ros/humble/setup.bash
colcon build --packages-select can_driver robot_hardware robot_bringup
source install/setup.bash
```

安装厂商动态库：

```bash
sudo cp robot/drivers/chassis_8030d/can_driver_8030D_sdk/lib/libcontrolcan.so /usr/local/lib/
sudo ldconfig
```

## 正式 ros2_control 底盘链

正式链为：

```text
/cmd_vel → diff_drive_controller → robot_hardware → can_driver → 8030D → /current_speed → wheel state/odom
```

真机几何和轮参由 `system_bringup/config/bringup.yaml:real_geometry` 统一生成；
`real_chassis.launch.py` 是内部 include，直接无参数启动会明确失败。正式启动命令：

```bash
cd ~/LiDAR-SLAM-Simulation/core
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

该 launch 将厂商节点的 `auto_enable_on_start` 覆盖为 `false`，由
`robot_hardware` 独占厂商输入话题 `/motor_speed` 和 `/driver`。启动后
检查唯一话题所有权及控制器状态：

```bash
ros2 topic info /motor_speed -v
ros2 topic info /driver -v
ros2 control list_controllers
```

首次运动测试必须架空驱动轮，并保证操作人员能够立即使用物理急停或切断
驱动电源。确认话题所有权和控制器状态后，可发布低速前进命令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.12}}}"
```

另开终端观察命令、实测反馈、轮状态和轮式里程计：

```bash
ros2 topic echo /motor_speed
ros2 topic echo /current_speed
ros2 topic echo /joint_states
ros2 topic echo /base_controller/odom --field twist.twist
```

停止发布 `/cmd_vel` 后，现有控制器的 0.5 秒命令超时会将轮速命令归零。
这条 `ros2 topic pub` 命令只用于独立启动底层时诊断控制器。完整 bringup
运行时只有 `cmd_vel_gate` 可以发布 `/cmd_vel`，不要同时直接发布。

## 验证状态

截至 2026-07-27，验收者已在目标环境完成并报告以下三个阶段全部通过：

1. Ubuntu 22.04 / ROS 2 Humble 自动构建与测试；
2. Gazebo Harmonic 回归；
3. Jetson + 厂商二进制 + 实体底盘架空轮实测。

Ubuntu Humble 自动验收使用以下命令：

```bash
cd ~/LiDAR-SLAM-Simulation/core
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select \
  robot_hardware robot_description robot_bringup robot_gz_bringup \
  --event-handlers console_direct+
colcon test --packages-select \
  robot_hardware robot_description robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

三个阶段的行为与本 Spec 预期一致：正式 real bringup 能启动厂商节点和
ros2_control 底盘，控制命令能到达电机，实测轮速能回到 wheel state/odom，
Gazebo 既有链路无回归。

当前 Windows 开发主机仍不具备 ROS 2/Gazebo/Jetson 运行环境，因此上述动态
结论记录的是验收者在目标环境的实测结果，不是 Windows 主机的独立复跑结果。

## 全栈网页手动控制

用 `system_bringup` 启动完整栈后：

- 手机访问 `http://<机器人或仿真主机IP>:8080`
- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

Web 发布 `/cmd_vel_manual`，经 `cmd_vel_gate` 输出 `/cmd_vel`，再走
`diff_drive_controller → robot_hardware → can_driver`；它不会直接操作
`/motor_speed` 或 `/driver`。接管不会取消已有 Nav2 goal。浏览器断连且仍在
manual 模式时，0.5 秒源超时会停车。

Web 不会失能硬件，也不能证明电机已经失能；首次实机运动仍须架空驱动轮，
准备物理急停或断电，并由现场人员保持可立即操作。
