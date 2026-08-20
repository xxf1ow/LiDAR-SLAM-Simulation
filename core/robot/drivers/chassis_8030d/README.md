# ZL-8030D 底盘驱动

本目录包含厂商 ROS 2 包 `can_driver`。它通过 ZLG USBCAN2 与 ZL-8030D 通信；项目侧
`robot_hardware` 把厂商话题接入 ros2_control。

## 平台和硬件

- Ubuntu 22.04、ROS 2 Humble。
- ARM aarch64 Jetson 平台。
- ZLG USBCAN2、ZL-8030D 和正确的 24–48 V 供电。

厂商节点与 `libcontrolcan.so` 是 aarch64 预编译文件，不能在 x86_64 WSL 中执行。CAN
接线、终端电阻和 udev 规则见
`can_driver_8030D_sdk/doc/HARDWARE_SETUP.md`。

原始交付包只在被 Git 忽略的 `vendor_archives/` 本地留档。v1.0.0 原始包 SHA256：

```text
DB445AFD1742E85C817D958285181D4867902A46532087B9A71141E44927F9CD
```

## 安装与构建

```bash
cd core
source /opt/ros/humble/setup.bash
sudo cp robot/drivers/chassis_8030d/can_driver_8030D_sdk/lib/libcontrolcan.so \
  /usr/local/lib/
sudo ldconfig
colcon build --packages-select can_driver robot_hardware robot_bringup
source install/setup.bash
```

## 正式控制链

```text
cmd_vel_gate -> /cmd_vel -> diff_drive_controller -> robot_hardware
             -> /motor_speed + /driver -> can_driver -> 8030D
             <- wheel state/odom <- /current_speed <- can_driver
```

真机几何和轮参来自
`core/bringup/system_bringup/config/profiles/real.yaml`。runtime compiler 生成 controller
配置并把同一组事实传给 URDF。正式启动：

```bash
cd core
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

`robot_hardware` 独占 `/motor_speed` 和 `/driver`。检查：

```bash
ros2 topic info /motor_speed -v
ros2 topic info /driver -v
ros2 topic hz /current_speed
ros2 topic hz /base_controller/odom
ros2 control list_controllers
```

## 独立底盘诊断

只有在架空驱动轮、现场人员能立即物理急停或断电时，才绕过完整控制链进行低速测试。
确认 controller active 和话题所有权后，可持续发送：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.12}}}"
```

停止发布后，命令超时应把轮速归零。完整 bringup 运行时不要执行此命令；此时
`cmd_vel_gate` 必须是唯一 `/cmd_vel` 发布者。

## 验收要求

- controller 正常加载和激活。
- `/motor_speed` 只有 `robot_hardware` 发布，`/driver` 所有权符合设计。
- 实测 `/current_speed` 能回到 wheel state 和 `/base_controller/odom`。
- 停止命令、浏览器断连或 gate 超时后车辆停止。
- 仿真回归和真机架空轮测试分别执行，不能用 WSL 仿真结果替代实体安全验收。

Web 手动控制发布 `/cmd_vel_manual`，经 gate 和同一 ros2_control 链驱动车轮。Web 停车不
代表电机失能，也不能替代物理急停。
