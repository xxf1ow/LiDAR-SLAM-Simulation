# ZL-8030D 底盘驱动

本目录包含厂商 ROS 2 包 `can_driver`。它通过 ZLG USBCAN2 与 ZL-8030D 通信；项目 `robot_hardware` 把厂商话题接入 ros2_control。

## 平台合同

厂商节点与 `libcontrolcan.so` 是 Ubuntu 22.04/ROS 2 Humble、ARM aarch64 预编译文件，不能在 x86_64 WSL 执行。硬件需要 ZLG USBCAN2、ZL-8030D、正确的 24–48 V 供电、终端电阻和设备权限；接线与 udev 步骤见 [HARDWARE_SETUP](can_driver_8030D_sdk/doc/HARDWARE_SETUP.md)。

原始交付包只保存在忽略的 `vendor_archives/`。v1.0.0 原始包 SHA256 为：

```text
DB445AFD1742E85C817D958285181D4867902A46532087B9A71141E44927F9CD
```

## 安装与正式控制链

```bash
cd core
source /opt/ros/humble/setup.bash
sudo cp robot/drivers/chassis_8030d/can_driver_8030D_sdk/lib/libcontrolcan.so /usr/local/lib/
sudo ldconfig
colcon build --packages-select can_driver robot_hardware robot_bringup
source install/setup.bash
```

```text
cmd_vel_gate -> /cmd_vel -> diff_drive_controller -> robot_hardware
             -> /motor_speed + /driver -> can_driver -> 8030D
             <- wheel state/odom <- /current_speed <- can_driver
```

真机几何和轮参只来自 `core/bringup/system_bringup/config/profiles/real.yaml`。正式运行通过 `system_bringup`；`robot_hardware` 独占 `/motor_speed` 和 `/driver`。

```bash
ros2 topic info /motor_speed -v
ros2 topic info /driver -v
ros2 topic hz /current_speed
ros2 topic hz /base_controller/odom
ros2 control list_controllers
```

## 独立底盘诊断

只有架空驱动轮且现场人员能立即物理急停或断电时，才绕过完整链路低速测试。确认 controller active 和话题所有权后持续发送：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.12}}}"
```

停止发布后，命令超时必须把轮速归零。完整 bringup 运行时禁止执行该命令，因为 `cmd_vel_gate` 必须是唯一 `/cmd_vel` 发布者。

## 验收

- controller 正常加载并 active，厂商话题所有权唯一。
- `/current_speed` 进入 wheel state 和 `/base_controller/odom`。
- 停止命令、manual 源断连或 gate 超时后车辆停止。
- 仿真回归与真机架空轮测试分别执行，静态或 WSL 结果不能替代实体安全验收。

Web 停车只停止命令，不代表电机失能，也不能替代物理急停。
