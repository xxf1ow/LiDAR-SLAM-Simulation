# Robot 模块

`core/robot/` 定义机器人本体、控制抽象、正式启动方式和真实设备驱动。

| 目录 | 职责 |
|---|---|
| `robot_description/` | URDF/Xacro、关节、碰撞几何、雷达/IMU 安装外参和 ros2_control 声明 |
| `robot_hardware/` | ros2_control `SystemInterface`；当前仍是回环占位实现，尚未接入 8030D |
| `robot_bringup/` | `robot_state_publisher`、controller manager 和控制器的正式启动入口 |
| `drivers/` | 这台真实机器人的厂商设备驱动及设备级验收工具 |

当前真实设备：

- [`drivers/chassis_8030d/`](drivers/chassis_8030d/README.md)：ZL-8030D
  厂商 CAN 节点和手机网页手动验收工具。
- 真实雷达（含内置 IMU）到货后放入
  `drivers/lidar_<model>/`，保持厂商包与项目适配包分离。

## 控制链边界

网页工具只用于设备验收：

```text
手机网页 → can_driver_web_control → can_driver → USBCAN2 → 8030D
```

规划中的正式控制链：

```text
Nav2 → diff_drive_controller → robot_hardware → can_driver → 8030D
```

当前 `robot_hardware` 的 `write()` 只把命令复制到内部状态，`read()` 只做
位置积分，因此 `use_mock_hardware:=false` 还不能用于实车。底盘验收完成后，
再实现 rad/s↔RPM、左右轮顺序、使能/失能和反馈超时处理。

## 构建

```bash
cd core
colcon build --packages-select \
  robot_description robot_hardware robot_bringup \
  can_driver can_driver_web_control
```

底盘专用构建、启动和验收步骤见设备组 README。
