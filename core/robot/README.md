# Robot 模块

`core/robot/` 定义机器人本体、控制抽象、正式启动方式和真实设备驱动。

| 目录 | 职责 |
|---|---|
| `robot_description/` | URDF/Xacro、关节、碰撞几何、雷达/IMU 安装外参和 ros2_control 声明 |
| `robot_hardware/` | 正式的 8030D ros2_control 话题适配器，将轮速命令和实测反馈接入 `SystemInterface` |
| `robot_bringup/` | `robot_state_publisher`、controller manager 和控制器的正式启动入口 |
| `drivers/` | 这台真实机器人的厂商设备驱动及设备级验收工具 |

当前真实设备：

- [`drivers/chassis_8030d/`](drivers/chassis_8030d/README.md)：ZL-8030D
  厂商 CAN 节点和手机网页手动验收工具。
- 真实雷达（含内置 IMU）到货后放入
  `drivers/lidar_<model>/`，保持厂商包与项目适配包分离。

## 控制链边界

正式底盘链为：

```text
/cmd_vel → diff_drive_controller → robot_hardware → can_driver → 8030D → /current_speed → wheel state/odom
```

一条命令启动厂商节点和真实 ros2_control 底盘：

```bash
ros2 launch robot_bringup real_chassis.launch.py
```

正式链运行时，`robot_hardware` 独占厂商输入话题 `/motor_speed` 和
`/driver`，并从 `/current_speed` 取得实测轮速。Web 网页工具是独立的
设备验收入口：

```text
手机网页 → can_driver_web_control → can_driver → USBCAN2 → 8030D
```

Web 网页工具与正式链会发布相同的厂商输入话题，必须互斥运行。当前正式链没有
软件急停，也没有手动/自动 `/cmd_vel` 仲裁；这些安全与控制权能力留待后续
Spec，现阶段不能替代物理急停或断电手段。

## 构建

```bash
cd core
colcon build --packages-select \
  robot_description robot_hardware robot_bringup \
  can_driver can_driver_web_control
```

底盘专用构建、启动和验收步骤见设备组 README。
