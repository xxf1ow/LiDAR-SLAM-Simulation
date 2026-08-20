# Robot 模块

`core/robot/` 定义机器人本体、ros2_control 抽象、命令入口和真实设备驱动。

| 目录 | 职责 |
|---|---|
| `robot_description/` | URDF/Xacro、碰撞几何、轮系、LiDAR/IMU mount 和 ros2_control 声明 |
| `robot_hardware/` | ZL-8030D 的 ros2_control `SystemInterface` 话题适配器 |
| `robot_bringup/` | mock/real controller manager、robot state publisher 和 controller spawner |
| `cmd_vel_gate/` | 自动/人工速度源仲裁和超时停车 |
| `drivers/` | ZL-8030D 与 Vanjee 722 厂商驱动 |

## 正式控制链

```text
Nav2 ──> /cmd_vel_auto ─┐
                        ├──> cmd_vel_gate ──> /cmd_vel ──> diff_drive_controller
Web  ──> /cmd_vel_manual┘                               │
                                                       v
                                 robot_hardware ──> can_driver ──> 8030D
                                       ^                    │
                                       └── /current_speed ──┘
```

完整 bringup 中：

- `cmd_vel_gate` 是唯一的 `/cmd_vel` 发布者。
- `robot_hardware` 独占 `/motor_speed` 和 `/driver`，并消费 `/current_speed`。
- Web 与 Nav2 共享同一控制器路径；Web 不直接发布厂商协议话题。
- 浏览器在 manual 模式断连后，命令源超时会输出零速。
- Web 停车不是物理急停，不能替代急停按钮或断电。

## 配置所有权

正式入口是：

```bash
ros2 launch system_bringup bringup.launch.py
```

车体、车轮、LiDAR/IMU mount 和运动限制只维护在：

```text
core/bringup/system_bringup/config/profiles/sim.yaml
core/bringup/system_bringup/config/profiles/real.yaml
```

runtime compiler 生成 controller 配置并把同一份几何事实传给 URDF、控制器和 Nav2。
`real_chassis.launch.py` 是 `system_bringup` 的内部 include；它要求父级提供完整参数，
不应作为带隐式默认值的正式真机入口。

## 真实设备

- [ZL-8030D 底盘](drivers/chassis_8030d/README.md)：CAN 厂商节点和底盘级验收。
- [Vanjee 722 ROS 2 驱动](drivers/lidar_vanjee_722/vanjee_lidar_ros/README.md)：发布
  `/points_raw` 和 `/imu/data`。

厂商包和厂商文档尽量保持原样；项目级适配和运行约束写在本模块或 `system_bringup`。

## 构建与测试

```bash
cd core
source /opt/ros/humble/setup.bash
colcon build --packages-up-to system_bringup
source install/setup.bash
colcon test --packages-select \
  robot_description robot_hardware robot_bringup cmd_vel_gate robot_web_ui
colcon test-result --all --verbose
```

在 x86 WSL 上默认测试会跳过 aarch64 厂商 `can_driver`；真机平台按其 README 单独验收。

## 运行检查

```bash
ros2 control list_controllers
ros2 topic info /cmd_vel -v
ros2 topic hz /joint_states
ros2 topic hz /base_controller/odom
```

完整栈应只有一个 `/cmd_vel` 发布者，两个 controller 均为 active，轮状态和里程计持续更新。
