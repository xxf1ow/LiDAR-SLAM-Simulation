# Robot 模块

`core/robot/` 定义机器人描述、ros2_control、速度入口和真实设备驱动。

| 目录 | 职责 |
|---|---|
| `robot_description/` | URDF/Xacro、碰撞几何、轮系、LiDAR/IMU mount 和 ros2_control 声明 |
| `robot_hardware/` | ZL-8030D 的 ros2_control `SystemInterface` 话题适配器 |
| `robot_bringup/` | mock/real controller manager、robot state publisher 和 controller spawner |
| `cmd_vel_gate/` | automatic/manual 速度源仲裁和超时停车 |
| `drivers/` | ZL-8030D 与 Vanjee 722 厂商驱动 |

## 控制合同

```text
Nav2 ──> /cmd_vel_auto ─┐
                        ├──> cmd_vel_gate ──> /cmd_vel ──> diff_drive_controller
Web  ──> /cmd_vel_manual┘                               │
                                                       v
                                 robot_hardware ──> can_driver ──> 8030D
                                       ^                    │
                                       └── /current_speed ──┘
```

完整 bringup 中 `cmd_vel_gate` 是唯一 `/cmd_vel` 发布者；`robot_hardware` 独占 `/motor_speed` 和 `/driver` 并消费 `/current_speed`。Web 与 Nav2 共享控制器路径，Web 不发布厂商协议话题。manual 源超时会输出零速，但 Web 停车不等同于硬件急停。

## 配置与设备

正式入口由 `system_bringup` 拥有。车体、轮系、LiDAR/IMU mount 和运动限制只在 `core/bringup/system_bringup/config/profiles/{sim,real}.yaml` 维护；runtime compiler 把同一组几何事实传给 URDF、控制器和 Nav2。

`real_chassis.launch.py` 是正式入口的内部 include，需要父级提供完整参数，不提供隐式正式默认值。

- [ZL-8030D 底盘](drivers/chassis_8030d/README.md)：CAN 厂商节点、ros2_control 接入和底盘验收。
- [Vanjee 722](drivers/lidar_vanjee_722/vanjee_lidar_ros/README.md)：标准点云与 IMU 接口。

厂商代码和文档保持原样；项目级适配、配置所有权和运行限制由本模块与 `system_bringup` 维护。

## 验证

```bash
cd core
colcon build --packages-up-to system_bringup
source install/setup.bash
colcon test --packages-select robot_description robot_hardware robot_bringup cmd_vel_gate robot_web_ui
colcon test-result --all --verbose
ros2 control list_controllers
ros2 topic info /cmd_vel -v
```

完整栈应只有一个 `/cmd_vel` 发布者，controller 均 active，轮状态和 `/base_controller/odom` 持续更新。x86 WSL 默认不执行 aarch64 厂商 `can_driver`；兼容真机按驱动 README 单独验收。
