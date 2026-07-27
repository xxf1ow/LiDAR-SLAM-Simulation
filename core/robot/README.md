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
  厂商 CAN 节点；正式网页入口位于 `core/bringup/robot_web_ui/`。
- 真实雷达（含内置 IMU）到货后放入
  `drivers/lidar_<model>/`，保持厂商包与项目适配包分离。

## 控制链边界

完整栈的控制链为：

```text
Nav2 → /cmd_vel_auto ─┐
                      ├→ cmd_vel_gate → /cmd_vel → diff_drive_controller
Web  → /cmd_vel_manual┘                    → robot_hardware → can_driver → 8030D
                                                     → /current_speed → wheel state/odom
```

`cmd_vel_gate` 是完整 bringup 中唯一的 `/cmd_vel` 发布者。Web 在仿真和真机
使用同一条控制器路径，不再直接发布厂商输入话题。真机底层由
`robot_hardware` 独占 `/motor_speed` 和 `/driver`，并从 `/current_speed`
取得实测轮速。

单独启动真实 ros2_control 底盘做底层诊断：

```bash
ros2 launch robot_bringup real_chassis.launch.py
```

此 standalone 场景仍可用 `ros2 topic pub /cmd_vel ...` 测试控制器；完整
bringup 运行时不要直接发布 `/cmd_vel`。

手机访问 `http://<机器人或仿真主机IP>:8080`

- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

接管不取消已有 Nav2 goal。浏览器断连且仍处于 manual 模式时，0.5 秒源超时
会停车。Web 不会失能硬件，不能替代物理急停或断电手段。

## 构建

```bash
cd core
colcon build --packages-select \
  robot_description robot_hardware robot_bringup cmd_vel_gate \
  can_driver robot_web_ui system_bringup
```

底盘专用构建、启动和验收步骤见设备组 README。
