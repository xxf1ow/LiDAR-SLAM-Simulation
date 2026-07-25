# ZL-8030D 底盘驱动与网页手动验收

本设备组位于 `core/robot/drivers/chassis_8030d/`，包含两个彼此独立的
ROS 2 包：

| 目录 | ROS 包名 | 职责 |
|---|---|---|
| `can_driver_8030D_sdk/` | `can_driver` | 厂商预编译节点，负责 ZLG USBCAN2 与 ZL-8030D 通信 |
| `can_driver_web_control/` | `can_driver_web_control` | 手机网页手动验收工具，发布使能和左右轮 RPM |

网页工具只用于架空轮实测，不属于正式控制链，也不会被
`system_bringup` 启动。正式控制链仍由 `diff_drive_controller` 和
`robot_hardware` 承担；在完成适配前，不要将当前占位版
`robot_hardware` 用于实车。

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
colcon build --packages-select can_driver can_driver_web_control
source install/setup.bash
colcon test --packages-select can_driver_web_control
colcon test-result --verbose
```

安装厂商动态库：

```bash
sudo cp robot/drivers/chassis_8030d/can_driver_8030D_sdk/lib/libcontrolcan.so /usr/local/lib/
sudo ldconfig
```

## 复制到独立验收工作区

如需只复制底盘验收所需内容，复制下面两个 ROS 包即可：

```bash
mkdir -p ~/can_test_ws/src
cp -a /path/to/can_driver_8030D_sdk ~/can_test_ws/src/
cp -a /path/to/can_driver_web_control ~/can_test_ws/src/
```

不需要复制 mapping、localization、navigation 或 simulation 模块。

## 启动

首次运行必须架空驱动轮，并准备物理断电或急停手段。

```bash
cd ~/LiDAR-SLAM-Simulation/core
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch can_driver_web_control can_driver_web_test.launch.py
```

运行 `hostname -I` 获取构建机地址，手机与构建机连接同一局域网后打开
`http://<构建机IP>:8080`。页面默认 20 RPM、最大 100 RPM。按住方向
按钮时持续发指令；松手或网络中断超过约 300 ms 后指令归零。

页面中的“已发送使能”只表示命令已发布。厂商 SDK 的 `/driver` 是单向
命令话题，没有使能成功/失败反馈，也没有读取 0x6041 Statusword。
`底盘速度反馈` 表示近期收到了 `/current_speed`，可以证明通信链路大概率
在线，但不能严格证明电机已经使能。

`/motor_speed` 顺序为 `[右轮, 左轮]`：

- 前进：`[+v, +v]`
- 后退：`[-v, -v]`
- 左转：`[+v, -v]`
- 右转：`[-v, +v]`

预编译 SDK 在组装 CAN 帧前会将第一路取反，以补偿两侧电机镜像安装。
所以驱动日志会表现为前后两路异号、左右转两路同号，这不是网页方向错误。

## 验收

1. 启动后确认驱动轮静止，页面能发现 ROS 驱动节点。
2. 确认页面收到真实底盘速度反馈，再发送使能命令。
3. 依次按住前进、后退、左转、右转，核对实际轮向。
4. 每次松手后确认两轮在约 300 ms 内停止。
5. 按住方向时关闭手机 Wi-Fi，确认看门狗停车。
6. 点击“停车 / 失能”，确认零速和失能命令均已发送。
7. 记录反馈顺序、符号、倍率以及断连行为。

同一时间只能有一个组件控制 USBCAN2。运行本工具时不要同时启动未来的
正式 `robot_hardware` 适配链。厂商预编译节点没有可验证的硬件看门狗；
本工具不能替代物理急停，也不能作为生产控制链。
