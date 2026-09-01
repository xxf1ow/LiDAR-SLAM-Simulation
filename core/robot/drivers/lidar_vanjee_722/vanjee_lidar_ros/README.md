# Vanjee LiDAR ROS 2 adapter

本包通过只读 `vendor/vanjee_driver` 发布 `/points_raw` (`sensor_msgs/msg/PointCloud2`) 和 `/imu/data` (`sensor_msgs/msg/Imu`)。正式运行由 `system_bringup` 生成完整 Vanjee 参数并启动；下列包级 launch 只用于驱动诊断。

## 默认设备

- 型号：无后缀 32 线 `vanjee_722`。
- 雷达地址：`192.168.2.86`；主机地址：`192.168.2.88`。
- 主机 MSOP：`3001`；雷达端口：`3333`。

```bash
cd core
colcon build --packages-select vanjee_lidar_ros
source install/setup.bash
ros2 launch vanjee_lidar_ros vanjee_lidar.launch.py
```

启动与 IMU 初始化期间保持雷达静止。`lidar_type` 只在节点重启后生效，不支持运行时热切换。完整厂商 SDK/msg 位于仓库根的忽略目录 `.vanjee_lidar_sdk/` 和 `.vanjee_lidar_msg/`，不是构建依赖。

在线获取的角度和 IMU 标定按 IP/型号保存在 `~/result/lidar_calibration/<lidar_address>/`。该目录独立于保存地图时会删除重建的 `~/result/loam/`。

## 上行合同

共享 sensor gate 按 real Profile 检查点云 frame、字段、32×1200 点数、频率、IMU frame/频率和消息新鲜度。驱动或固件变化后还要复核逐点时间单位；不得通过修改上层扫描频率掩盖时间语义错误。回归 bag、标定文件和日志保存在仓库外。
