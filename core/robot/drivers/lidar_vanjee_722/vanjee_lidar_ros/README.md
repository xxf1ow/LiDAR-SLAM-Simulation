# Vanjee LiDAR ROS 2 driver

本包通过只读 `vendor/vanjee_driver` 发布：

- `/points_raw` (`sensor_msgs/msg/PointCloud2`)
- `/imu/data` (`sensor_msgs/msg/Imu`)

默认设备是无后缀、32 线 `vanjee_722`：

- 雷达：`192.168.2.86`
- 主机：`192.168.2.88`
- 主机 MSOP：`3001`
- 雷达端口：`3333`

以下命令只用于驱动独立诊断；正式运行由 `system_bringup` 生成完整 Vanjee 参数并启动：

```bash
cd core
colcon build --packages-select vanjee_lidar_ros
source install/setup.bash
ros2 launch vanjee_lidar_ros vanjee_lidar.launch.py
```

启动和 IMU 初始化期间保持雷达静止。型号可以通过配置文件中的
`lidar_type` 切换，修改后重启节点；本包不支持运行中热切换。完整厂商 ROS
SDK/msg 只存在于仓库根目录被忽略的 `.vanjee_lidar_sdk/`、
`.vanjee_lidar_msg/`，不属于构建依赖。

在线查询得到的角度和 IMU 标定文件按雷达 IP、型号保存到
`~/result/lidar_calibration/<lidar_address>/`。该目录与 LIO-SAM 地图同属
`~/result/`，但独立于保存地图时会被删除重建的 `~/result/loam/`。

完整栈的上行契约由 shared sensor gate 检查：点云 frame、字段、32×1200 总点数、频率、
IMU frame/频率和消息新鲜度必须与所选 real Profile 一致。
