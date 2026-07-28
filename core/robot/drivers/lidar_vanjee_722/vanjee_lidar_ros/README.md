# Vanjee LiDAR ROS 2 driver

本包通过只读 `vendor/vanjee_driver` 发布：

- `/points_raw` (`sensor_msgs/msg/PointCloud2`)
- `/imu/data` (`sensor_msgs/msg/Imu`)

默认设备是无后缀、32 线 `vanjee_722`：

- 雷达：`192.168.2.86`
- 主机：`192.168.2.88`
- 主机 MSOP：`3001`
- 雷达端口：`3333`

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
