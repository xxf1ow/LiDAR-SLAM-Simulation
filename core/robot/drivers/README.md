# 真实设备驱动

本目录按设备聚合厂商 ROS 驱动和薄项目适配：

```text
drivers/
├── chassis_8030d/
│   └── can_driver_8030D_sdk/
└── lidar_<model>/
    ├── <vendor_driver_package>/
    └── <project_adapter_package>/
```

- 厂商包尽量保持原样；项目修改使用独立适配包或跟踪 patch。
- 原始 tar/zip 只保存在本地忽略目录，不与解压内容重复提交。
- 正式 Web 控制位于 `core/bringup/robot_web_ui/`，经 `cmd_vel_gate` 使用统一 `/cmd_vel` 路径。
- 一个串口、CAN 或 USB 设备在运行时只能由一个进程独占。
- [Vanjee 722 适配](lidar_vanjee_722/vanjee_lidar_ros/README.md)提供 `/points_raw` 和 `/imu/data`；正式参数由 `system_bringup` 生成，不在驱动目录复制平台事实。
