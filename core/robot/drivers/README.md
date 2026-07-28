# 真实设备驱动

本目录存放机器人实机使用的厂商 ROS 驱动和薄适配包。每种设备用一个目录
聚合，避免测试工具或厂商包散落在仓库根目录：

```text
drivers/
├── chassis_8030d/
│   └── can_driver_8030D_sdk/
└── lidar_<model>/
    ├── <vendor_driver_package>/
    └── <project_adapter_package>/   # 仅在确有需要时添加
```

约定：

- 厂商包尽量保持原样；项目修改放在独立适配包或明确记录的补丁中。
- 原始 tar/zip 仅本地归档，不与解压目录重复提交。
- 正式网页控制位于 `core/bringup/robot_web_ui/`，通过 `cmd_vel_gate` 走统一
  `/cmd_vel` 控制器路径，不放在厂商驱动目录。
- 正式运行时，同一个串口、CAN 或 USB 设备只能有一个进程独占。
- 雷达驱动应向上层稳定提供 `/points_raw` 和 IMU 契约；若厂商输出不同，
  由项目适配包负责 remap、字段和 frame 规范化。
