# Changelog

All notable changes to the CAN Driver 8030D SDK.

## [1.0.0] — 2024-07-14

### 新增 (Added)
- ROS 2 launch 文件 (`can_driver_8030.launch.py`)，支持一键启动
- YAML 参数配置文件 (`config/can_driver_params.yaml`)
- Jetson 一键安装脚本 (`scripts/setup_jetson.sh`)
- PC 开发依赖安装脚本 (`scripts/install_deps.sh`)
- 完整的 README 文档
- CAN 协议参考文档 (`doc/CAN_PROTOCOL.md`)
- 硬件接线说明 (`doc/HARDWARE_SETUP.md`)
- MIT 许可证

### 修改 (Changed)
- 包版本号从 0.0.0 升级至 1.0.0
- 节点名从 `reap_height_capture` 修改为 `can_driver_8030`
- 许可证从 `TODO` 变更为 `MIT`

### 优化 (Improved)
- `control_speed()` 中的 hex 转换从字符串操作重构为直接位运算（减少约 60 行代码）
- `/current_voltage` 和 `/current_weight` 话题类型从 `Int8` 改为 `Int16`，修复溢出风险
- CAN 设备打开失败时不再调用 `exit(1)` 直接退出，改为优雅处理并记录错误日志
- 清理了三个不再使用的辅助函数 (`hex2dec`, `DecIntToHexStr`, `DecStrToHexStr`)

### 修复 (Fixed)
- 修正了一些代码注释中的笔误

### 文档 (Documentation)
- 新增中英文双语 README，包含完整的安装、使用、API 参考和故障排除指南
- 删除旧的 `使用方法.txt`，内容已整合到 README
