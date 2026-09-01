# System bringup

`system_bringup` 编译本次运行配置、执行启动前一致性检查，并按所选 platform/mode 编排全栈。正式入口是：

```bash
ros2 launch system_bringup bringup.launch.py
```

系统级数据流、TF 与控制所有权见[系统架构](../../../docs/architecture.md)。本文只说明本包的配置、编排、闸门、诊断和限制。

## 运行合同

`bringup.launch.py` 从安装位置映射到工作区唯一的源码 `config/bringup.yaml`，读取一次并调用一次 runtime compiler。platform、mode、`use_sim_time`、几何和生成配置路径随后全部来自同一 manifest；正式入口不接受绕过编译器的临时覆盖。

编译器把所选 Profile、`config/templates/` 下的完整原生 YAML 和 runtime-only 输入写入唯一 `/tmp/system_bringup-runtime-*` 目录。每次运行生成 controllers、Web UI、Nav2、FAST-LIO、LIO-SAM、GICP、sensor gate 和当前 sensor backend 的八份 YAML，以及独立 `effective_profile.generated.yaml`。source/install 不被改写，生成文件不进入 Git。

`run_runtime_consistency()` 在创建节点前检查 manifest、生成产物、effective report、输入选择和 ROS 实际加载的 source/install runtime 文件。失败会中止且不创建节点。精确节点数、launch 表达式和源码拓扑只由静态测试验证，生产闸门不解析 launch AST。

## 模式编排

| platform | mode | 底层 | 上层 |
|---|---|---|---|
| `sim` | `mapping` | `robot_gz_bringup` | LIO-SAM |
| `sim` | `navigation` | `robot_gz_bringup` | FAST-LIO → GICP → Nav2 |
| `real` | `mapping` | 8030D + Vanjee 722 | LIO-SAM |
| `real` | `navigation` | 8030D + Vanjee 722 | FAST-LIO → GICP → Nav2 |

`slam_stack.launch.py` 在 navigation 模式启动 FAST-LIO 与 GICP，并提前提供 RViz；GICP 首次 accepted、`/localization` 出现后才启动 Nav2。mapping 模式只启动 LIO-SAM。`slam_stack` 永久拥有 `body -> base_footprint` bridge，`robot_navigation` 不发布该 TF。

## 控制和 Web

完整 bringup 中 Nav2 发布 `/cmd_vel_auto`，Web 发布 `/cmd_vel_manual`，只有 `cmd_vel_gate` 发布 `/cmd_vel`。Web 人工接管只切换 gate 接受的速度源，不取消现有 Nav2 goal；恢复 automatic 后 Nav2 可以继续输出。

浏览器断连且仍处于 manual 模式时，所选源超过 Profile 的命令超时没有新消息，gate 持续发布零速。Web 不会使能或失能硬件，不能替代物理急停或断电。HTTP 外部接口见 [Robot Web UI](../robot_web_ui/README.md)。

## 传感器契约闸门

sim adapter 和 real Vanjee 都向共享 `sensor_contract_gate` 提供 `/points_raw`、`/imu/data`、`velodyne`、`imu_link` 和 `x/y/z/intensity/ring/time`。闸门按当前 Profile 连续检查点云形状与频率、IMU 频率、frame 和消息时间戳新鲜度；任一条件失败会重置稳定窗口，超时会中止上层启动。

仿真的 graph-only ready gate 只负责 `/joint_states` discovery。topic 出现后的 settling 和 sensor gate 使用各自节点的 ROS 时钟，因此暂停仿真、低 RTF 或 `/clock` 冻结时保持等待，时钟恢复后继续。

独立诊断闸门时必须生成本次专用运行目录，并加载编译器刚生成的完整参数文件：

```bash
cd core
runtime_dir="$(mktemp -d /tmp/system_bringup-runtime-XXXXXX)"
trap 'rm -rf -- "$runtime_dir"' EXIT
ros2 run system_bringup compile_runtime_configs \
  --bringup-config "$PWD/bringup/system_bringup/config/bringup.yaml" \
  --output-dir "$runtime_dir"
ros2 run system_bringup sensor_contract_gate \
  --ros-args --params-file "$runtime_dir/sensor_gate.generated.yaml"
```

不要硬编码临时目录、跨次复用生成物或用 launch 参数覆盖 Profile 契约。

## 修改配置

- `config/bringup.yaml`：platform、mode、GUI/RViz、world、spawn、地图资源和 settling。
- `config/profiles/{sim,real}.yaml`：车体、轮系、LiDAR/IMU mount、运动限制、传感器和后端选择。
- `config/templates/*.yaml`：各模块完整原生参数；adapter/Vanjee template 按 backend 选择。

Profile、`bringup.yaml` 和 source templates 由 runtime compiler 从源码读取，修改后不需要重建即可生成本次配置；template 变化仍需在 packaging/static acceptance 前重建安装副本。ROS 实际从 install 加载的 launch 或 Python runtime 变化后必须重建。真机几何的当前值与派生变换以 `real.yaml` 和本次 `effective_profile.generated.yaml` 为准，不在文档复制数值表。

## 构建与测试

按[开发指南](../../../docs/development.md#构建工作区)准备工作区。聚焦回归：

```bash
python3 -m pytest core/bringup/system_bringup/test -q
cd core
colcon build --packages-up-to system_bringup
source install/setup.bash
colcon test --packages-select system_bringup
colcon test-result --all --verbose
```

跨模块 Profile、template、manifest 或正式编排变化需要按[测试指南](../../../docs/testing.md#工作区回归)运行工作区回归。真机动态验收按该指南的证据规则和本 README 的当前合同定义任务范围。

## 限制与排错

- generated 配置缺失或陈旧：检查 runtime compiler、manifest、输入快照和实际加载的 source/install runtime 文件，再重建受影响包。
- sensor gate 不放行：检查点云 frame、字段、形状、频率，以及 IMU frame、频率和时间戳；不得放宽临时参数绕过 Profile。
- `xacro` 缺失：安装 ROS 2 Humble 对应包；CMake build 成功不能证明运行时依赖存在。
- 真机 mapping/navigation 静态链已接入，但动态移动、最终外参和物理安全链需要现场验收。
- GICP 首次有效配准和 `/initialpose` 基准帧语义仍是定位边界，下发目标前必须确认 registered cloud 与 prior map 贴合。
