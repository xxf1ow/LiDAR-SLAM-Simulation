# Profile 配置迁移进度

## 最终架构

正式运行只有 `ros2 launch system_bringup bringup.launch.py`。所选 Profile、共享完整 native
templates 与 runtime-only 输入共同编译为 generated YAML；manifest 记录其绝对路径，formal
launch 只消费 manifest。每次运行在唯一 `/tmp/system_bringup-runtime-*` 目录生成 controllers、
web_ui、nav2、fast_lio、lio_sam、gicp、sensor_gate 和恰好一个平台传感器后端的八份模块 YAML；
`effective_profile.generated.yaml` 是独立的完成报告。生成文件只存在于 `/tmp`，绝不进入 Git。

## 配置所有权与明确边界

- sim 和 real 是两份完整、同 schema 的 Profile，维护平台事实、几何、传感器、后端和共享限制。
- `system_bringup/config/templates/` 是共享 native 模板的唯一来源；Profile 仅覆盖编译器明确
  负责的平台事实，不在 acceptance 中固定可调算法值。
- `map_artifacts.lio_sam_work_dir`、`map_artifacts.prior_pcd` 和 `map_artifacts.nav2_map` 是
  runtime-only 输入，分别供 LIO-SAM 工作目录、GICP 先验图和 Nav2 地图覆盖使用。
- 未跟踪的上游源码默认值不重写；正式运行由 manifest 引用的 generated 配置在运行时覆盖。
- Windows 是权威编辑 checkout；测试只在 WSL 执行，验收依赖最新 WSL acceptance report。

## 阶段状态

### [x] 1–5. Profile、控制、传感器与 FAST-LIO

完成 Profile 基础 schema、控制器和 Web UI 配置生成、传感器契约 gate、FAST-LIO 生成配置及
sim/real 正式入口。安全、动态标定和真机动态运行不因此完成。

### [x] 6A. 生成能力完整化

完成 GICP、LIO-SAM、Nav2 的共享模板、全量生成、manifest 和 effective report；四种
platform/mode 组合生成同构公共产物。

### [x] 6B. 消费者切换与旧配置退役

formal bringup 与包级 launch 切换为 manifest 指定的 generated YAML，运行时地图输入成为必传，
旧 selector、重复地图路径和旧 YAML/patch 配置段已退役。

### [x] 6C-1. Legacy checker/CLI 退役

生产 consistency checker 仅检查 manifest、generated 产物、effective report 和运行时新鲜度，
不再维护旧 generator、旧 YAML/patch 解析或生产 launch AST 拓扑。

### [x] 6C-2. 配置所有权收口

生成器、manifest、formal launch 和模块边界已收口到同一 Profile + shared template 合同；
上游 defaults 保持未跟踪并在运行时被覆盖。

### [x] 6C-3. 文档与最终自动化验收

架构与模块文档、最终自动化验收和状态审计已完成。以下仅记录本轮实际观测，不作为未来固定门槛：

- source pytest 为 826 passed、0 skipped；explicit matrix 为 8 passed、0 skipped，覆盖 4/4 generation 与 4/4 consistency。
- full workspace 首次并行构建因 host OOM 失败；唯一环境调度调整为 sequential 后 16 packages build 通过。default-policy test 执行 11 packages，`colcon test-result` 为 926 tests、0 errors、0 failures、0 skipped。
- metadata parse 与 stale-document scan 通过；独立预提交审查最终为 Ready Yes（0 Critical、0 Important，最后一个 Minor 已修复）。

### [ ] 6D. 真机 mapping/navigation 统一验收

在 6C-3 完成后，按 tracked 6D runbook 分别执行有人在场的真机 mapping/navigation 静态与
运行观察；不下发导航目标，不把动态调参或长期稳定性验收混入该 gate。

## 当前已知边界与后续产品工作

首次有效 GICP 配准 readiness 与 `/initialpose` 的帧语义尚未完成；LiDAR/IMU 动态标定、物理
安全链、碰撞与失联保护、lifecycle、全局 diagnostics 和真实动态操作仍需产品级工作与验收。
这些边界不通过增加 Profile 字段或在 acceptance 中冻结可调值来掩盖。

## 下一步

按 tracked `docs/acceptance/profile-migration-real-acceptance.md` 执行 6D 真机 mapping/navigation 统一验收。
