# 测试指南

本文是项目测试层级、命令归属、选择规则和验收证据的参考。测试应覆盖改动的最窄有效表面，并在跨模块契约变化时逐级扩展。

## 测试层级

| 层级 | 适用范围 | 结果要求 |
|---|---|---|
| 文档检查 | Markdown、文档引用、预算或 Agent Notes | 两个文档 validator 与 `git diff --check` 通过 |
| Python 快速回归 | 单个 Python 包或运行时编译器 | 受影响 pytest 集合全部通过，无意外 skip |
| ROS 包测试 | package manifest、launch、xacro、C++ 或包间配置 | 受影响包 `colcon test` 与 `colcon test-result` 无 errors/failures |
| 工作区回归 | 共享 Profile、模板、正式 bringup 或跨模块契约 | 默认策略下完整 build/test 通过，无意外 skip |
| 动态验收 | Gazebo 行为、真机传感器、定位、导航或控制 | 当前任务定义的观察项、清理和状态恢复全部通过 |

## 文档检查

文档-only 改动执行[开发指南中的文档命令](development.md#文档工作流)。预算清单覆盖项目自有 Markdown；`docs/agent-notes/archived/`、vendor、generated、fixtures 和 snapshots 不参与编辑维护。

## 快速回归

Python 包可以从仓库根目录运行聚焦测试，例如：

```bash
python3 -m pytest core/bringup/system_bringup/test -q
python3 -m pytest core/bringup/robot_web_ui/test -q
```

Web 资产测试依赖 `node`。测试开始前确认 `command -v node` 和 `node --version`；缺少 Node 必须按环境失败处理。

## ROS 包测试

从 `core/` 构建并测试受影响包：

```bash
cd /home/lxx/xxsim/core
source /opt/ros/humble/setup.bash
colcon build --packages-up-to <package>
source install/setup.bash
colcon test --packages-select <package> [<package> ...]
colcon test-result --all --verbose
```

模块 README 维护其推荐包集合。`colcon test-result` 必须为零 errors 和 failures；只有明确记录且由默认策略拥有的 package exclusion 才能接受。

## 工作区回归

跨模块改动执行：

```bash
cd /home/lxx/xxsim/core
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

`core/colcon_defaults.yaml` 是默认 test package exclusion 的权威来源。它跳过上游 clone 和 aarch64-only 厂商包，而不是跳过这些包内部的测试用例；兼容平台上的显式厂商验收仍按对应 README 执行。

当前 WSL 资源不足时可把 build executor 调整为 sequential，但不得因此缩小测试集合。WSL 可能记录无法启用 FIFO 实时调度的警告；当控制器与测试其余部分通过时，该警告是当前环境限制，不是代码失败。

## 动态验收

CPU-only WSL 的 launch/config/xacro 测试不证明 Gazebo 动态行为或真机安全。每次动态验收必须从受影响模块的当前合同出发，明确现场安全条件、正式入口、观察项、停止条件、进程组清理和配置恢复；这些任务特定步骤不作为长期项目文档保留。

动态报告记录时间、主机、分支、精确 HEAD、运行输入快照、观察时长、首个失败、环境限制、清理结果和 `PASS|FAILED|BLOCKED`。报告、普通日志、bag 和生成 YAML 保存在仓库外。

## 证据规则

- 使用正式入口验证全栈；直接包级 launch 只能证明隔离诊断结果。
- 静态节点、topic、TF 或 lifecycle 检查不能替代动态移动和安全验收。
- generated/live 参数比较保留类型和值，且只忽略明确的 runtime-only 输入。
- 任一清理失败、配置未 byte-exact 恢复、残留进程或 tracked tree 污染都会使动态验收失败。
- 环境或基线不满足使用 `BLOCKED`；产品或清理检查失败使用 `FAILED`。
