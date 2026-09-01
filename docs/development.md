# 开发指南

本文是贡献者从环境准备到本地验证的顺序教程。完成后应能在 CPU-only WSL 中构建和测试工作区；动态 Gazebo 和真机操作使用独立运行主机，并按当前任务定义受控验收范围。

## 前提

- Windows checkout 是权威编辑副本；WSL2 Ubuntu 22.04 发行版 `slam` 是 ROS 2 Humble CPU build/test executor。
- WSL checkout 位于 `/home/lxx/xxsim`，其本地 `windows` remote 指向 Windows 仓库。
- ROS 2 Humble 安装在 `/opt/ros/humble`，系统 Python 为 `/usr/bin/python3` 3.10。
- `core/` 是 colcon 工作区根目录，不创建额外的 `src/` 或仓库 `.venv`。

在接受 WSL 结果前同步并确认两个 checkout 指向同一提交：

```bash
cd /home/lxx/xxsim
test -z "$(git status --porcelain)"
git fetch windows
git merge --ff-only '@{u}'
git rev-parse HEAD
```

未提交的 Windows 修改不能通过该流程进入 WSL。文档-only 修改可先在 Windows 运行文档检查；代码和配置修改必须在同步提交后取得 WSL 构建/测试证据。

## 安装依赖

基础依赖包括 ROS 2 Humble、`xacro`、ros2_control、Nav2、PCL、OpenMP、GTSAM 和用于 PCD 转二维地图的 Open3D。按系统包管理器安装与当前 ROS 发行版匹配的包；`xacro` 是运行时/测试依赖，CMake 构建成功不能证明它存在。

FAST-LIO、LIO-SAM 和 small_gicp 不是仓库内容。按 [Localization](../core/localization/README.md#fast-lio-集成) 与 [Mapping](../core/mapping/README.md#上游集成) 的固定提交和 patch 步骤准备。Gazebo Harmonic 与 Humble 的运行主机依赖见 [Simulation](../core/simulation/robot_gz_bringup/README.md#运行主机边界)；CPU-only WSL 不安装或启动 Gazebo。

Web 资产测试要求可执行的 `node`。当前 WSL 通过 `/home/lxx/.local/bin/node` 使用既有 Windows Node；运行相关测试前执行 `command -v node` 和 `node --version`。缺少 Node 是环境失败，不是允许的 skip。

## 构建工作区

从 `core/` 执行默认 copy-install 构建：

```bash
cd /home/lxx/xxsim/core
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

不要在现有 `build/` 与 `install/` 上切换为 `--symlink-install`。确需改变安装模式时，必须先有意重建两个目录，避免 `ament_cmake_python` 目录和符号链接混用。

日常窄构建使用包边界，例如：

```bash
colcon build --packages-up-to system_bringup
source install/setup.bash
```

各模块的包选择和诊断入口由其 README 维护。构建后按[测试指南](testing.md)选择与改动表面相称的检查。

## 修改运行配置

- `core/bringup/system_bringup/config/bringup.yaml`：platform、mode、GUI/RViz、world、spawn、地图资源和编排选项。
- `core/bringup/system_bringup/config/profiles/{sim,real}.yaml`：平台几何、传感器、后端和跨模块限制。
- `core/bringup/system_bringup/config/templates/*.yaml`：controller、Web UI、Nav2、SLAM、定位和传感器后端的完整原生配置。

Profile、`bringup.yaml` 和 source templates 由 runtime compiler 从源码读取，修改后不需要重建即可生成本次配置；template 变化仍需在 packaging/static acceptance 前重建安装副本。ROS 实际从 install 加载的 launch 或 Python runtime 变化后必须重建对应包。不得通过临时 launch 参数或分散配置复制绕过运行时编译器。

## 文档工作流

文档事实先更新其 owner：系统流写入 `docs/architecture.md`，贡献者命令写入本文件，测试策略写入 `docs/testing.md`，模块契约写入模块 README，持久决策写入 Agent Note。一次性计划、迁移记录、验收脚本、报告和实现进度不进入维护文档；运行证据保存在仓库外。项目文档使用一段一个物理行和相对 Markdown 链接。

安装了项目所用文档 skills 的环境执行：

```bash
python ~/.claude/skills/doc-standards/scripts/validate_project_docs.py
python ~/.claude/skills/agent-notes/scripts/validate_agent_notes.py
git diff --check
```

在 Codex Windows 环境中，同一检查脚本位于 `%USERPROFILE%\.codex\skills\`。`scripts/doc-budgets.manifest.json` 是项目预算权威；提高预算必须在 owning Agent Note 中记录理由。

## 提交前

检查 `git status` 和差异范围，确认没有 build/install/log、地图、bag、运行时 YAML、厂商归档或 Superpowers 产物进入 Git。代码改动在 WSL 完成相应测试后再创建本地提交；任何 push、PR 创建或其他远程变更都需要当前会话中的明确人工批准。
