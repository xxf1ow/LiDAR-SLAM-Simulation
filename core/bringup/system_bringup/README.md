# core/bringup/system_bringup — 全模块启动 + 启动前一致性闸门

把"跨模块魔法值一致性检查"焊成**全模块启动的强制前置闸门**:每次启动先跑检查,不一致即中止、**不拉任何节点**;通过后错峰拉起全栈。

## 设计要点
- **无命令行参数 + 集中配置**:可变参数(mode/gui/rviz/world/spawn/先验图路径/settling)集中在本模块 `config/bringup.yaml`,`sim/real.launch.py` 运行时用 `find_repo_root()` 读**源码** config(不经 install),**改 config 不用 rebuild system_bringup**。
- **闸门**:`consistency_check.run()` 同步先跑,失败 `raise`(launch 直接报错、零节点)。无逃生口。
- **共享上层** `slam_stack.launch.py`:`mode=navigation` 错峰起 fast_lio→gicp→nav2;`mode=mapping` 起 lio_sam(互斥)。
- **一致性检查**(`consistency_check.py`,纯 Python、本机可跑):几何 G1–G5(footprint / 轮参 / weld 常量 / 共位外参 / 限速 ≤ 底盘)、雷达 L1–L4(线数 / 水平 / 频率 / 盲区)。契约类(帧/话题)不纳入。权威源 = `robot_macro.urdf.xacro`。

## 本机自查(无需 ROS/构建)
```bash
pip install pyyaml
python -m pytest core/bringup/system_bringup/test -q
# 或仅跑检查:
cd core/bringup/system_bringup && python -c "from system_bringup import consistency_check as c; import sys; sys.exit(c.main(['--repo-root','../../..']))"
```

## 本机:构建 + 测试 + 运行
前提:`core/` 在工作区;各下游包可建(robot_gz_bringup/fast_lio/gicp_localization/robot_navigation/lio_sam);先验图在 `~/result/GlobalMap.pcd`、2D 图在 `~/result/factory_map.yaml`(`pcd_to_occupancy` 生成)。

```bash
cd core
colcon build --packages-select system_bringup
source install/setup.bash
colcon test --packages-select system_bringup && colcon test-result --verbose   # 一致性全过

# 仿真 + 导航(切 mapping 改 config/bringup.yaml 的 mode 字段,不用 rebuild)。注意:不带任何 arg:=
ros2 launch system_bringup sim.launch.py
#   源码根自动检测(从 launch 文件上溯 core/bringup/system_bringup 或 .git),无需配置。
```

## 验收判据(PASS)
1. `colcon build` / `colcon test` 全绿;本机 `pytest core/bringup/system_bringup/test` 通过。
2. `ros2 launch system_bringup sim.launch.py`(`config/bringup.yaml` 的 `mode: navigation`):闸门通过 → 错峰起 robot_gz + fast_lio + gicp + nav2,全链可导航(沿用 5e 验收)。
3. 改 `config/bringup.yaml` 的 `mode: mapping`(**不用 rebuild**)再起:闸门通过 → robot_gz + lio_sam,可建图存先验图。
4. **闸门有效性**:临时把 `robot_controllers.yaml` 轮径改 0.10 → `ros2 launch system_bringup sim.launch.py` 报"一致性闸门未通过 + [G2] wheel_radius ..."且**无任何节点启动**;改回即恢复。
5. `weld_z` 不再是裸 `-0.556`,由 `navigation.launch.py` 几何常量算出;`tf2_echo map base_footprint` z≈-0.56(与 5e 一致)。
6. `real.launch.py` 存在、结构完整、文档标注"骨架待实现",不要求运行。

## 改参数去哪
- 启动相关(模式 / 世界 / spawn / 先验图路径 / settling):**`config/bringup.yaml`** —— `sim/real.launch.py` 运行时读**源码** config,**改它不用 rebuild system_bringup**。源码根由闸门自动检测。
- 调参(Nav2 vx_max 等、GICP fitness 等):仍在各自模块 yaml(本设计不集中化它们;它们各自单一 home)。
- 几何/雷达魔法值:改**权威源**(xacro / patch),`consistency_check` 守住别处不漂。

> 为什么 config 不进 `setup.py data_files`(不 install):ament_python 的 launch/config 是 data_files 拷贝到 install、改了要 rebuild。launch 用 `find_repo_root()` 读**源码** config 绕开 install,才能改 config 不 rebuild。见 `consistency_check.load_bringup_config`。

## 已知边界
- 契约类(帧/话题)不入测试;真机驱动未实现;lio-sam `savePCDDirectory` 用 `/result/loam/`(存图以 `save_map` 服务 `destination` 为准,见 mapping 模块 `save_map.sh`)。
- G3 守住 navigation.launch.py 的几何常量 == xacro,但不解析 weld_z 计算式本身(式子是带注释的单行)。
