# core/bringup/system_bringup — 全模块启动 + 启动前一致性闸门

把"跨模块魔法值一致性检查"焊成**全模块启动的强制前置闸门**:每次启动先跑检查,不一致即中止、**不拉任何节点**;通过后错峰拉起全栈。

## 设计要点
- **唯一入口 + 集中配置**:`bringup.launch.py` 读 `config/bringup.yaml` 的 `platform`(sim/real) + `mode`(navigation/mapping) 选底层与模式;可变参数按节点分组(`robot_gz`/`robot_bringup`/`slam_stack.*`)。launch 用 `find_repo_root()` 读**源码** config(不经 install),**改 config 不用 rebuild**。无命令行参数。
- **闸门**:`consistency_check.run()` 同步先跑,失败 `raise`(launch 直接报错、零节点)。无逃生口。
- **共享上层** `slam_stack.launch.py`:`mode=navigation` 错峰起 fast_lio→gicp→nav2;`mode=mapping` 起 lio_sam(互斥)。
- **唯一控制出口**:完整 bringup 中 Nav2 发 `/cmd_vel_auto`、Web 发 `/cmd_vel_manual`，
  只有 `cmd_vel_gate` 发布 `/cmd_vel`；仿真和真机共用这条控制器入口。
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

# 全栈启动。切 platform/mode 改 config/bringup.yaml 顶层两行,不用 rebuild。不带任何 arg:=
ros2 launch system_bringup bringup.launch.py
#   源码根自动检测(从 launch 文件上溯 core/bringup/system_bringup 或 .git),无需配置。
```

## 手机手动控制

手机访问 `http://<机器人或仿真主机IP>:8080`

- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

接管只切换 `cmd_vel_gate` 接受的速度源，不会取消已有 Nav2 goal；恢复自动后
Nav2 可继续输出。浏览器断连且仍处于 manual 模式时，所选源超过 0.5 秒无新命令，
gate 会持续发布零速。Web 不会失能硬件，不能替代物理急停或断电。

只启动 `robot_gz_bringup` 或 `robot_bringup` 做底层诊断时，仍可用
`ros2 topic pub /cmd_vel ...` 直接测试控制器；完整 bringup 运行时不要这样做，
因为 `/cmd_vel` 必须只有 `cmd_vel_gate` 一个发布者。

## config/bringup.yaml 切换矩阵
| platform | mode | 底层 | 上层栈 |
|---|---|---|---|
| sim | navigation | robot_gz | fast_lio→gicp→nav2 |
| sim | mapping | robot_gz | lio_sam 建图 |
| real | navigation | 真实 8030D 底盘 + Vanjee 722 + 真实传感器 gate | FAST-LIO(real) → GICP(real) → Nav2(real) |
| real | mapping | 真实 8030D 底盘 + Vanjee 722 + 真实传感器 gate | LIO-SAM(real) 建图 |

改 config 顶层 `platform` + `mode` 即切,不用 rebuild `system_bringup`。`use_sim_time` 从 platform 推断(sim=true, real=false)。首次使用新增 launch/YAML 或下游包时，仍须先构建对应包。

### 真机传感器 gate

`platform: real` 时，在真实底盘和 Vanjee 722 启动后，`real_sensor_ready_gate`
才会放行共享 SLAM 栈。它连续观察 **2 秒**，同时要求：

- `/points_raw` 是 `velodyne`、字段严格为 `x/y/z/intensity/ring/time`、组织形状 **32×1200**、频率至少 **8 Hz**；
- `/imu/data` 是 `imu_link`、频率至少 **150 Hz**；
- 两个消息的 header stamp 都是 fresh（相对当前 ROS 时钟年龄在 0–0.5 s）。

任一契约、频率或新鲜度不满足都会重置连续观察时间；超时则整个 launch 中止而不启动上层。仿真的 graph-only `ready_gate` 保持原有 `/points_raw + /joint_states + settling` 行为，不受此真机 gate 替代。

独立检查真机 gate（正常数据应约 2 秒后以 0 退出；停雷达后的失败路径应非 0）：

```bash
ros2 run system_bringup real_sensor_ready_gate
echo $?

ros2 run system_bringup real_sensor_ready_gate \
  --ros-args -p timeout:=5.0
echo $?
```

### 真机静态验收与 bag

真机静态验收时临时把源码 `config/bringup.yaml` 设为 `platform: real`，再选
`mode: mapping` 或 `mode: navigation`，启动 `ros2 launch system_bringup bringup.launch.py`。
验收后必须恢复仓库默认 `platform: sim`、`mode: navigation`；该切换不需要 rebuild。

建图模式应复验真实底盘、Vanjee、control gate、Web 与 LIO-SAM real 均启动，日志选用
`params_real.yaml`，`/lio_sam/mapping/odometry` 持续发布，且 `map → base_footprint`
可查询。这是既有 LIO-SAM 真机集成的静态复验，非重新调参。

车身静止、雷达节点和 `robot_state_publisher` 正常运行时，录约 30 秒回归 bag 到仓库外：

```bash
mkdir -p ~/result/rosbag
ros2 bag record \
  -o ~/result/rosbag/vanjee_722_static_2026-07-31 \
  /points_raw /imu/data /tf /tf_static

# 约 30 秒后 Ctrl-C
ros2 bag info ~/result/rosbag/vanjee_722_static_2026-07-31
```

预期约 300 条 `/points_raw`、约 6000 条 `/imu/data`（以 `ros2 bag info` 实际值为准）。
bag 保留在 `~/result/rosbag/`，不提交 `.db3` 或 metadata；可用 `ros2 bag play` 回放。

## 验收判据(PASS)
1. `colcon build` / `colcon test` 全绿;本机 `pytest core/bringup/system_bringup/test` 通过。
2. `ros2 launch system_bringup bringup.launch.py`(config `platform: sim, mode: navigation`):闸门通过 → 错峰起 robot_gz + fast_lio + gicp + nav2,全链可导航。
3. 改 config `mode: mapping`(**不用 rebuild**)再起:闸门通过 → robot_gz + lio_sam,可建图存先验图。
4. **闸门有效性**:临时把 `robot_controllers.yaml` 轮径改 0.10 → `ros2 launch system_bringup bringup.launch.py` 报"一致性闸门未通过 + [G2] wheel_radius ..."且**无任何节点启动**;改回即恢复。
5. `weld_z` 不再是裸 `-0.556`,由 `navigation.launch.py` 几何常量算出;`tf2_echo map base_footprint` z≈-0.56。
6. `platform: real` 时 bringup 启动真实 8030D 底盘和 Vanjee 722，经真实传感器 gate 后选择 real SLAM/Nav2 参数。仅源码与纯测试已在本 Windows checkout 验证；真机运行验收须在 Ubuntu/ROS 2 与目标硬件上完成。

## 改参数去哪
- 启动相关(platform/mode/gui/rviz/world/spawn/先验图路径/settling):**`config/bringup.yaml`** —— 按节点分组(`robot_gz`/`robot_bringup`/`slam_stack.*`),`bringup.launch.py` 读**源码** config,**改它不用 rebuild**。源码根由闸门自动检测。
- 调参(Nav2 vx_max 等、GICP fitness 等):仍在各自模块 yaml。
- 几何/雷达魔法值:改**权威源**(xacro / patch),`consistency_check` 守住别处不漂。

> 为什么 config 不进 `setup.py data_files`(不 install):ament_python 的 launch/config 是 data_files 拷贝到 install、改了要 rebuild。launch 用 `find_repo_root()` 读**源码** config 绕开 install。见 `consistency_check.load_bringup_config`。

## 已知边界
- 契约类(帧/话题)不入仿真 graph-only gate;真实传感器 gate 对真机接口作独立强制检查。lio-sam `savePCDDirectory` 用 `/result/loam/`(存图以 `save_map` 服务 `destination` 为准,见 mapping 模块 `save_map.sh`)。
- G3 守住 navigation.launch.py 的几何常量 == xacro,但不解析 weld_z 计算式本身。
