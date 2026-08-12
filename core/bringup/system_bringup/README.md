# core/bringup/system_bringup — Profile 编译 + 全模块启动

正式入口先把所选 Profile 编译成一次性 runtime 配置，再执行启动前一致性检查；失败即
中止且不创建任何节点，通过后按所选 platform/mode 拉起全栈。

## 设计要点
- **唯一入口 + 单次编译**：`bringup.launch.py` 从安装位置确定工作区内唯一的源码
  `config/bringup.yaml`，读取一次并调用一次 `compile_runtime_configs()`。platform、mode、
  `use_sim_time`、几何和生成配置路径随后全部来自同一 manifest；无命令行覆盖。
- **共享完整 templates**：sim/real 共用 `config/templates/` 下的 controller、Web UI、Nav2、
  FAST-LIO 和 sensor gate 原生 YAML；sensor backend 按平台选择 adapter 或 Vanjee template。
  每个平台一次生成 7 份 YAML 到唯一 `/tmp/system_bringup-runtime-*` 目录：
  `robot_controllers.generated.yaml`、`robot_web_ui.generated.yaml`、`nav2.generated.yaml`、
  `fast_lio.generated.yaml`、`sensor_gate.generated.yaml`、当前平台的
  `lidar_adapter.generated.yaml` 或 `vanjee_lidar.generated.yaml`，以及
  `effective_profile.generated.yaml`。进程内 manifest 保存这些绝对路径；源码和 install 均不被改写。
- **传感器配置单一来源**：sim adapter、real Vanjee 和两平台共享的
  `sensor_contract_gate` 分别只加载 manifest 指定的一份完整 generated YAML，不叠加 clock、
  timeout 或旧 backend 配置。主动接口固定为 `/points_raw`、`/imu/data`、`velodyne`、
  `imu_link` 和点字段 `x/y/z/intensity/ring/time`。
- **运行时闸门**：`run_runtime_consistency()` 检查 manifest、生成产物、effective report、
  所选配置和源码/安装态新鲜度，失败即 `raise`（零节点）。精确节点数、参数字典、launch
  表达式及旧路径不可达等拓扑断言只属于测试；生产启动不解析源码 AST、不维护第二份拓扑。
- **共享上层** `slam_stack.launch.py`：`mode=navigation` 错峰起
  fast_lio→gicp→nav2，并永久拥有 `body -> base_footprint` bridge；`mode=mapping` 起
  lio_sam（互斥）。`robot_navigation` 不拥有该 TF。
- **唯一控制出口**:完整 bringup 中 Nav2 发 `/cmd_vel_auto`、Web 发 `/cmd_vel_manual`，
  只有 `cmd_vel_gate` 发布 `/cmd_vel`；仿真和真机共用这条控制器入口。
- **单时钟**：所有功能状态使用节点自身的 `node.get_clock()`。graph-only `ready_gate` 的
  墙钟 `discovery_timeout` 只约束必需 topic 是否出现；topic 出现后的 settling 只累计节点
  时钟；shared sensor gate 的频率、新鲜度、稳定窗口和功能 timeout 也只使用同一节点时钟。
  仿真暂停、低 RTF 或 `/clock` 冻结时保持等待，时钟恢复后继续。

## 快速 Python 回归
```bash
python3 -m pytest core/bringup/system_bringup/test -q
```

## WSL 构建、测试与运行
前提:`core/` 在工作区;各下游包可建(robot_gz_bringup/fast_lio/gicp_localization/robot_navigation/lio_sam);先验图在 `~/result/GlobalMap.pcd`、2D 图在 `~/result/factory_map.yaml`(`pcd_to_occupancy` 生成)。

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
colcon build --packages-select system_bringup
source install/setup.bash
colcon test --packages-select system_bringup
colcon test-result --all --verbose

# 全栈启动。切 platform/mode 改源码 config/bringup.yaml 顶层两行，不用 rebuild。不带 arg:=
ros2 launch system_bringup bringup.launch.py
```

## 手机手动控制

手机访问 `http://<机器人或仿真主机IP>:8080`

- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

接管只切换 `cmd_vel_gate` 接受的速度源，不会取消已有 Nav2 goal；恢复自动后
Nav2 可继续输出。浏览器断连且仍处于 manual 模式时，所选源超过 0.5 秒无新命令，
gate 会持续发布零速。Web 不会失能硬件，不能替代物理急停或断电。

仿真单独启动 `robot_gz_bringup` 做底层诊断时，仍可用 `ros2 topic pub /cmd_vel ...`
直接测试控制器。真机 `real_chassis.launch.py` 需要权威几何参数，仅作为本 launch 的内部
include；完整 bringup 运行时不要直接发布 `/cmd_vel`，因为它必须只有
`cmd_vel_gate` 一个发布者。

## config/bringup.yaml 切换矩阵
| platform | mode | 底层 | 上层栈 |
|---|---|---|---|
| sim | navigation | robot_gz | fast_lio→gicp→nav2 |
| sim | mapping | robot_gz | lio_sam 建图 |
| real | navigation | 真实 8030D 底盘 + Vanjee 722 + shared sensor contract gate | FAST-LIO(real) → GICP(real) → Nav2(real) |
| real | mapping | 真实 8030D 底盘 + Vanjee 722 + shared sensor contract gate | LIO-SAM(real) 建图 |

改 config 顶层 `platform` + `mode` 即切，不用 rebuild `system_bringup`。`use_sim_time` 由所选
Profile 编译一次并写入 manifest 和生成 YAML，后续模块不再自行推断。首次使用新增
launch/YAML 或下游包时，仍须先构建对应包。

### 真机几何参数

真机尺寸只维护 `config/profiles/real.yaml`。启动 real Profile 时会在系统临时目录生成
controller、Web UI、Nav2、`fast_lio.generated.yaml`、Vanjee、sensor gate 和 effective report，并把同一 manifest 的
几何传给 URDF 和由 `slam_stack` 发布的 `body → base_footprint` bridge；不会修改源码或
install。以后复测尺寸只改该 Profile。

| 参数 | 当前值 | 状态/派生用途 |
|---|---:|---|
| 车体最外轮廓 长×宽 | 0.960 × 0.610 m | 人工测量；Nav2 footprint 为 ±0.480 × ±0.305 m |
| 车体外壳高度 | 0.377 m | 人工测量 |
| 地面到外壳下沿 | 0.143 m | 人工测量；`base_link` 高度派生为 0.3315 m |
| 驱动轮直径/半径 | 0.205 / 0.1025 m | 最新人工测量；影响轮式里程计尺度 |
| 单轮宽度 | 0.101 m | 人工测量 |
| 驱动轮中心距 | 0.463 m | 由内侧间距 0.362 m + 单轮宽度 0.101 m 得出 |
| 两轮内侧净距/轮外缘到车体外轮廓 | 0.362 / 0.023 m(每侧) | 原始复核尺寸；0.362+2×0.101+2×0.023=0.610 m |
| 雷达原点(base_footprint) x/y/z | 0.443 / 0 / 0.905 m | 原点暂按底座与半球盖之间中线 |
| 雷达 roll/pitch/yaw | 0 / 0 / 0 rad | 重新安装后正向、水平 |
| 雷达原点(base_link) x/y/z | 0.443 / 0 / 0.5735 m | 派生值 |
| IMU 原点(base_footprint) x/y/z | 0.443 / 0 / 0.905 m | 独立 Profile mount；当前值与雷达相同 |
| IMU roll/pitch/yaw | 0 / 0 / 0 rad | 独立 Profile mount；当前值与雷达相同 |
| IMU 原点(base_link) x/y/z | 0.443 / 0 / 0.5735 m | 独立派生值 |
| body → base_footprint x/y/z | -0.443 / 0 / -0.905 m | 派生值；当前零旋转 |

向厂家核对：WLR-722 机壳内坐标原点、XYZ 正方向/正面定义、雷达与内置 IMU 的精确外参、32 线垂直角表/FOV、整机与安装孔尺寸；底盘厂家还需确认有效滚动半径、轮中心距、驱动轴相对车体前后方向的位置、前后悬长度、减速比及反馈单位。当前暂按驱动轴通过车体几何中心；若厂家图显示不是这样，必须先重定义 `base_footprint` 并复测雷达 x/footprint。非零安装角使用完整 SE(3) 逆变换派生 permanent bridge，不做平移量简单取反。

### Shared sensor contract gate

sim/real 都使用同一个 `sensor_contract_gate`。sim 在 `/joint_states` discovery 与既有 settling
完成后启动它；real 在真实底盘和 Vanjee 722 启动后启动它。gate 连续观察 **2 秒**，成功后
才放行共享 SLAM 栈，并同时要求：

- `/points_raw` 是 `velodyne`、字段严格为 `x/y/z/intensity/ring/time`、总点数为当前 Profile
  的 `scan_lines × columns_per_scan`（sim 28800、real 38400）、频率至少 **8 Hz**；
- `/imu/data` 是 `imu_link`、频率至少 **150 Hz**；
- 两个消息的 header stamp 都是 fresh（相对当前 ROS 时钟年龄在 0–0.5 s）。

任一契约、频率或新鲜度不满足都会重置连续观察时间；超时则整个 launch 中止而不启动上层。
仿真的 graph-only `ready_gate` 只等待 `/joint_states`：墙钟 `discovery_timeout` 只处理 topic
始终未出现，topic 出现后的 settling 和 sensor gate 都只使用各自的节点时钟。因此暂停仿真、
低 RTF 或 `/clock` 冻结时保持等待，时钟恢复后继续累计。

独立检查 gate 时必须为本次运行新建随机 runtime 目录，并加载 compiler 刚生成的完整参数
文件；不要硬编码目录、跨次复用生成物或用 launch 参数覆盖：

```bash
cd core
runtime_dir="$(mktemp -d /tmp/system_bringup-runtime-XXXXXX)"
trap 'rm -rf -- "$runtime_dir"' EXIT

ros2 run system_bringup compile_runtime_configs \
  --bringup-config "$PWD/bringup/system_bringup/config/bringup.yaml" \
  --output-dir "$runtime_dir"
ros2 run system_bringup sensor_contract_gate \
  --ros-args --params-file "$runtime_dir/sensor_gate.generated.yaml"
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
  -o ~/result/rosbag/vanjee_722_static_latest \
  /points_raw /imu/data /tf /tf_static

# 约 30 秒后 Ctrl-C
ros2 bag info ~/result/rosbag/vanjee_722_static_latest
```

预期约 300 条 `/points_raw`、约 6000 条 `/imu/data`（以 `ros2 bag info` 实际值为准）。
bag 保留在 `~/result/rosbag/`，不提交 `.db3` 或 metadata；可用 `ros2 bag play` 回放。

## 验收判据(PASS)
1. `colcon build` / `colcon test` 全绿;本机 `pytest core/bringup/system_bringup/test` 通过。
2. `ros2 launch system_bringup bringup.launch.py`(config `platform: sim, mode: navigation`):闸门通过 → 错峰起 robot_gz + fast_lio + gicp + nav2,全链可导航。
3. 改 config `mode: mapping`(**不用 rebuild**)再起:闸门通过 → robot_gz + lio_sam,可建图存先验图。
4. **闸门有效性**：`test_runtime_consistency.py` 覆盖 manifest/产物/report 不一致、缺失文件和
   源码/安装态陈旧等失败路径；正式入口在创建任何节点前中止。拓扑精确断言由
   `test_control_topology.py` 等静态测试负责，不进入生产闸门。
5. 仿真 bridge 保持 `z=-0.556`；真机由 real Profile 派生
   `body→base_footprint=[-0.443,0,-0.905]`，`tf2_echo body base_footprint` 应与其一致。
6. `platform: real` 时 bringup 启动真实 8030D 底盘和 generated Vanjee 722，经 shared sensor
   contract gate 后选择
   real SLAM/Nav2 参数。静态/fake/mock 链已验收；无人看护的真机动态运动未执行。

## 改参数去哪
- 启动选择和编排参数（platform/mode/gui/rviz/world/spawn/先验图路径/settling）：
  **`config/bringup.yaml`**。正式 launch 固定读取工作区源码文件，修改后不用 rebuild。
- 平台事实（车体/车轮/雷达安装几何、运动上限、传感器和后端选择）：
  **`config/profiles/sim.yaml`** 或 **`config/profiles/real.yaml`**。
- controller、Web UI、Nav2、FAST-LIO、sensor gate 及 sensor backend 的完整原生参数：
  **`config/templates/*.yaml`**。共享模块由 sim/real 共用 template；adapter/Vanjee 按当前
  backend 选择，Profile 字段只覆盖编译器明确负责的值。
- 正式 FAST-LIO 参数由上述 source template 渲染成 manifest 指定的绝对
  `fast_lio.generated.yaml`；安装副本仅证明打包，GICP 和 LIO-SAM 仍在各自模块 YAML。
- 雷达设备协议和厂家标定仍归驱动/设备配置；Profile 维护跨模块需要共享的平台事实。

> `bringup.yaml` 与 Profiles 是运行时源码事实，不复制到 install；launch 从已安装 package share
> 固定映射回同一 colcon 工作区的源码路径，所以改配置不用 rebuild。templates 同时保留安装副本，
> runtime 闸门会校验源码与安装态字节一致，防止修改 template 后忘记重建。

## 已知边界
- 契约类（帧/话题/字段）由 sim/real 共用的 `sensor_contract_gate` 强制检查；仿真
  graph-only gate 只负责 `/joint_states` discovery。lio-sam `savePCDDirectory` 用
  `/result/loam/`（存图以 `save_map` 服务 `destination` 为准，见 mapping 模块
  `save_map.sh`）。
- runtime 闸门只验证实际消费的 manifest 与产物，不复述 launch 拓扑；精确拓扑和表达式结构
  由测试在源码侧验证。
