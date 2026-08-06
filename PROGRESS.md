# 参数 Profile 化改造进度

## 目标

用两套中央 Profile 隔离仿真与真实平台差异，用集中在 `system_bringup` 的各模块原生
YAML template 保留算法配置，由 `system_bringup` 在启动前结构化生成 `/tmp` 下的最终
配置。用户只需要选择 `platform:=sim|real`；bringup config 根据该值选择 Profile 路径，
`use_sim_time` 等底层参数由选择结果派生。

本工作不一次性设计或修改全部链路。以下小节必须逐节完成：先讨论边界和验收条件，
再实现、验证，经确认后才进入下一节。

## 已确认原则

- 第一版只建立 `sim`、`real` 两套中央 Profile，二者完整、自包含并使用相同 schema。
- Profile 文件不保存版本号或平台名；bringup config 保存 sim/real Profile 的相对路径。
- Profile 路径相对于 `bringup.yaml` 所在目录解析，不依赖进程当前工作目录。
- Profile 只保存机器人事实、平台差异和必须跨模块一致的策略。
- 本机器人使用的模块原生 YAML template 集中放在 `system_bringup`；模块内部算法细节
  归对应 template 所有，不复制进中央 Profile。
- 固定 topic/frame、驱动协议换算和设备内部极限不是可调 Profile 字段。
- Profile 中的运动参数就是系统实际采用的统一限制，不重复维护“实际值/硬上限”两套值；
  Web UI 只允许用户在此范围内运行时调速。
- Profile 每个字段必须用简短注释说明含义、单位和必要的坐标约定。
- 不引入 Jinja2。使用 `yaml.safe_load()`、结构化字典修改和 `yaml.safe_dump()`。
- URDF 继续使用 Xacro；Launch 继续使用 ROS 2 Python Launch。
- 最终配置生成到 `/tmp`，不跟踪、不回写源码、不修改安装目录。
- `platform:=sim|real` 负责选择 Profile；`mode:=mapping|navigation` 保持独立。
- `use_sim_time` 是派生的 ROS 参数，不作为平台选择开关。
- 每个迁移小节都必须保持未迁移模块的现有行为。
- 严禁使用 git worktree。

## 状态说明

- `[ ]` 未开始
- `[~]` 正在讨论或实现
- `[x]` 已实现并通过本节验收
- `[!]` 已知阻塞，需先解决记录的问题

## 当前基线

- 已完成分支：`feat/vanjee-722-full-stack-integration`
- 功能完成提交：`8b091dd fix(system_bringup): exit real sensor gate cleanly`
- 主分支合并提交：`61c3abe merge: 合并 Vanjee 722 真实全链路适配`
- Profile 化工作分支：`feat/profile-template-config`
- 已有 `build_real_runtime_configs()` 是结构化生成的原型，但不是最终边界。

### 已撤销的临时实机调参记录

以下数值曾在真机导航中验证，可以明显减弱猛烈转向、左右摆动和危险恢复动作；它们原先
只存在于暂存区，现已从源码撤销，不属于已完成的雷达/全链路适配基线。后续分别在底盘与
Nav2 小节讨论后，通过新 Profile 正式恢复，不能直接照抄回旧 `bringup.yaml`：

- 底盘最大角速度：`0.4 rad/s`
- 底盘最大角加速度：`0.3 rad/s²`
- MPPI 最小线速度：`-0.1 m/s`
- MPPI 角速度采样标准差：`0.2 rad/s`
- MPPI 最大角速度：与底盘上限统一为 `0.4 rad/s`
- 保留 Spin 恢复动作；最大/最小旋转速度为 `0.2/0.1 rad/s`
- Spin 旋转加速度上限：`0.2 rad/s²`

实机现象记录：原始配置会在障碍恢复和接近终点时出现突然猛烈转向；仅降低角速度后仍会
反复过冲和修正。上述组合限制后，正常路径跟踪和恢复转向明显柔和，受阻场景也能正常结束。
这些结论只证明当前车辆上的可用性，不代表 sim 与 real 应共享同一组数值。

## 分节进度

### [x] 1. Profile 职责与 schema

目的：确定中央 Profile 管什么、不管什么，以及 `sim.yaml`、`real.yaml` 的共同字段结构。

本节已确定：

- 机器人几何与运动能力
- 传感器物理契约
- 硬件能力与安全边界
- 跨模块导航、建图、定位策略
- 参数是直接事实、人工调参还是派生值

本节只形成最小 schema、字段归属表和派生关系表；尚不迁移任何模块。

#### 1.1 三层职责

| 层级 | 负责回答 | 典型内容 |
|---|---|---|
| bringup config / launch 参数 | 本次启动什么 | `platform`、`mode`、Profile 路径、地图、world、GUI、RViz |
| Profile | 当前平台实际是什么 | 几何、安装位姿、连接参数、传感器事实、统一运动限制、障碍高度带 |
| module template | 各算法如何工作 | FAST-LIO、GICP、LIO-SAM、Nav2、ros2_control 的完整原生 YAML |

bringup config 不是第三套模块参数覆盖层，只负责选择和编排。运行时按以下顺序生成：

```text
bringup config 选择 platform
            -> 取得 profiles[platform] 的相对路径
            -> 读取 Profile + 中央 module templates
            -> 结构化生成 /tmp 下的模块原生 YAML
            -> launch 将绝对路径传给各模块
```

算法模块不主动寻找 `/tmp`，也不因本改造修改算法代码；只允许调整我们拥有的 launch
包装层，使其接收最终配置的绝对路径。

#### 1.2 bringup config 的 Profile 选择

```yaml
platform: real  # 当前平台：sim 或 real

profiles:
  sim: profiles/sim.yaml    # 相对 bringup.yaml 所在目录解析
  real: profiles/real.yaml  # 相对 bringup.yaml 所在目录解析
```

Profile 自身不保存 `schema_version` 或 `platform`。schema 由编译器代码和测试定义；
`use_sim_time` 由 bringup config 中被选中的 `platform` 派生。

#### 1.3 最小 Profile schema

下列示例展示 real 的结构和当前已确认值。`null` 只表示数值留到对应实施小节确认；最终
可运行 Profile 的必填字段不得为 `null`。sim 使用完全相同的字段结构，不适用的真机
连接字段明确写 `null`，不填写伪造值。

```yaml
hardware:
  chassis:
    backend: can_8030d  # 底盘驱动后端；仿真使用 gazebo

  lidar:
    backend: vanjee              # 雷达驱动后端；仿真使用 gazebo
    model: vanjee_722            # 驱动支持的雷达型号
    host_address: 192.168.2.88   # 本机雷达网口 IPv4；仿真填 null
    device_address: 192.168.2.86 # 雷达设备 IPv4；仿真填 null
    host_msop_port: 3001         # 本机接收 MSOP 的 UDP 端口；仿真填 null
    device_msop_port: 3333       # 雷达发送 MSOP 的 UDP 端口；仿真填 null

robot:
  body:
    front_extent: 0.480     # base_footprint 到车体最前端，单位 m
    rear_extent: 0.480      # base_footprint 到车体最后端，单位 m
    left_extent: 0.305      # base_footprint 到车体最左侧，单位 m
    right_extent: 0.305     # base_footprint 到车体最右侧，单位 m
    height: 0.377           # 车体外壳高度，单位 m
    ground_clearance: 0.143 # 地面到车体外壳最下沿，单位 m

  drive:
    wheel_radius: 0.1025    # 驱动轮有效半径，单位 m
    wheel_width: 0.101      # 单个驱动轮宽度，单位 m
    wheel_separation: 0.463 # 左右驱动轮中心距，单位 m

  mounts:
    # 以下安装位姿均以 base_footprint 为父坐标系，采用 REP-103：x 前、y 左、z 上。
    lidar:
      x: 0.443   # 雷达原点相对父坐标系向前距离，单位 m
      y: 0.0     # 雷达原点相对父坐标系向左距离，单位 m
      z: 0.905   # 雷达原点相对父坐标系向上距离，单位 m
      roll: 0.0  # 绕 x 轴旋转，单位 rad
      pitch: 0.0 # 绕 y 轴旋转，单位 rad
      yaw: 0.0   # 绕 z 轴旋转，单位 rad

    # 当前按厂商 LIO-SAM 的零平移、单位旋转外参填写；逻辑上仍与雷达独立建模。
    imu:
      x: 0.443   # IMU 逻辑原点相对父坐标系向前距离，单位 m
      y: 0.0     # IMU 逻辑原点相对父坐标系向左距离，单位 m
      z: 0.905   # IMU 逻辑原点相对父坐标系向上距离，单位 m
      roll: 0.0  # 绕 x 轴旋转，单位 rad
      pitch: 0.0 # 绕 y 轴旋转，单位 rad
      yaw: 0.0   # 绕 z 轴旋转，单位 rad

sensors:
  lidar:
    scan_lines: 32                   # 垂直扫描线数
    columns_per_scan: 1200           # 每帧每线的水平点数
    scan_rate_hz: 10.0               # 完整点云帧频率，单位 Hz
    min_range: 0.05                  # 驱动输出的最小距离，单位 m
    max_range: 70.0                  # 驱动输出的最大距离，单位 m
    horizontal_start_angle: 0.0      # 水平输出起始角，单位 rad
    horizontal_end_angle: 6.2831853  # 水平输出终止角，单位 rad
    point_time_field: time           # PointCloud2 逐点相对时间字段名
    point_time_unit: seconds         # 逐点时间字段数值单位
    point_time_reference: scan_start # 逐点时间相对本帧起始时刻

  imu:
    rate_hz: 200.0 # IMU 标称发布频率，单位 Hz

motion:
  max_linear_velocity: null     # 系统采用的最大线速度绝对值，单位 m/s；第 3 节确认
  max_angular_velocity: null    # 系统采用的最大角速度绝对值，单位 rad/s；第 3 节确认
  max_linear_acceleration: null # 系统采用的最大线加速度绝对值，单位 m/s²；第 3 节确认
  max_angular_acceleration: null # 系统采用的最大角加速度绝对值，单位 rad/s²；第 3 节确认

perception:
  obstacle_height:
    min: null # 纳入障碍物处理的最低离地高度，单位 m；第 8 节确认
    max: null # 纳入障碍物处理的最高离地高度，单位 m；第 8 节确认
```

所有长度使用 m，角度使用 rad，时间使用 s，频率使用 Hz，线速度/加速度使用 m/s、
m/s²，角速度/加速度使用 rad/s、rad/s²。驱动要求其他单位时由编译器转换。

#### 1.4 字段所有权

| 参数类别 | 唯一所有者 | 示例 |
|---|---|---|
| 本次启动选择 | bringup config | `platform`、`mode`、sim/real Profile 路径 |
| 启动资源与交互 | bringup config / launch 参数 | 地图、先验 PCD、world、GUI、RViz |
| 硬件连接 | Profile | 底盘/雷达后端、雷达型号、IP、UDP 端口 |
| 机器人几何 | Profile | 车体边界、车高、离地间隙、轮径、轮宽、轮距 |
| 传感器安装 | Profile | LiDAR/IMU 相对 `base_footprint` 的 `xyz+rpy` |
| 传感器数据事实 | Profile | 32×1200、10 Hz、200 Hz、量程、逐点时间单位 |
| 系统采用的运动限制 | Profile | 最大线/角速度和加速度；Web UI 在范围内运行时调速 |
| 跨模块障碍高度带 | Profile | 以地面为基准的最低/最高障碍高度 |
| 模块算法与行为参数 | 对应中央 module template | voxel、噪声、GICP 阈值、MPPI critic、Spin、倒车限制、膨胀参数 |
| 驱动行为参数 | 对应中央 module template | `wait_for_difop`、时间戳策略、dense points |
| 固定接口契约 | 代码、URDF 和 template 约束 | `/points_raw`、`camera_init`、`base_footprint` 等 |
| 设备协议实现 | 驱动代码 | CAN 报文、RPM 换算、左右轮符号、设备内部限幅 |
| 运行时模块配置 | `/tmp` 生成物 | Nav2、FAST-LIO、GICP、LIO-SAM、驱动 YAML |

固定 topic/frame 中部分目前是参数、部分由 FAST-LIO/URDF 写死，但都不是正常调参入口。
第一版不实现全链路任意改名；中央 templates 保持现有接口，一致性检查负责防止漂移。

#### 1.5 派生规则

| 派生值 | 规则 |
|---|---|
| 选中的 Profile | `bringup.profiles[platform]`，路径相对 `bringup.yaml` 解析 |
| `use_sim_time` | `platform == sim` |
| 车体长度/宽度 | `front+rear` / `left+right` extent |
| 车体中心偏移 | `x=(front-rear)/2`，`y=(left-right)/2` |
| `base_link` 离地高度 | `ground_clearance + body.height/2` |
| Nav2 footprint | `[[front,left],[front,-right],[-rear,-right],[-rear,left]]` |
| URDF 轮子位置 | 由轮距、轮径和 `base_link` 高度计算 |
| LiDAR/IMU 相对 `base_link` 位姿 | 从各自相对 `base_footprint` 的位姿转换 |
| LiDAR↔IMU 外参 | 两个独立安装位姿之间的相对变换 |
| 单帧点数 | `scan_lines * columns_per_scan`，不在 Profile 重复保存 |
| 扫描周期与逐点时间跨度 | `1/scan_rate_hz`；实际跨度应约等于一个扫描周期 |
| FAST-LIO `timestamp_unit` | 从 `point_time_unit` 映射为模块要求的枚举 |
| Vanjee 水平角度 | Profile 的 rad 转换为驱动参数要求的 degree |
| readiness 频率下限 | Profile 标称频率乘 gate template 中的容差比例 |
| 地图裁剪高度 | 使用以地面为基准的障碍高度带 |
| 传感器相对高度过滤 | 目标模块需要传感器相对值时，用障碍高度减安装高度 |
| 通用运动限制 | Profile 值写入语义相同的 ros2_control、Web UI 和 Nav2 通用上限 |

当前底盘通过厂商 USB-CAN SDK 连接，并不使用 Linux SocketCAN `can0`。因此 Profile
不设置 `can_interface`；`can_device_type/index/channel/timing` 等已调通且不需日常切换的
驱动细节继续归底盘驱动 template 所有。

Spin 速度、倒车速度、MPPI 采样标准差等不是通用运动限制的重复表达，继续由 Nav2
template 独立拥有。人工驾驶速度由用户在 Web UI 中运行时选择，不另建人工驾驶 Profile。

### [~] 2. Profile 编译器最小骨架

目的：建立读取 Profile、校验、派生并在 `/tmp` 报告有效配置的最小流程。

已批准设计：新增独立 `profile_compiler.py`、完整同 schema 且逐字段注释的 sim/real
Profile，以及 `compile_profile` CLI。本节只严格启用几何校验，尚未讨论的 motion/
perception 值允许为 `null`；生成 `effective_profile.generated.yaml` 供检查。

边界：不接入正式 `bringup.launch.py`，不修改旧 `derive_real_geometry()` /
`build_real_runtime_configs()`，不生成任何模块 YAML，也不迁移后续参数。

完成条件：sim/real 都能生成有效报告；缺字段、类型错误和非法几何明确失败；real 已确认
几何与旧原型等价；原有测试通过且现有启动行为不变。

本地设计产物：`docs/superpowers/specs/2026-08-06-profile-compiler-skeleton-design.md`
（按用户要求不加入 Git 提交）。

### [ ] 3. 底盘、ros2_control 与手动控制

目的：统一轮径、轮宽、轮距和系统实际采用的速度/加速度限制；Web UI 允许用户在该
范围内运行时调速。设备协议换算、左右轮符号和设备内部极限继续归驱动代码所有。

完成条件：controller、Web UI 和 Nav2 的同义通用限制来自同一 Profile；各模块不同
语义的行为参数不被错误合并；仿真与真机可分别构建、静态检查。

### [ ] 4. 雷达、IMU 与传感器契约

目的：统一雷达型号、线数、水平分辨率、频率、量程、视场角、时间语义和安装位姿；
固定 topic/frame 契约由中央 templates 保持并由一致性检查验证。

完成条件：驱动、仿真适配器、TF 和 gate 使用同一平台事实；厂商标定文件继续作为
设备数据管理，不被模板生成。

### [ ] 5. FAST-LIO 配置迁移

目的：从 Profile 注入真实/仿真的传感器契约、时间同步和明确需要跨模块一致的参数；
滤波、迭代等 FAST-LIO 内部调参仍留在模板。

完成条件：两平台生成原生 FAST-LIO YAML；现有仿真行为不变，真机点云时间与里程计
契约可单独验收。

### [ ] 6. GICP 配置迁移

目的：将 GICP 原生配置迁入中央 template，并注入已确认的平台事实；地图路径继续归
bringup config / launch 参数，固定坐标系与输入契约不提升为 Profile 可调字段。

完成条件：保持现有已验证的 `fitness_threshold` 和配准行为；已记录的 readiness 与
`/initialpose` 问题不在本节顺带修复。

### [ ] 7. LIO-SAM 与地图产物

目的：统一传感器事实和二维地图转换使用的障碍高度带，明确厂商 IMU 参数与算法内部
参数的所有权；地图输出目录和保存目标继续归 bringup config / launch 参数。

完成条件：建图、PCD 保存、GICP 先验图和 Nav2 地图路径一致；地图转换参数可从所选
Profile 获取，现有地图不会被测试过程覆盖。

### [ ] 8. Nav2 障碍感知与代价地图

目的：由 Profile 派生 footprint、雷达安装高度和统一障碍高度带；局部窗口、膨胀和
完整车体碰撞等 Nav2 行为继续归中央 Nav2 template。

完成条件：明确 `inflation_radius`、`cost_scaling_factor` 与
`consider_footprint` 的安全关系；sim/real 使用各自传感器模型；先做静态验收，再安排
有人看护的动态验收。

### [ ] 9. Nav2 规划、控制与恢复

目的：将 Profile 的通用运动限制注入同义参数；MPPI 采样、倒车、Spin 恢复和 Critic
等不同语义的行为参数继续归中央 Nav2 template。

完成条件：规划器、MPPI、底盘和恢复动作不存在互相矛盾的运动能力；保留当前已经实机
验证的柔和转向与受阻行为。

### [ ] 10. 全链路切换、验收与文档收尾

目的：让唯一平台选择贯穿 mapping/navigation 两种模式，删除完成迁移后失去用途的
重复 real/sim 参数文件或旧注入路径，并更新操作文档。

完成条件：

- `platform:=sim|real` 可生成并启动对应完整链路。
- 能查看每个模块最终使用的有效配置。
- 仿真回归通过，真机按静态、有人看护动态、长时间稳定三层验收。
- 无源码配置被运行时生成物覆盖，`/tmp` 产物不进入 Git。
- 重复参数和旧路径的删除单独审查，不与功能迁移混在同一小节。

## 执行规则

每一节按以下顺序推进：

1. 盘点本节参数及当前来源。
2. 讨论 Profile 边界、派生规则和不迁移项。
3. 确认本节设计与验收条件。
4. 只实现本节范围。
5. 执行针对性测试，并检查 sim/real 未发生意外串扰。
6. 自审遗漏、未确认假设和兼容风险。
7. 经用户确认后更新本文件状态，再进入下一节。

任何小节发现跨节依赖时，只记录依赖；除非它阻塞当前验收，否则不提前实现后续小节。

## 下一步

第 1 节设计已完成。下一步在用户确认后进入第 2 节，只讨论并实现 Profile 编译器最小
骨架；不提前迁移第 3 节及后续模块参数。
