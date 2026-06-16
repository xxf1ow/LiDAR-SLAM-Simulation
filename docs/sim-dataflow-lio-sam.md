# 仿真系统消息流转与配置参考 —— Gz Harmonic + LIO-SAM 建图

> 适用对象:本仓库 **Gazebo Sim Harmonic** 仿真 + **LIO-SAM** 建图(`core/mapping`)一起运行时的形态。
> 仿真前端(Gz 原生传感器 → 桥接 → adapter → `/points_raw`、`/imu_plugin/out`)与定位栈
> (FAST-LIO,`core/localization`)**共用**;本文聚焦建图配置,定位侧差异在文末点出。
>
> **怎么用这份文档**:左手读本文搞清"应该是什么样",右手在构建机把仿真跑起来,用第 8 节的运行时
> 命令逐条对照"实际是什么样"。两者对上,你就真正看懂了。
>
> 事实来源:`core/simulation/robot_gz_bringup/{launch/robot_gz.launch.py, config/bridge.yaml, worlds/factory.sdf}`、
> `core/simulation/lidar_pointcloud_adapter/`、`core/robot/robot_bringup/config/robot_controllers.yaml`、
> `core/mapping/lio-sam.patch`。SLAM **内部**逐值参数(N_SCAN/噪声等)以 `lio-sam.patch` 实际内容为准,
> 本文只列已在仿真侧坐实的契约值。

---

## 1. 启动拓扑:两条 launch,分两步起

整套建图系统由**两条相互独立的 launch** 组成,分别在两个终端启动(均 `cd core && source install/setup.bash`):

```
① 仿真本体(含传感器)   ros2 launch robot_gz_bringup robot_gz.launch.py
② SLAM 算法(LIO-SAM)   ros2 launch lio_sam run.launch.py
```

**① robot_gz.launch.py** 拉起(`core/simulation/robot_gz_bringup/launch/robot_gz.launch.py`):
- `ros_gz_sim` 起 Gz Harmonic,加载世界 `worlds/factory.sdf`(`-r` 立即运行;`gui:=false` 走 headless)。
- `ros_gz_sim create`:从 `/robot_description` 话题 spawn 机器人(默认 `x=4, y=0, z=0.05`)。
- `robot_state_publisher`:展开 `robot.urdf.xacro`(`use_gazebo:=true`),发布机器人 URDF 静态/动态 TF。
- `ros_gz_bridge`(`parameter_bridge`,配置见 §3):桥接 `/clock`、`/lidar/points`、`/imu`。
- `lidar_pointcloud_adapter`:Gz 组织化点云 → Velodyne 风格 `/points_raw`(见 §3.2)。
- 控制器 spawner:`joint_state_broadcaster` + `base_controller`(`controller_manager` 由 URDF 里的
  `gz_ros2_control` 插件提供,**无独立 `ros2_control_node`**)。
- 可选 `rviz2`(`rviz:=true`,默认关;看裸 URDF 用,建图看 LIO-SAM 自带 RViz)。

**② lio_sam run.launch.py**(来自 `core/mapping/lio-sam.patch`)拉起:
- `static_transform_publisher`:发布 **`map → odom` 静态单位阵**。
- `lio_sam_imuPreintegration`、`lio_sam_imageProjection`、`lio_sam_featureExtraction`、
  `lio_sam_mapOptimization`(LIO-SAM 四节点流水线)。
- `rviz2`(`use_sim_time:=true`)。

> 注意:run.launch.py 里 LIO-SAM 自带的 `robot_state_publisher` 被**禁用**——TF 由仿真那条 launch
> 统一提供,避免两边都发 URDF 静态 TF 打架。

---

## 2. 节点清单(谁是谁)

| 节点 | 来源 | 角色 |
|---|---|---|
| `gz sim` server/GUI | ros_gz_sim | 物理仿真 + 渲染;**承载 `factory.sdf` 世界 + spawn 的机器人传感器** |
| `ros_gz_bridge` | ros_gz_bridge | Gz↔ROS 话题桥接(clock/lidar/imu) |
| `lidar_pointcloud_adapter` | 本仓库 | Gz 组织化点云 → Velodyne 风格 `/points_raw`(补 `time`、透传 `ring`) |
| `robot_state_publisher` | URDF | 把 `robot.urdf.xacro` 的 link/joint 发成 TF(机器人骨架) |
| `controller_manager` | gz_ros2_control(URDF 插件) | 在 Gz 物理步里跑 ros2_control(无独立进程) |
| `base_controller` | diff_drive_controller | 订阅 `/cmd_vel`(TwistStamped)、驱动两轮、发 `/base_controller/odom` |
| `joint_state_broadcaster` | ros2_controllers | 发 `/joint_states` |
| `lio_sam_imageProjection` | LIO-SAM | 点云去畸变、投影成 range image |
| `lio_sam_featureExtraction` | LIO-SAM | 提取角点/平面点特征 |
| `lio_sam_mapOptimization` | LIO-SAM | scan-to-map 优化、关键帧、回环、建全局图 |
| `lio_sam_imuPreintegration` | LIO-SAM | IMU 预积分,高频里程计 + 发布 `odom→base_footprint` TF |
| `static_transform_publisher` | tf2_ros | `map→odom` 静态单位阵 |

---

## 3. 数据源:Gz 原生传感器 + 桥接 + adapter

旧 Classic 栈的传感器是 robot.sdf 里的 `libgazebo_ros_*` 插件;**新栈是 Gz Harmonic 原生传感器系统**
(`gpu_lidar` / `imu`,定义在 `robot_description` 的 gazebo xacro 里),经 `ros_gz_bridge` 进 ROS,
雷达再过一道 adapter。这是数据的源头。

### 3.1 IMU
- Gz 原生 `imu` 传感器,**200 Hz**。`bridge.yaml` 把 Gz 的 `/imu` 桥成 ROS 的 **`/imu_plugin/out`**
  (`sensor_msgs/msg/Imu`)——**SLAM 契约话题名保持 `/imu_plugin/out` 不变**(与旧栈一致,免改 SLAM 配置)。
- `header.frame_id` = `imu_link`;IMU 与雷达**共位**(`base_link + z=0.236`,extrinsic 为零)。

### 3.2 LiDAR(两段:Gz 原生 → adapter)
- Gz 原生 `gpu_lidar`(VLP-16 式:16 线、每圈约 1800 列、**10 Hz**),发布**组织化**点云到 Gz 话题
  `/lidar/points`,`bridge.yaml` 桥成 ROS `/lidar/points`(`sensor_msgs/msg/PointCloud2`,
  由 `gz.msgs.PointCloudPacked` 转来,`height=16`、`width≈1800`)。
- `lidar_pointcloud_adapter`(参数见 launch):`input_topic=/lidar/points` → `output_topic=/points_raw`,
  `output_frame=velodyne`,`scan_period=0.1`。它把组织化点云转成 **Velodyne 风格 `/points_raw`**:
  - 字段补成 `x/y/z/intensity/ring/time`;
  - **`ring`** 优先透传 Gz native,缺失时按 `i // width` 重算;
  - **`time`** 按列号合成 `(i % width) / width * scan_period`(方位角时间)。
- **为何要补 `time`**:Gz 输出的是**瞬时快照**,无原生逐点时间。FAST-LIO 检测到非零 `time` 即
  `given_offset_time=true` 并据此去畸变;LIO-SAM 对无 time 输入会自动关去畸变。详见 §9。

### 3.3 差速驱动 —— `base_controller`(diff_drive_controller)
`core/robot/robot_bringup/config/robot_controllers.yaml`:
- 名为 **`base_controller`**,`controller_manager` `update_rate=50 Hz`。
- 订阅 **`/cmd_vel`(TwistStamped)** —— ⚠️ Humble 的 `diff_drive_controller` 默认收 **TwistStamped**,
  发普通 `Twist` 会被**静默忽略**(车不动的头号原因)。
- `wheel_separation=0.55`、`wheel_radius=0.12`;`publish_rate=50`,发 `/base_controller/odom`
  (`odom_frame_id=odom`、`base_frame_id=base_footprint`)。
- **`enable_odom_tf:false`(关键)**:轮式里程计**只发话题、不发 TF**;`odom→base_footprint` 让给 SLAM
  (建图时 LIO-SAM 独占),避免双发布者抢同一条 TF 边而跳变。
- 限速:`linear.x` ±1.5 m/s、`angular.z` ±2.0 rad/s(加速度限幅见文件)。

> 旧栈的 `robot_control.py` 旁路节点、`libgazebo_ros_joint_state_publisher` 等已不存在;关节状态由
> `joint_state_broadcaster` 发 `/joint_states`。

---

## 4. 话题流转表

| 话题 | 类型 | 发布者 | 主要消费者 | 频率 |
|---|---|---|---|---|
| `/clock` | Clock | Gz(经桥接) | 全体(`use_sim_time`) | — |
| `/lidar/points` | PointCloud2 | Gz gpu_lidar(经桥接) | `lidar_pointcloud_adapter` | 10 Hz |
| `/points_raw` | PointCloud2 | `lidar_pointcloud_adapter` | LIO-SAM `imageProjection`(定位时 FAST-LIO) | 10 Hz |
| `/imu_plugin/out` | Imu | Gz imu(经桥接) | `imageProjection`, `imuPreintegration`(定位时 FAST-LIO) | 200 Hz |
| `/cmd_vel` | **TwistStamped** | teleop / Nav2(经 twist_stamper) | `base_controller` | 按需 |
| `/base_controller/odom` | Odometry | `base_controller` | (建图不用;Nav2 用其 twist) | 50 Hz |
| `/joint_states` | JointState | `joint_state_broadcaster` | `robot_state_publisher` | ~50 Hz |
| `lio_sam/mapping/odometry` | Odometry | mapOptimization | imuPreintegration | ~10 Hz |
| `lio_sam/mapping/cloud_registered` | PointCloud2 | mapOptimization | RViz | ~10 Hz |
| `lio_sam/mapping/map_global` | PointCloud2 | mapOptimization | RViz(全局图) | 低频 |
| `lio_sam/mapping/path` | Path | mapOptimization | RViz | ~10 Hz |

---

## 5. LIO-SAM 内部流水线(四节点怎么串起来)

```
/points_raw ┐
            ├─► imageProjection ──cloud_info──► featureExtraction ──cloud_info──► mapOptimization
/imu_plugin/out ┘  (去畸变/投影)                  (角点/平面特征)                 (scan-to-map+回环)
       │                                                                              │
       └──────────────────► imuPreintegration ◄──────mapping/odometry────────────────┘
                            (IMU 预积分,高频)
                                  │
                                  └─► 发布 TF: odom → base_footprint
```

- **imageProjection**:用 IMU 去畸变 + 投影成 range image,输出 `cloud_deskewed` / `cloud_info`。
- **featureExtraction**:从 range image 提角点/平面点。
- **mapOptimization**:scan-to-map 因子图优化、关键帧管理、回环检测,产出全局地图与 `mapping/odometry`;
  `/lio_sam/save_map` 服务把全局图存成 PCD(**这就是定位/导航阶段的先验图来源**,默认存 `~/result`)。
- **imuPreintegration**:融合 IMU 与 `mapping/odometry`,输出高频位姿,并广播 `odom→base_footprint`。

---

## 6. TF 树(谁发布哪条边)

```
map ──(静态单位阵, run.launch 的 static_transform_publisher)──► odom
 odom ──(动态, lio_sam_imuPreintegration)──► base_footprint
   base_footprint ──(robot_state_publisher, URDF)──► base_link
      base_link ──► velodyne     (固定, z=0.236)
      base_link ──► imu_link      (固定, 与 velodyne 共位)
      base_link ──► {left_wheel, right_wheel, 万向轮}  (轮子配合 /joint_states)
```

要点:
- **`map → odom`**:建图时**固定为静态单位阵**(run.launch)。即把 `odom` 当世界系用,全局位姿都体现在
  `odom→base_footprint`。
- **`odom → base_footprint`**:由 `imuPreintegration` 动态发布——SLAM 的实际输出位姿。轮式 `enable_odom_tf:false`
  确保这条边只有 LIO-SAM 一个发布者(避免与轮式里程计震荡)。
- **`base_footprint` 以下**:全部来自 URDF,由 `robot_state_publisher` 发布。根 link = `base_footprint`。
- ⚠️ 原版 LIO-SAM 中 `map→odom` 本是 mapOptimization 发的修正边;本仿真改用静态单位阵 + 把位姿压进
  `odom→base_footprint`。**建议用 `view_frames` 运行时实测确认**(见 §8)。

---

## 7. LIO-SAM 关键配置(契约值)

完整逐值参数见 **`core/mapping/lio-sam.patch`**(改配置的正确姿势:改 `core/mapping/LIO-SAM` working tree
→ `git diff > ../lio-sam.patch` → 提交补丁)。已在仿真侧坐实的契约值:

| 参数 | 值 | 含义 |
|---|---|---|
| `pointCloudTopic` | `points_raw` | 订阅的点云话题(对应 §3.2 adapter 输出) |
| `imuTopic` | `/imu_plugin/out` | 订阅的 IMU 话题(对应 §3.1 桥接输出) |
| `lidarFrame` | `velodyne` | lidar frame(与 adapter `output_frame` 一致) |
| `baselinkFrame` | `base_footprint` | 机器人本体 frame |
| `odometryFrame` / `mapFrame` | `odom` / `map` | 里程计/地图 frame |
| `sensor` / `N_SCAN` | `velodyne` / `16` | VLP-16 式,16 线 |
| `extrinsicTrans` / `extrinsicRot` | `[0,0,0]` / 单位阵 | **lidar↔imu 外参归零**(仿真里二者共位;真实传感器的翻转矩阵在此会让点云倒置、建图发散) |

> ⚠️ `Horizon_SCAN` 等基于 range image 的参数以补丁实际值为准;adapter 实测每圈约 1800 列
> (`ros2 topic echo /points_raw --field width`),与 VLP-16 名义分辨率一致。

---

## 8. 运行时命令对照表(看懂的关键一步)

仿真 + LIO-SAM 都起来后,在构建机逐条跑,和上文对照:

| 命令 | 预期看到 / 对照点 |
|---|---|
| `gz topic -l \| grep -E "lidar\|imu"` | Gz 侧 `/lidar/points`、`/imu`(对照 §3) |
| `ros2 topic list` | 含 `/points_raw`、`/imu_plugin/out`、`/clock`、`/cmd_vel`、`/base_controller/odom`、`/joint_states`、一堆 `/lio_sam/...` |
| `ros2 topic hz /points_raw` | ≈ 10 Hz(RTF<1 时显示约一半)(对照 §3.2) |
| `ros2 topic hz /imu_plugin/out` | ≈ 200 Hz(对照 §3.1) |
| `ros2 topic echo /points_raw --once --field fields` | 含 `x y z intensity ring time`(对照 §3.2) |
| `ros2 topic echo /points_raw --once --field header.frame_id` | `velodyne`(对照 §3.2) |
| `ros2 topic echo /imu_plugin/out --once --field header.frame_id` | `imu_link`(对照 §3.1) |
| `ros2 run tf2_ros tf2_echo base_link velodyne` | 平移 `0 0 0.236`(对照 §6;与 `base_link imu_link` 相同=共位) |
| `ros2 run tf2_tools view_frames` | TF 树 PDF,核对 §6——**重点看 `map→odom` 谁发、`odom→base_footprint` 谁发** |

> 建图前**让机器人静止几秒**,IMU 预积分/重力初始化需要静止起步。驱动用持续发布的 **TwistStamped**
> (`ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped '{header:{frame_id: base_link}, twist:{linear:{x:0.3}}}' -r 10`)。

---

## 9. 已知坑 / 易混点

1. **`/cmd_vel` 是 TwistStamped 不是 Twist**:Humble `diff_drive_controller` 默认收 TwistStamped,
   发普通 Twist 会被静默忽略、车不动。
2. **传感器经桥接 + adapter,不是 robot.sdf 插件**:Gz 原生 `gpu_lidar`/`imu` → `ros_gz_bridge` →
   (雷达再过 `lidar_pointcloud_adapter`)。话题契约名 `/points_raw`、`/imu_plugin/out` 保持不变。
3. **点云无原生逐点 `time`**:Gz 是瞬时快照。adapter 按方位角**合成** `time`;FAST-LIO 据此去畸变
   (对一份本无运动畸变的快照施加的是"伪去畸变",原地旋转若拖影,治本是让 adapter 发**常量 time**
   令去畸变成恒等,而非改 FAST-LIO);LIO-SAM 对无 time 输入自动关去畸变,**存图不受影响**。
4. **`map→odom` 是静态单位阵**:与原版 LIO-SAM(mapOptimization 发修正边)不同,本仿真简化为静态 +
   位姿压进 `odom→base_footprint`,务必用 `view_frames` 实测确认。
5. **轮式不发 TF**(`enable_odom_tf:false`):`odom→base_footprint` 建图时专属 LIO-SAM,避免双发布震荡;
   轮式只发 `/base_controller/odom` 话题(供 Nav2 用 twist)。
6. **传感器/IMU 共位**:`velodyne` 与 `imu_link` 同位(`base_link + z=0.236`),故 LIO-SAM extrinsic 归零。
7. **`controller_manager` 在 Gz 物理步里**:由 URDF 的 `gz_ros2_control` 插件提供,`ros2 node list` 应见
   `/controller_manager`,但没有独立 `ros2_control_node` 进程。

---

## 10. 换成 FAST-LIO 跑(定位栈)时的差异

定位阶段(`core/localization`)复用**完全相同的仿真前端**(§3 的 Gz 传感器 + 桥接 + adapter,
喂同样的 `/points_raw`、`/imu_plugin/out`),区别:
- LIO-SAM 不在了,换 FAST-LIO 跑里程计:发 TF `camera_init→body`、话题 `/Odometry`、`/cloud_registered`
  (已在 `camera_init` 帧)。
- `gicp_localization` 在 LIO-SAM 先验图上做 GICP,发**校正** TF `map→camera_init`,把 FAST-LIO 局部
  里程计锚定到先验图全局坐标。
- 此时 `odom→base_footprint` 没有发布者(LIO-SAM 不在、轮式又不发 TF)——FAST-LIO 用自己的
  `camera_init→body` 平行子树,导航阶段再经 `body→base_footprint` 焊接接进 URDF 树。
- 详见 `core/localization/README.md` 与 `core/navigation/README.md`。

---

*事实来源:`robot_gz.launch.py`、`bridge.yaml`、`lidar_pointcloud_adapter`、`robot_controllers.yaml`、
`factory.sdf`、`core/mapping/lio-sam.patch`、LIO-SAM 源码话题名。标注 ⚠️ 或"实测确认"的条目请以构建机实测为准。*
