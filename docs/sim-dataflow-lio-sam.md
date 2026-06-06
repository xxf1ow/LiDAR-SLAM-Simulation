# 仿真系统消息流转与配置参考 —— LIO-SAM 配置

> 适用对象：本仓库 Gazebo 仿真 + **LIO-SAM** 一起运行时的形态。
> 配套文档（待建）：FAST-LIO2 配置下的同名参考。
>
> **怎么用这份文档**：左手读本文搞清"应该是什么样"，右手在 WSL 里把仿真跑起来，用第 8 节的运行时命令逐条对照"实际是什么样"。两者对上，你就真正看懂了。

---

## 1. 启动拓扑：两条 launch，分两步起

整套系统由**两条相互独立的 launch** 组成，需要分别在两个终端启动：

```
① 仿真本体        ros2 launch robot_gazebo robot_sim.launch.py
② SLAM 算法       ros2 launch lio_sam run.launch.py
```

**① robot_sim.launch.py**（`src/robot_gazebo/launch/robot_sim.launch.py`）拉起：
- `gzserver` + `gzclient`：加载世界 `worlds/lio_world.model`，其中引用机器人模型 `models/robot/robot.sdf`（**所有传感器/驱动插件都在这个 SDF 里**）
- `robot_control.launch.py` → `robot_control.py`（辅助节点，见 §3）
- `robot_state_publisher.launch.py` → `robot_state_publisher`（读 `urdf/robot.urdf`，发布机器人静态 TF）
- `teleop_twist_keyboard`（在 xterm 窗口里，键盘遥控发 `/cmd_vel`）

**② run.launch.py**（`src/LIO-SAM/launch/run.launch.py`）拉起：
- `static_transform_publisher`：发布 **`map → odom` 静态单位阵**
- `lio_sam_imuPreintegration`、`lio_sam_imageProjection`、`lio_sam_featureExtraction`、`lio_sam_mapOptimization`（LIO-SAM 四节点流水线）
- `rviz2`（加载 `config/rviz2.rviz`，`use_sim_time:=true`）

> 注意：run.launch.py 里 LIO-SAM 自带的 `robot_state_publisher` 被注释掉了——TF 由仿真那条 launch 统一提供，避免两边都发 URDF 静态 TF 打架。

---

## 2. 节点清单（谁是谁）

| 节点 | 来源 | 角色 |
|---|---|---|
| `gzserver`/`gzclient` | gazebo_ros | 物理仿真 + 渲染；**承载 robot.sdf 里的全部插件** |
| `robot_state_publisher` | URDF | 把 `robot.urdf` 的 link/joint 发成 TF（机器人骨架） |
| `teleop_twist_keyboard` | 标准包 | 键盘 → `/cmd_vel` |
| `robot_control.py` | robot_control | **辅助旁路**，不参与建图也不真正驱动（见 §3） |
| `lio_sam_imageProjection` | LIO-SAM | 点云去畸变、投影成 range image |
| `lio_sam_featureExtraction` | LIO-SAM | 提取角点/平面点特征 |
| `lio_sam_mapOptimization` | LIO-SAM | scan-to-map 优化、关键帧、回环、建全局图 |
| `lio_sam_imuPreintegration` | LIO-SAM | IMU 预积分，高频里程计 + 发布 `odom→base_footprint` TF |
| `static_transform_publisher` | tf2_ros | `map→odom` 静态单位阵 |
| `rviz2` | rviz2 | 可视化 |

---

## 3. Gazebo 插件配置逐项注解（robot.sdf）

这是**数据的源头**。四个插件全部定义在 `src/robot_gazebo/models/robot/robot.sdf`。

### 3.1 IMU —— `imu_plugin`（line 86–103）
```xml
<sensor name='imu_sensor' type='imu'>
  <update_rate>200</update_rate>                         <!-- 传感器 200 Hz -->
  <plugin name='imu_plugin' filename='libgazebo_ros_imu_sensor.so'>
    <topicName>imu_raw</topicName>                       <!-- ⚠️ ROS1 旧标签，ROS2 插件忽略 -->
    <frameName>base_link</frameName>                     <!-- IMU 消息 header.frame_id -->
    <gaussianNoise>0.0</gaussianNoise>                   <!-- 仿真无噪声 -->
  </plugin>
</sensor>
```
- **实际发布话题 = `/imu_plugin/out`**：ROS2 版插件用「插件名 `imu_plugin` + 默认后缀 `~/out`」组成话题名，`<topicName>imu_raw</topicName>` 这一行**不生效**（这就是历史上"IMU topic mismatch"困惑的根源）。
- **频率 200 Hz**，`frame_id = base_link`，无噪声。
- 对应 params.yaml 的 `imuTopic: "/imu_plugin/out"`。

### 3.2 LiDAR —— `gazebo_ros_laser_controller`（line 104–140）
```xml
<sensor name='velodyne-VLP16' type='ray'>
  <update_rate>10</update_rate>                           <!-- 10 Hz 转速 -->
  <ray>
    <horizontal><samples>440</samples> ...               <!-- ⚠️ 水平 440 线/圈（非 1800） -->
              <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>  <!-- 360° -->
    <vertical><samples>16</samples>                       <!-- VLP-16，16 线 -->
              <min_angle>-0.261799</min_angle><max_angle>0.261799</max_angle></vertical>  <!-- ±15° -->
  </ray>
  <plugin name='gazebo_ros_laser_controller' filename='libgazebo_ros_velodyne_laser.so'>
    <topicName>/points_raw</topicName>                    <!-- 实际就发这个 -->
    <frameName>velodyne</frameName>                       <!-- 点云 header.frame_id = velodyne -->
    <min_range>0.9</min_range><max_range>130.0</max_range>
    <gaussianNoise>0.008</gaussianNoise>
  </plugin>
</sensor>
```
- **发布话题 = `/points_raw`**，类型 `sensor_msgs/PointCloud2`，**10 Hz**，`frame_id = velodyne`。
- **点字段只有 x/y/z/intensity/ring，没有逐点 `time`**——这正是 LIO-SAM 去畸变被关、以及 FAST-LIO 要走方位角回退的原因。
- **水平 440 而非 1800**：见 §9 易混点。

### 3.3 差速驱动 —— `differential_drive_controller`（line 539–565）
```xml
<plugin filename='libgazebo_ros_diff_drive.so'>
  <update_rate>50</update_rate>
  <publish_odom>1</publish_odom>            <!-- 发 /odom 话题 -->
  <publish_odom_tf>0</publish_odom_tf>      <!-- ⚠️ 不发 TF！把 odom→base_footprint 让给 LIO-SAM -->
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```
- 订阅 `/cmd_vel`，驱动轮子，发布 `/odom`（`nav_msgs/Odometry`，50 Hz，轮式里程计）。
- **`publish_odom_tf=0` 是关键**：若同时让它发 `odom→base_footprint`，会和 LIO-SAM 的 imuPreintegration 抢同一条 TF 边，导致 frame 在 0 和巨大值之间跳变（历史 bug）。
- 注：LIO-SAM 默认**不用**轮式 `/odom`，所以这个话题在本配置里基本无人消费。

### 3.4 关节状态 —— `robot_joint_state`（line 567–574）
```xml
<plugin filename='libgazebo_ros_joint_state_publisher.so'>
  <remapping>~/out:=joint_states</remapping>   <!-- 发 /joint_states -->
  <update_rate>30</update_rate>
  <joint_name>left_wheel_joint</joint_name><joint_name>right_wheel_joint</joint_name>
</plugin>
```
- 发布 `/joint_states`（30 Hz），供 `robot_state_publisher` 计算轮子的动态 TF。

### 3.5 robot_control.py（辅助，非关键路径）
- 订阅 `/cmd_vel` → 转发到 `/motor_cmd_vel`（**无人消费**）；订阅 `/imu_plugin/out`（**读了不用**）。
- 真正的运动链路是：`teleop → /cmd_vel → diff_drive 插件`。robot_control 不影响建图，可视为遗留/占位节点。

---

## 4. 话题流转表

| 话题 | 类型 | 发布者 | 主要消费者 | 频率 |
|---|---|---|---|---|
| `/points_raw` | PointCloud2 | Gazebo velodyne 插件 | `imageProjection` | 10 Hz |
| `/imu_plugin/out` | Imu | Gazebo imu 插件 | `imageProjection`, `imuPreintegration`, robot_control | 200 Hz |
| `/cmd_vel` | Twist | teleop | Gazebo diff_drive, robot_control | 按键 |
| `/odom` | Odometry | Gazebo diff_drive | （LIO-SAM 默认不用） | 50 Hz |
| `/joint_states` | JointState | Gazebo joint_state 插件 | robot_state_publisher | 30 Hz |
| `/motor_cmd_vel` | Twist | robot_control | 无 | — |
| `lio_sam/deskew/cloud_deskewed` | PointCloud2 | imageProjection | （RViz/调试） | 10 Hz |
| `lio_sam/deskew/cloud_info` | 自定义 | imageProjection | featureExtraction | 10 Hz |
| `lio_sam/feature/cloud_info` | 自定义 | featureExtraction | mapOptimization | 10 Hz |
| `lio_sam/mapping/odometry` | Odometry | mapOptimization | imuPreintegration | ~10 Hz |
| `lio_sam/mapping/cloud_registered` | PointCloud2 | mapOptimization | RViz | ~10 Hz |
| `lio_sam/mapping/map_global` | PointCloud2 | mapOptimization | RViz（全局图） | 低频 |
| `lio_sam/mapping/path` | Path | mapOptimization | RViz | ~10 Hz |
| `odometry/imu` | Odometry | imuPreintegration | （高频预测） | 200 Hz |

> LIO-SAM 完整发布列表（源码）：`deskew/{cloud_deskewed,cloud_info}`、`feature/{cloud_info,cloud_corner,cloud_surface}`、`mapping/{odometry,odometry_incremental,path,trajectory,map_global,map_local,cloud_registered,cloud_registered_raw}`、`imu/path`。

---

## 5. LIO-SAM 内部流水线（四节点怎么串起来）

```
/points_raw ┐
            ├─► imageProjection ──cloud_info──► featureExtraction ──cloud_info──► mapOptimization
/imu_plugin/out ┘  (去畸变/投影)                  (角点/平面特征)                 (scan-to-map+回环)
       │                                                                              │
       └──────────────────► imuPreintegration ◄──────mapping/odometry────────────────┘
                            (IMU 预积分，高频)
                                  │
                                  └─► 发布 TF: odom → base_footprint
```

- **imageProjection**：用 IMU 去畸变 + 投影成 range image，输出 `cloud_deskewed` / `cloud_info`。
- **featureExtraction**：从 range image 提角点/平面点。
- **mapOptimization**：scan-to-map 因子图优化、关键帧管理、回环检测，产出全局地图与 `mapping/odometry`。
- **imuPreintegration**：融合 IMU 与 `mapping/odometry`，输出 200 Hz 高频位姿，并广播 `odom→base_footprint`。

---

## 6. TF 树（谁发布哪条边）

```
map ──(静态单位阵, run.launch 的 static_transform_publisher)──► odom
 odom ──(动态, lio_sam_imuPreintegration)──► base_footprint
   base_footprint ──(robot_state_publisher, 来自 URDF 固定关节)──► base_link
      base_link ──► velodyne_base_link ──► velodyne          (固定)
      base_link ──► imu_link                                  (固定)
      base_link ──► {front_left/right_steering→wheel, left/right_wheel}  (轮子: 部分由 joint_states 动态)
```

要点：
- **`map → odom`**：本仿真**固定为静态单位阵**（run.launch）。即把 `odom` 当世界系用，全局位姿都体现在 `odom→base_footprint`。
- **`odom → base_footprint`**：由 `imuPreintegration` 动态发布——这是 SLAM 的实际输出位姿。
- **`base_footprint` 以下**：全部来自 URDF，由 `robot_state_publisher` 发布（轮子转动部分配合 `/joint_states`）。
- diff_drive 的 `publish_odom_tf=0` 确保 `odom→base_footprint` 只有 imuPreintegration 一个发布者。
- ⚠️ 原版 LIO-SAM 中 `map→odom` 本是 mapOptimization 发布的修正边；本仿真改用静态单位阵 + 把位姿压进 `odom→base_footprint`。**这条建议用 `view_frames` 在运行时实测确认**（见 §8），是检验你理解的好切入点。

> 易混：点云 `frame_id = velodyne`（来自插件 frameName），而 params.yaml 的 `lidarFrame = velodyne_base_link`（LIO-SAM 内部用）。两者是 URDF 里相邻的两个固定 frame，相差一个固定变换。

---

## 7. params.yaml 关键参数注解

`src/LIO-SAM/config/params.yaml`（全局 `use_sim_time: true`）。

| 参数 | 值 | 含义 |
|---|---|---|
| `pointCloudTopic` | `points_raw` | 订阅的点云话题（对应 §3.2） |
| `imuTopic` | `/imu_plugin/out` | 订阅的 IMU 话题（对应 §3.1） |
| `odomTopic` | `odometry/imu` | imuPreintegration 输出的高频里程计名 |
| `lidarFrame` | `velodyne_base_link` | LIO-SAM 内部 lidar frame |
| `baselinkFrame` | `base_footprint` | 机器人本体 frame |
| `odometryFrame` / `mapFrame` | `odom` / `map` | 里程计/地图 frame |
| `sensor` | `velodyne` | 雷达类型 |
| `N_SCAN` | `16` | **线数=16**（VLP-16） |
| `Horizon_SCAN` | `1800` | 每圈水平点数（⚠️ 与 SDF 的 440 不符，见 §9） |
| `lidarMinRange` / `lidarMaxRange` | `1.0` / `1000.0` | 距离裁剪 |
| `extrinsicTrans` | `[0,0,-0.0103]` | lidar↔imu 平移外参 |
| `extrinsicRot` / `extrinsicRPY` | 单位阵 | **旋转外参=单位阵**（仿真插件已对齐 lidar/imu，真实传感器的翻转矩阵在此会让点云倒置、建图发散，故注释掉） |
| `imuAccNoise/GyrNoise/...` | 见文件 | IMU 噪声/零偏模型参数 |
| `imuGravity` | `9.80511` | 重力 |
| `odometrySurfLeafSize` 等 | `0.2/0.1/0.2` | 体素降采样（patch 调小为室内值，仿真特征更细） |
| `loopClosureEnableFlag` | `true` | 开回环 |
| `savePCD` / `savePCDDirectory` | `true` / `/Downloads/LOAM/` | 存全局地图 PCD（**这就是阶段 2 的先验地图来源**） |

---

## 8. 运行时命令对照表（看懂的关键一步）

仿真 + LIO-SAM 都起来后，在 WSL 里逐条跑，和上文对照：

| 命令 | 预期看到 / 对照点 |
|---|---|
| `ros2 topic list` | 应含 `/points_raw`、`/imu_plugin/out`、`/cmd_vel`、`/odom`、`/joint_states`、一堆 `/lio_sam/...`（对照 §4） |
| `ros2 topic hz /points_raw` | ≈ 10 Hz（对照 §3.2） |
| `ros2 topic hz /imu_plugin/out` | ≈ 200 Hz（对照 §3.1） |
| `ros2 topic info /points_raw -v` | 发布者 = gazebo，订阅者含 imageProjection（对照 §4） |
| `ros2 topic echo /points_raw --field header.frame_id` | `velodyne`（对照 §3.2 / §6 易混点） |
| `ros2 topic echo /imu_plugin/out --field header.frame_id` | `base_link`（对照 §3.1） |
| `ros2 node info /lio_sam_imageProjection` | 订阅 points_raw + imu，发布 deskew/*（对照 §5） |
| `ros2 run tf2_tools view_frames` | 生成 TF 树 PDF，核对 §6——**重点看 `map→odom` 是谁发的、`odom→base_footprint` 是谁发的** |
| `rqt_graph` | 节点-话题连线图，整体对照 §4/§5 |

> 建图前**让机器人静止几秒**，IMU 预积分/重力初始化需要静止起步。

---

## 9. 已知坑 / 易混点

1. **`<topicName>imu_raw</topicName>` 不生效**：ROS2 imu 插件实际发 `/imu_plugin/out`。别被 SDF 里那行误导。
2. **点云 frame 是 `velodyne`，不是 `velodyne_base_link`**：插件 frameName 与 params 的 lidarFrame 是两个相邻 frame。
3. **`Horizon_SCAN=1800` vs SDF 实际 440**：params 写的是 VLP-16 名义分辨率，仿真实际每圈 440 点。LIO-SAM 基于 range image，分辨率不符可能影响投影，但当前能跑；FAST-LIO 基于逐点、不依赖 `Horizon_SCAN`，对此更宽容。运行时以 `ros2 topic echo /points_raw --field width` 实测为准。
4. **没有逐点 time 字段**：LIO-SAM 因此关闭去畸变（README 已记录的限制）；FAST-LIO 会走方位角回退重建时间，仍做去畸变。
5. **`map→odom` 是静态单位阵**：与原版 LIO-SAM（mapOptimization 发修正）不同，本仿真做了简化，务必用 `view_frames` 实测确认。
6. **diff_drive 不发 TF**（`publish_odom_tf=0`）：`odom→base_footprint` 专属 imuPreintegration，避免双发布震荡。
7. **换成 FAST-LIO 跑时**：LIO-SAM 不在了、diff_drive 又不发 TF，则 `odom→base_footprint` **没有发布者**——这正是后续 FAST-LIO 集成要处理的 TF 接线点（旁路方案下 FAST-LIO 用自己的 `camera_init→body` 孤岛，不受影响）。

---

*事实来源：`robot_sim.launch.py`、`run.launch.py`、`robot.sdf`、`robot.urdf`、`params.yaml`、`rviz2.rviz`、LIO-SAM 源码话题名、`lio-sam.patch`。标注 ⚠️ 或"建议运行时确认"的条目请以实测为准。*
