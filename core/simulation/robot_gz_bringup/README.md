# Phase 1 — 差速骨架 + Gz/mock/real 切换 构建机验收协议

**目的:** 坐实"同一份 xacro 三态切换 + Gz Harmonic 里差速机器人能被 /cmd_vel 驱动"。

**前置:** 构建机 Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic + `ros-humble-ros-gz` + `ros-humble-gz-ros2-control` + `ros-humble-ros2-control` + `ros-humble-ros2-controllers`；`core/` 已拷到工作区(colcon 工作区根)。

## 1. 构建全部包
```bash
colcon build --packages-select robot_hardware robot_description robot_bringup robot_gz_bringup
source install/setup.bash
colcon test --packages-select robot_description && colcon test-result --verbose   # xacro 三态解析全 PASS
```

## 2. RViz 看模型(无控制)
```bash
ros2 launch robot_description view_robot.launch.py
```
预期:RViz 里看到盒身 + 两轮 + 两万向轮;joint_state_publisher_gui 拖动轮关节模型随动。

## 3. mock 硬件驱动(无 Gz)
```bash
ros2 launch robot_bringup robot.launch.py use_mock_hardware:=true gui:=false
# 另一终端：
ros2 control list_controllers      # base_controller、joint_state_broadcaster = active
# Humble 的 diff_drive_controller 默认收 TwistStamped(发 Twist 会被忽略、车不动)
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.2}}}' -r 10
ros2 run tf2_ros tf2_echo odom base_link     # odom->base_link 随命令前进
```

## 4. Gz Harmonic 仿真驱动(核心)
```bash
ros2 launch robot_gz_bringup robot_gz.launch.py
# 另一终端：
ros2 control list_controllers      # 两控制器 active(controller_manager 由 gz 插件提供)
# Humble 的 diff_drive_controller 默认收 TwistStamped(发 Twist 会被忽略、车不动)
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.3}, angular: {z: 0.4}}}' -r 10
```
预期:Gz GUI 3D 窗口里机器人沿弧线前进;`ros2 topic echo /base_controller/odom --field pose.pose.position` 数值变化。
> RViz 的 Fixed Frame 默认是 `base_link`(跟车),看不出平移;要在 RViz 看运动把 Fixed Frame 改成 `odom`。

## 5. 裁决

**PASS(Phase 1 完成、进 Phase 2)需全部满足:**
1. 四个包 `colcon build` 全绿;`robot_description` 三态 xacro 测试全 PASS。
2. `view_robot` 在 RViz 里模型正确显示。
3. mock 模式:两控制器 active,发 `/cmd_vel` 后 `odom->base_link` 变换随动。
4. **Gz 模式:机器人在 Gazebo Harmonic 里被 `/cmd_vel` 驱动移动(直线 + 转向),控制器 active,里程计/ TF 输出正常。**
5. 同一 `robot.urdf.xacro` 仅靠 arg 完成 gz/mock/real 三态切换(real 分支编译且插件登记,真机通信留作后续)。

**FAIL(回流程，按 systematic-debugging):**
- Gz 模式机器人不动:**先确认发的是 `TwistStamped` 不是 `Twist`**(Humble 默认,发错类型会被静默忽略);再查 `/cmd_vel` 是否到 `base_controller`(remap 是否生效)、Gz 是否暂停(左下 RTF>0)、轮子是否打滑(empty.sdf 地面摩擦)、关节命令接口是否 velocity。记录 `ros2 topic info /cmd_vel -v`、`ros2 control list_hardware_interfaces` 回报。
- 控制器起不来:查 controller_manager 是否由 gz 插件起(`ros2 node list` 应有 `/controller_manager`)、`gz_controllers_file` 路径是否注入正确(看展开后的 URDF `<parameters>` 是否绝对路径)。
- xacro 测试 FAIL:看 `colcon test-result --verbose` 的断言与 check_urdf 报错。

### Phase 2 追加判据(参数化真车尺寸)
跑 `ros2 launch robot_description view_robot.launch.py` 或 `robot_gz_bringup robot_gz.launch.py`,确认:
6. 车体可见尺寸 ≈ 0.75(长)×0.55(宽)×0.40(高) m;两驱动轮在两侧、前后两万向球触地,车落在地面不下陷/不弹飞。
7. **雷达 puck(黑色圆柱)完整露在车顶中央之上,未埋进车体**。
8. TF:`ros2 run tf2_ros tf2_echo base_link velodyne` 与 `base_link imu_link` 的平移**完全相同**(共位,xyz=0 0 0.236)。
9. 里程计:发 TwistStamped 后 `ros2 topic echo /base_controller/odom` 的 `child_frame_id` = `base_link`(本阶段未加 base_footprint);提速后(linear 可达 1.5 m/s)行驶不再"痛苦地慢"。
10. 已知:`kdl_parser: root link base_link has an inertia` 警告**仍在**(根仍是 base_link,符合示例;base_footprint 留 Phase 5)——无害,勿误判为回归。

### Phase 3 验收(Gz 原生传感器 + 桥接 + ring/time)
构建(adapter 需一起):
```bash
colcon build --packages-select lidar_pointcloud_adapter robot_description robot_bringup robot_gz_bringup
source install/setup.bash
ros2 launch robot_gz_bringup robot_gz.launch.py
```
另一终端核验:
```bash
gz topic -l | grep -E "lidar|imu"          # 确认 Gz 侧实际话题名(预期 /lidar/points、/imu)
ros2 topic list | grep -E "points_raw|imu_plugin|lidar"
ros2 topic echo /lidar/points --once --field height   # 16(组织化)
ros2 topic echo /lidar/points --once --field width    # ~1800
ros2 topic echo /points_raw --once --field fields      # 含 x y z intensity ring time
ros2 topic hz /points_raw                              # ~10 Hz(sim 时钟下显示约一半正常)
ros2 topic hz /imu_plugin/out                          # ~200 Hz
ros2 run tf2_ros tf2_echo base_link velodyne           # 平移 0 0 0.236(preserveFixedJoint 保住了帧)
```
喂 FAST-LIO:
```bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
ros2 topic hz /Odometry                                # 持续发布、不崩
ros2 topic list | grep -E "Odometry|cloud_registered"
```
开车看跟踪:发 TwistStamped 驱动,RViz 里 /cloud_registered 随运动累积、/Odometry 跟随。

**PASS(进 Phase 4)需:**
11. `gz topic -l` 能看到 lidar/imu 话题;`/points_raw` 字段含 `ring`+`time`、稳定 ~10 Hz;`/imu_plugin/out` ~200 Hz。
12. `base_link→velodyne`/`imu_link` TF 在(preserveFixedJoint 生效,帧未被并掉)。
13. FAST-LIO 无 preprocess 崩溃,持续发 `/Odometry`;开车时位姿跟随、`/cloud_registered` 合理累积。

**FAIL 排查:**
- `gz topic -l` 无 lidar 点云 / `/points_raw` 不发:查 Gz 实际 sensor 话题名是否被加了模型作用域前缀(spawn 模型可能)——若是,改 `config/bridge.yaml` 的 `gz_topic_name` 与 launch 里 adapter 的 `input_topic` 为实际名(**不改算法**)。
- 点云 `height!=16`:查 gpu_lidar 的 vertical samples;`fields` 无 `ring` 时 adapter 仍按 ring=i//width 重算。
- TF 无 `base_link→velodyne`:preserveFixedJoint 未生效 → 查展开 URDF 是否含该 `<gazebo reference="velodyne_joint">` 段、版本是否支持。
- 传感器无数据但话题在:多半世界缺 `gz-sim-sensors-system`/`ogre2`(headless 要 `--headless-rendering`)。

## 已知注意
- Phase 1 用参考 DiffBot 的微小尺寸(轮距 0.10 m),Gz 里机器人很小、移动距离小,看 odom 数值确认即可;真车尺寸在 Phase 2。
- Gz 模式 RTF 可能 <1,`/cmd_vel` 用持续发布(`-r 10`)而非 `--once`。
- 若 Gz 里轮子打滑导致直线跑偏,记为 Phase 2 摩擦/惯量调参项,不在 Phase 1 阻塞判据(只要能被驱动移动即 PASS)。

## Phase 4 — 工厂世界(Classic → Harmonic 迁移)

### 资产前置(构建机)
`lio_world.model` 引用的 mesh 资产是 Classic 模型库(ARIAC/gazebo_models),按依赖惯例 **不入库**(`.gitignore: models/*`)。构建机需备好 `models/factory_model/`(整套 `model.config`+`model.sdf`+meshes,~30MB),并让 Gz 能解析 `model://`:
```bash
# 把 models/factory_model 的绝对路径告诉 Gz(二选一)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/abs/path/to/models/factory_model
# 或启动时传 arg(见下),launch 会 AppendEnvironmentVariable
```

### 重新生成世界(本机/任意机,纯 Python)
`worlds/factory.sdf` 是由 `scripts/convert_classic_world.py` 从 `src/robot_gazebo/worlds/lio_world.model` 机械生成并入库的(删 `<state>`、删带 ObjectDisposalPlugin 的模型、删内嵌老 robot include、剥 `frame=''`、中和 Ogre1 script 材质为纯色、补地面、注入 Harmonic 系统插件)。源世界变了就重跑:
```bash
python core/simulation/robot_gz_bringup/scripts/convert_classic_world.py \
  src/robot_gazebo/worlds/lio_world.model \
  core/simulation/robot_gz_bringup/worlds/factory.sdf
python -m pytest core/simulation/robot_gz_bringup/scripts/test_convert_classic_world.py -v
```

### 构建 & 启动
```bash
colcon build --packages-select lidar_pointcloud_adapter robot_description robot_bringup robot_gz_bringup
source install/setup.bash
# factory 世界(默认);资产路径用 arg 传(或已 export GZ_SIM_RESOURCE_PATH)
ros2 launch robot_gz_bringup robot_gz.launch.py \
  factory_models_path:=/abs/path/to/models/factory_model
# 回退冒烟:world:=test_world.sdf
# 机器人 spawn 默认 x=4,y=0,z=0.33;若落在结构里,调 spawn_x:= / spawn_y:=
```

### PASS(进 Phase 5)需:
14. Gz GUI 里**工厂完整加载**:墙体/workcell/集装箱/货架/托盘/管材等就位,布局与老 Classic 工厂一致(90° 伪对称在);控制台无 "unable to find model"(资产路径正确)、无致命解析错。
15. 机器人 spawn 在空旷过道、不与结构重叠、落地不下陷/弹飞(必要时调 `spawn_x/spawn_y`)。
16. 传感器照常:`/points_raw` ~10Hz 含 ring+time、`/imu_plugin/out` ~200Hz;点云在 RViz 里勾勒出工厂结构(不是空场)。
17. 开车跑一段:FAST-LIO `/Odometry` 持续、`/cloud_registered` 合理累积出工厂特征。

### FAIL 排查:
- "unable to find model [workcell]..." → `GZ_SIM_RESOURCE_PATH` 没指到 `models/factory_model`(父目录),或资产没拷到构建机。
- 模型变灰/无纹理 → 基本体盒子的 Ogre1 script 已被中和成灰色(预期,对 LiDAR 无影响);带纹理 mesh(workcell/dumpster/集装箱等)仍应有色。
- 世界加载极慢/卡 → workcell mesh 较大属正常;若超时先 `world:=test_world.sdf` 验证管线再排查。
- spawn 进了某模型里 → 调 `spawn_x:= / spawn_y:=` 到 GUI 中可见的空地。
- 机器人穿地坠落 → 确认 factory.sdf 含 ground model(转换器 ensure_ground 应已补);若用了旧的 factory.sdf 请重新生成。
