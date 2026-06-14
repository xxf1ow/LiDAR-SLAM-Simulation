# core/localization — FAST-LIO 里程计 + GICP 先验图定位

本模块负责**激光-惯性里程计**与**先验图定位**,两步落地:

- **5c(✅)— FAST-LIO 里程计**:FAST-LIO2 在 Gz Harmonic 工厂世界里跑 LiDAR-IMU 里程计,
  发布 `camera_init→body` TF、`/Odometry`、`/cloud_registered`。定位栈的地基。
- **5d(本次)— GICP 先验图定位**:`gicp_localization`(本仓库自研包)把 FAST-LIO 的
  `/cloud_registered` 在 5b 先验图(`~/result/GlobalMap.pcd`)上做 scan-to-map 配准,发布**校正**
  `map→camera_init`,把 FAST-LIO 的局部里程计锚定到先验图全局坐标。引擎 = koide3 `small_gicp`。

运行时定位栈 = 两者叠加,TF 链:`map ─[gicp]→ camera_init ─[FAST-LIO]→ body`。

## 集成方式:clone + patch,落在本模块名下(core 自成一体)
FAST-LIO 源码 **clone 到本模块目录 `core/localization/FAST_LIO`**(被 `.gitignore` 排除、不入库),
再 `git apply` 本模块跟踪的两个补丁。**不放在 src 下**——core 是自成一体的完整 colcon 工作区。

- `fast-lio2.patch`(**中性**,对真实无害):新增 `config/gazebo_velodyne.yaml`——话题
  `points_raw` / `/imu_plugin/out`、velodyne `lidar_type:2`、`scan_line:16`、blind 1.0、
  **lidar-IMU 外参归零** `extrinsic_T:[0,0,0]`(新模型 velodyne 与 imu_link 同位置,与 mapping 一致)。

> **去畸变说明**:`lidar_pointcloud_adapter` 已给点云补 `time` 字段(按列号合成 `(i%width)/width*0.1`),
> FAST-LIO 检测到非零 time 即自动 `given_offset_time=true`、用该 time 去畸变。注意这是**合成**的方位角
> 时间(Gz 是瞬时快照、无真实帧内畸变),FAST-LIO 仍会按它做一次去畸变。若构建机原地旋转测试(判据 25)
> 出现拖影/发散,治本是让 adapter 发**常量 time**(令去畸变成恒等),而非在 FAST-LIO 侧打补丁。

`livox_ros_driver2`(本目录内,**入库**)是仅含 `CustomMsg`/`CustomPoint` 的**消息桩包**,只为满足
FAST-LIO 在 velodyne-only 配置下的编译期类型依赖——无驱动、不需 Livox-SDK。

clone + apply(从仓库根):
```bash
git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 --single-branch --depth 1 \
  --filter=blob:none core/localization/FAST_LIO
cd core/localization/FAST_LIO
git fetch origin a4743b095409588842a5b30ddfa27e29d2f99164 --depth 1
git checkout a4743b095409588842a5b30ddfa27e29d2f99164
git apply ../fast-lio2.patch
```

**改 FAST-LIO 配置的正确姿势**:改 `core/localization/FAST_LIO` working tree → `git diff > ../fast-lio2.patch`
重生成 → 提交补丁。构建机重新 `git apply`。

## GICP 定位(5d):自研包入库 + small_gicp clone
- **`gicp_localization/`(本目录内,入库)** 是**本仓库自研**的 GICP 先验图定位包,非上游——故**直接迁入、
  保留 git 历史**(自 `src/` `git mv` 过来),原样复用、不重写(架构本就干净):单节点 + 双定时器
  (`MultiThreadedExecutor` + 两 callback group);慢定时器 `localization_freq`(0.5Hz)跑 GICP,
  快定时器 `tf_pub_freq`(50Hz)广播 `map→camera_init` + 发 `/localization`;GICP `align()` 在锁外跑、
  重定位不阻塞 50Hz TF。接受门**只看 fitness**(`fitness≥fitness_threshold`,默认 0.9);诊断是**编译期开关**
  (`-DGICP_DIAGNOSTICS=ON` 选 `diagnostics_real.cpp`,默认 `_null.cpp`,主逻辑零 `#ifdef`)。
  配置 `config/gicp_localization.yaml` 新栈**零改动即对**:`map`/`camera_init`/`body` 帧、`/cloud_registered`+
  `/Odometry`、`~/result/GlobalMap.pcd`、`fitness_threshold:0.9`、`use_sim_time:true`。
- **`small_gicp`(clone 到 `core/localization/small_gicp`,gitignore)** 是上游 GICP 库(无补丁),
  colcon 按 `find_package(small_gicp)` 在 core 工作区先建它再建 `gicp_localization`(位置无关,免改 CMake)。
  需 `sudo apt install libomp-dev`。

clone small_gicp(从仓库根;LFS 大文件跳过不影响编译):
```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/koide3/small_gicp.git --depth 1 \
  --filter=blob:none core/localization/small_gicp
cd core/localization/small_gicp
git fetch origin 78f2e7a221720625eb95271ad9da21a04fb77f86 --depth 1
git checkout 78f2e7a221720625eb95271ad9da21a04fb77f86
```

## TF 约定:定位子树是**平行子树**,不接进 URDF 树
```
map ─[gicp 校正,50Hz]→ camera_init ─[FAST-LIO 里程计]→ body     ← 定位子树(map=5b 先验图坐标)
base_footprint ─(URDF)→ base_link ─→ velodyne/imu_link/轮             ← 机器人 URDF 树
```
- 定位子树 `map→camera_init→body` **不焊接进机器人 URDF 树**(把 `body` 焊到 `base_footprint`
  供 Nav2 用,是 5e 阶段的事;届时按更优架构重做,不照搬旧栈魔数焊接)。5d 验证定位子树自身即可。
- 运行时 **LIO-SAM 不跑**,轮式里程计 TF 仍关(`enable_odom_tf:false`),故没有 `odom→base_footprint`;
  机器人 URDF 树暂"悬空",**RViz 关于 `base_footprint` 无父帧的告警是预期的、非 bug**。
- `/cloud_registered` 已在 `camera_init` 帧,故 GICP 的 `T_target_source` **直接就是** `T_map→camera_init`;
  GICP 是局部配准——大偏移 / ~90° 不会自动恢复(那是后续全局重定位的事),靠正确 `/initialpose` 引导。

## 构建机:验证流程
前置:**构建根 = `core/`**;`FAST_LIO` 已 clone+apply、`small_gicp` 已 clone;`libomp-dev` 已装;
Phase 4 `models/factory_model` + `GZ_SIM_RESOURCE_PATH` 就位;**5b 先验图在 `~/result/GlobalMap.pcd`**(5d 需要)。

### 5c — FAST-LIO 里程计(已通过)
```bash
cd core && colcon build --packages-up-to fast_lio robot_gz_bringup && source install/setup.bash
ros2 launch robot_gz_bringup robot_gz.launch.py                                   # 终端1 仿真(默认不起看模型的 RViz;要看传 rviz:=true)
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true  # 终端2 里程计
ros2 run robot_gz_bringup sticky_teleop.py                                        # 终端3 开车
```

### 5d — GICP 先验图定位(在 5c 跑通的基础上加一个终端)
```bash
# 构建(从 core 工作区根;small_gicp 先建,再 gicp_localization)
cd core && colcon build --packages-up-to gicp_localization && source install/setup.bash
# 终端 4：起 GICP 定位(先验图默认 ~/result/GlobalMap.pcd,已 expanduser,免传)
cd core && source install/setup.bash
ros2 launch gicp_localization localization.launch.py
#   初值默认单位阵(本仿真原点起步即可);偏了用 RViz「2D Pose Estimate」(/initialpose)重设
#   诊断(可选):colcon build --packages-select gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON
#               → ros2 topic echo /gicp_localization/diagnostics 看 fitness/converged
```

## 验收判据
**5c(✅)**
22. FAST-LIO 起来无 `extrinsic`/`frame`/话题报错;`ros2 topic hz /Odometry` 持续发布。
23. `tf2_echo camera_init body` 有输出且随车动;RViz `/cloud_registered` 勾勒工厂、不重影不发散。
24. 里程计贴合真实运动(直线+转弯轨迹吻合、尺度正确;本仿真特征丰富基本不漂)。
25. 原地转一圈点云保持清晰(若发散见"去畸变说明",治本在 adapter 发常量 time)。

**5d(PASS → 进 5e Nav2)**
26. `gicp_localization` 起来无报错、加载先验图成功(日志报点数),`~/prior_map`(frame `map`)latched
    可在 RViz 显示;`/localization` 持续发布。
27. `tf2_echo map camera_init` 有输出且**稳定**(锁定后是个近似不变的小校正,不跳变、不发散);
    `map→camera_init→body` 全链通(`tf2_echo map body` 随车动)。
28. RViz 里实时 `/cloud_registered` 与先验图 `~/prior_map` **贴合对齐**(机器人定位正确);
    fitness ≥ 0.9(`-DGICP_DIAGNOSTICS=ON` 时看 `~/diagnostics`)。
29. **引导 + 抗假解**:初值偏时 RViz「2D Pose Estimate」重设后 GICP 锁定(fitness 跳到 ≈1.0);
    工厂 90° 伪对称的假最小值(fitness≈0.8)被 `fitness_threshold:0.9` 拒绝、不误锁。

## FAIL 排查
- FAST-LIO 起来即报 `extrinsic`/`frame` 或点云方向错乱 → 确认 `fast-lio2.patch` 已 apply、
  `config_file:=gazebo_velodyne.yaml` 传对,且构建机重新 `git apply` + `colcon build fast_lio`。
- 编译报 `livox_ros_driver2` 类型缺失 → 桩包未建:`--packages-up-to fast_lio` 会先建 `livox_ros_driver2`,
  确认它在 `core/localization/` 下且被 colcon 发现。
- `/Odometry` 不发 / 节点静默 → 查 `points_raw`(经 lidar_pointcloud_adapter 补 ring/time)与
  `/imu_plugin/out` 是否在发(`ros2 topic hz`),use_sim_time 是否 true(否则等不到 /clock)。
- 旋转拖影/发散 → FAST-LIO 对 Gz 快照云按 adapter 合成 time 去畸变所致;治本=让 lidar_pointcloud_adapter
  发常量 time(快照云无帧内畸变,令去畸变成恒等),而非改 FAST-LIO。
- 车不动 → 见 mapping/README 同条(teleop 终端焦点 / 默认 stamped+sim_time)。
- GICP 编译报 `small_gicp` 找不到 → 未 clone 或 colcon 没发现:确认在 `core/localization/small_gicp`,
  `--packages-up-to gicp_localization` 会先建它;`OpenMP` 缺 → `sudo apt install libomp-dev`。
- GICP 起来即报加载先验图失败 → `~/result/GlobalMap.pcd` 不存在(先跑完 5b 建图存图)或路径未展开
  (默认已 expanduser;手动传时确保 shell 展开了 `~`)。
- `map→camera_init` 不发 / 一直拒绝(fitness 低)→ ① FAST-LIO 的 `/cloud_registered`/`/Odometry` 是否在发;
  ② 初值差太远,用 `/initialpose` 在机器人真实 map 位姿附近重设(GICP 局部、大偏移不自恢复);
  ③ 若锁在 fitness≈0.8 的 90° 假解,确认 `fitness_threshold:0.9` 没被调低。
