# core/mapping — LIO-SAM 建图(先验图制作)

本模块负责用 **LIO-SAM** 在仿真或真实 Vanjee 722 数据上建图、保存先验地图
`GlobalMap.pcd`(供 localization 阶段的 GICP 用)。

## 集成方式:clone + patch,落在本模块名下(core 自成一体)
LIO-SAM 的源码 **clone 到本模块目录 `core/mapping/LIO-SAM`**(被 `.gitignore` 排除、不入库),再 `git apply` 本模块跟踪的 **`core/mapping/lio-sam.patch`**。**不放在 src 下**——core 是自成一体的完整 colcon 工作区,`colcon build` 从 `core/` 跑即可发现 `lio_sam` 包,无需 src。该补丁含全部 sim 适配:
- `config/params.yaml`:话题 `points_raw` / `/imu_plugin/out`、帧 `lidarFrame=velodyne`、`baselinkFrame=base_footprint`、外参归零(雷达/IMU 共位)、VLP-16 16/1800、indoor leaf、`savePCD:true` + `savePCDDirectory:/result/loam/`(存盘前 rm -r 此目录,故用专门子目录,勿改 /result/)。
- `config/params_real.yaml`:Vanjee 722 真机话题 `/points_raw` / `/imu/data`、32×1200、厂商 IMU/特征参数、主机时钟；IMU→LiDAR 外参暂用厂商单位阵，待实测。
- `launch/run.launch.py`:发 `map→odom` 静态 TF、禁用 LIO-SAM 自带 robot_state_publisher(TF 由外部提供)、起 4 个 lio_sam 节点 + RViz；RViz 与节点读取同一个 `params_file`。
- `src/mapOptmization.cpp`:存图/行为微调。

clone 命令(pinned SHA 见主文档,clone 到本模块):
```bash
git clone <LIO-SAM upstream> core/mapping/LIO-SAM && cd core/mapping/LIO-SAM && git checkout <pinned SHA>
git apply ../lio-sam.patch
```

**改 LIO-SAM 配置的正确姿势**:改 `core/mapping/LIO-SAM` working tree → `cd core/mapping/LIO-SAM && git diff > ../lio-sam.patch` 重生成 → 提交 `core/mapping/lio-sam.patch`。构建机重新 `git apply`(或 clone 重置后再 apply)。

## TF 约定(REP-105)
```
map ─(run.launch.py 静态)→ odom ─(LIO-SAM 激光里程计,独占)→ base_footprint
    ─(URDF 固定)→ base_link ─(robot_state_publisher)→ velodyne / imu_link / 轮
```
轮式里程计 TF 已在 `robot_controllers.yaml` 关闭(`enable_odom_tf:false`),`odom→base_footprint` 由 LIO-SAM 独占,避免被轮式抖动污染。

## 构建机:建图流程
前置:**构建根 = `core/`**(从 core 跑 colcon,build/install 落 core);`core/mapping/LIO-SAM` 已 clone + apply 最新 `lio-sam.patch`;Phase 4 的 `models/factory_model` + `GZ_SIM_RESOURCE_PATH` 就位。

```bash
# 构建(从 core 工作区根,一次建全:本仓库包 + lio_sam clone)
cd core && colcon build --packages-up-to lio_sam robot_gz_bringup
source install/setup.bash

# 终端 1：起仿真(工厂世界 + 机器人 + 传感器)
cd core && source install/setup.bash
ros2 launch robot_gz_bringup robot_gz.launch.py
#（factory_models_path 默认 ~/LiDAR-SLAM-Simulation/models/factory_model,路径不同才传该 arg；
#  必要时 spawn_x:=/spawn_y:= 调到空旷过道；默认不起看模型的 RViz——建图看终端 2 LIO-SAM 自带 RViz 即可）

# 终端 2：起 LIO-SAM 建图
cd core && source install/setup.bash
ros2 launch lio_sam run.launch.py

# 分步启动只用于底层诊断；可持续发布低速 TwistStamped 验证控制器，Ctrl+C 即停止
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.2}}}'

# 建够后存图 + 转 2D 栅格(一键脚本)
#   注意:LIO-SAM 存盘前 rm -r 目标目录重建(mapOptmization.cpp:188/414),故脚本存到专门子目录
#   ~/result/loam/ 再 cp 到 ~/result/。勿手写 destination:'/result'(会删 ~/result/ 主级地图!)
bash mapping/save_map.sh
```

## 通过 bringup 启动 mapping(替代上面手动分步)

把 `core/bringup/system_bringup/config/bringup.yaml` 的 `mode: navigation` 改为 `mode: mapping`(`platform: sim` 保持),不用 rebuild,然后:
```bash
cd core && source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```
链路:一致性闸门 → robot_gz → ready_gate[/points_raw, /joint_states] → lio_sam(4 节点 + 自带 RViz)。

- 手机访问 `http://<机器人或仿真主机IP>:8080`
- mapping：点击“人工接管”后按住方向按钮驾驶
- navigation：默认自动；点击“人工接管”屏蔽 Nav2，点击“恢复自动导航”恢复

完整 bringup 中只有 `cmd_vel_gate` 发布 `/cmd_vel`。浏览器断连且仍处于 manual
模式时，0.5 秒源超时会停车。**建够后先跑 `bash mapping/save_map.sh`**
(此时 lio_sam 仍在跑、service 在线)存盘+转换,完成后再 Ctrl+C 停栈。

- ready_gate 把"手动判断起 lio_sam 的时机"换成"等 /points_raw + /joint_states 出现 + settling",其余等价于手动分步。
- **use_sim_time**:lio_sam 与 RViz 都从所选参数文件读取；默认 `params.yaml` 使用仿真时钟，真机显式选择 `params_real.yaml` 使用主机时钟。
- **存盘**:bringup 不含存盘/转换——靠 `save_map.sh` 完成(service 存 ~/result/loam + cp + 转 occupancy)。service 要 lio_sam 在线,**必须在 Ctrl+C 停栈前跑**(见上一行)。

## Vanjee 722 真机分步启动

本阶段只提供 LIO-SAM 真机入口，不接入 `system_bringup`。先确保驱动已持续发布
`/points_raw`、`/imu/data`，并由外部提供 `base_footprint↔velodyne` TF，然后启动：

```bash
cd core && source install/setup.bash
REAL_PARAMS="$(ros2 pkg prefix lio_sam)/share/lio_sam/config/params_real.yaml"
ros2 launch lio_sam run.launch.py params_file:="$REAL_PARAMS"
```

初始配置按实测点云采用 32×1200；厂商参考中的 IMU 噪声和特征参数原样采用。
当前单位外参只是首轮联调值，待实测 IMU 轴向和安装位姿后再调整。地图仍用
`mapping/save_map.sh` 保存到现有的 `~/result/loam/` 和 `~/result/GlobalMap.pcd`。

## 验收判据(PASS → 进 5c 定位)
18. 终端 2 起 LIO-SAM 后无 TF/参数报错;`ros2 topic hz /lio_sam/mapping/odometry` 持续发布。
19. RViz 里 LIO-SAM 累积的点云地图**勾勒出工厂结构**(墙/货架/集装箱清晰、不重影不发散);行驶中 `map→odom→base_footprint` TF 链完整(`ros2 run tf2_ros tf2_echo map base_footprint` 有输出且随车动)。
20. 回环闭合后地图一致(绕工厂一圈回到起点,地图不分裂/不错层)。
21. `save_map` 在 `~/result/` 生成 `GlobalMap.pcd`(及 cornerMap/surfMap);`pcl_viewer` 或 RViz 加载该 PCD 能看出完整工厂、尺度合理(与 35×18m 量级相符)。

## FAIL 排查
- LIO-SAM 起来即报 `extrinsic`/`frame` 或点云方向错乱 → 确认补丁已是最新(`lidarFrame:velodyne`、`extrinsicTrans:[0,0,0]`),且构建机重新 apply 并 `colcon build lio_sam`。
- `map→base_footprint` TF 断 → 查 `base_footprint` 是否在 URDF(5a)、`robot_state_publisher` 是否在跑、轮式 TF 是否已关(否则与 LIO-SAM 抢 odom→base)。
- 地图发散/重影 → 多为驱动太快或转太急(雷达 10Hz、RTF<1),少按几下 `i`/`j`/`l` 降速、多用 `s` 回正;或工厂特征不足处(空旷区)正常,回到特征区会收敛。
- 车不动 → 完整 bringup 确认网页已点击“人工接管”且仍在按住方向按钮；
  分步底层诊断确认持续发布的是 `TwistStamped` 而不是 `Twist`。
