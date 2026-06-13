# core/mapping — LIO-SAM 建图(先验图制作)

本模块负责用 **LIO-SAM** 在 Gz Harmonic 工厂世界里跑建图、保存先验地图 `GlobalMap.pcd`(供 localization 阶段的 GICP 用)。

## 集成方式:clone + patch(不 fork)
LIO-SAM **不纳入 core/、不被本仓库提交**,而是按 README 主文档的 pinned SHA `git clone` 到 `src/LIO-SAM`,再 `git apply` 本仓库跟踪的 `src/lio-sam.patch`。该补丁含全部 sim 适配:
- `config/params.yaml`:话题 `points_raw` / `/imu_plugin/out`、帧 `lidarFrame=velodyne`、`baselinkFrame=base_footprint`、外参归零(雷达/IMU 共位)、VLP-16 16/1800、indoor leaf、`savePCD:true`。
- `launch/run.launch.py`:发 `map→odom` 静态 TF、禁用 LIO-SAM 自带 robot_state_publisher(TF 由仿真侧提供)、起 4 个 lio_sam 节点 + RViz。
- `src/mapOptmization.cpp`:存图/行为微调。

**改 LIO-SAM 配置的正确姿势**:改 `src/LIO-SAM` working tree → `cd src/LIO-SAM && git diff > ../lio-sam.patch` 重生成 → 提交补丁。构建机重新 `git apply`(或 clone 重置后再 apply)。

## TF 约定(REP-105)
```
map ─(run.launch.py 静态)→ odom ─(LIO-SAM 激光里程计,独占)→ base_footprint
    ─(URDF 固定)→ base_link ─(robot_state_publisher)→ velodyne / imu_link / 轮
```
轮式里程计 TF 已在 `robot_controllers.yaml` 关闭(`enable_odom_tf:false`),`odom→base_footprint` 由 LIO-SAM 独占,避免被轮式抖动污染。

## 构建机:建图流程
前置:`core/` 已拷入工作区;`src/LIO-SAM` 已 clone + apply 最新 `lio-sam.patch` 并 `colcon build --packages-select lio_sam`;Phase 4 的 `models/factory_model` + `GZ_SIM_RESOURCE_PATH` 就位。

```bash
# 终端 1：起仿真(工厂世界 + 机器人 + 传感器)
cd src && source install/setup.bash
ros2 launch robot_gz_bringup robot_gz.launch.py factory_models_path:=/abs/.../models/factory_model
#（必要时 spawn_x:=/spawn_y:= 调到空旷过道）

# 终端 2：起 LIO-SAM 建图
cd src && source install/setup.bash
ros2 launch lio_sam run.launch.py

# 终端 3：缓慢遍历工厂(Humble diff_drive 收 TwistStamped！发 Twist 会被忽略)
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.4}, angular: {z: 0.0}}}' -r 10
# 转弯/绕行用 angular.z；把工厂主要通道、墙面、货架都扫到,回环走一圈利于回环检测

# 建够后存图(写到 ~/result，与主文档定位阶段读取路径一致)
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/result'}"
```

## 验收判据(PASS → 进 5c 定位)
18. 终端 2 起 LIO-SAM 后无 TF/参数报错;`ros2 topic hz /lio_sam/mapping/odometry` 持续发布。
19. RViz 里 LIO-SAM 累积的点云地图**勾勒出工厂结构**(墙/货架/集装箱清晰、不重影不发散);行驶中 `map→odom→base_footprint` TF 链完整(`ros2 run tf2_ros tf2_echo map base_footprint` 有输出且随车动)。
20. 回环闭合后地图一致(绕工厂一圈回到起点,地图不分裂/不错层)。
21. `save_map` 在 `~/result/` 生成 `GlobalMap.pcd`(及 cornerMap/surfMap);`pcl_viewer` 或 RViz 加载该 PCD 能看出完整工厂、尺度合理(与 35×18m 量级相符)。

## FAIL 排查
- LIO-SAM 起来即报 `extrinsic`/`frame` 或点云方向错乱 → 确认补丁已是最新(`lidarFrame:velodyne`、`extrinsicTrans:[0,0,0]`),且构建机重新 apply 并 `colcon build lio_sam`。
- `map→base_footprint` TF 断 → 查 `base_footprint` 是否在 URDF(5a)、`robot_state_publisher` 是否在跑、轮式 TF 是否已关(否则与 LIO-SAM 抢 odom→base)。
- 地图发散/重影 → 多为驱动太快或转太急(雷达 10Hz、RTF<1),放慢 `linear.x`、减小 `angular.z`;或工厂特征不足处(空旷区)正常,回到特征区会收敛。
- 车不动 → 八成发了 `Twist` 而非 `TwistStamped`(Humble diff_drive 默认)。
