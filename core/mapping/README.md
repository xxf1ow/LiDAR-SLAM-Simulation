# Mapping：LIO-SAM 先验图制作

本模块使用 LIO-SAM 消费 `/points_raw` 和 `/imu/data`，生成供 GICP 使用的
`~/result/GlobalMap.pcd`，并通过 `save_map.sh` 转换出 Nav2 二维占据地图。

## 上游集成

LIO-SAM 源码克隆到 `core/mapping/LIO-SAM`，该目录被 Git 忽略。项目修改只通过
`core/mapping/lio-sam.patch` 交付。

```bash
git clone <LIO-SAM upstream> core/mapping/LIO-SAM
cd core/mapping/LIO-SAM
git checkout <pinned SHA>
git apply ../lio-sam.patch
```

补丁维护两份原生配置：

- `params.yaml`：Gazebo 16×1800 点云和仿真时钟。
- `params_real.yaml`：Vanjee 722 32×1200 点云和系统时钟。

FAST-LIO/LIO-SAM 算法外参尚未迁入中央 Profile；当前配置中的兼容值不是由 URDF mount
自动推导。修改上游配置后，从 clone 工作树重新生成补丁并提交补丁文件，不提交 clone。

## TF 与接口

```text
map ── static ──> odom ── LIO-SAM ──> base_footprint ── URDF ──> base_link
```

轮式里程计的 `enable_odom_tf` 为 false，因此 mapping 模式由 LIO-SAM 独占
`odom -> base_footprint`。输入契约为 `/points_raw`、`/imu/data`、`velodyne` 和
`imu_link`。

## 正式建图

将 `core/bringup/system_bringup/config/bringup.yaml` 设置为：

```yaml
platform: sim       # 真机使用 real
mode: mapping
```

```bash
cd core
source /opt/ros/humble/setup.bash
source ~/res2_ws/install/setup.bash
source install/setup.bash
ros2 launch system_bringup bringup.launch.py
```

启动顺序为底层平台、关节状态 settling、shared sensor contract gate、LIO-SAM。sim 使用
adapter generated YAML，real 使用 Vanjee generated YAML；LIO-SAM 参数文件由
`bringup.yaml` 的平台分支选择。

手机访问 `http://<主机IP>:8080`，进入“人工接管”后驾驶建图。完整 bringup 中不要直接
发布 `/cmd_vel`。

## 保存地图

LIO-SAM 仍在运行时执行：

```bash
cd core
bash mapping/save_map.sh
```

脚本调用 `save_map` 服务，把中间结果保存在 `~/result/loam/`，复制生成
`~/result/GlobalMap.pcd`，并生成 Nav2 占据栅格。LIO-SAM 保存实现会先删除并重建目标目录，
禁止把目标直接设为 `/result` 或其他包含重要文件的父目录。

## 独立诊断

只有在排查 LIO-SAM 自身时才绕过正式入口。先确保平台驱动、TF、`/points_raw` 和
`/imu/data` 已就绪，再执行：

```bash
cd core
source install/setup.bash
ros2 launch lio_sam run.launch.py
```

真机诊断显式选择安装态 real 配置：

```bash
REAL_PARAMS="$(ros2 pkg prefix lio_sam)/share/lio_sam/config/params_real.yaml"
ros2 launch lio_sam run.launch.py params_file:="$REAL_PARAMS"
```

## 验收

- `/lio_sam/mapping/odometry` 持续发布。
- `map -> odom -> base_footprint` 连通且没有 TF 竞争。
- RViz 累积地图结构清晰，无持续重影、分裂或尺度异常。
- 回环后地图保持一致。
- `save_map.sh` 生成可加载的 `GlobalMap.pcd` 和二维地图文件。

## 排错

- 参数、frame 或点云方向错误：确认 clone 位于正确提交、最新补丁已 apply，并重新构建。
- sensor gate 不放行：检查点云字段/形状/频率、IMU 频率和消息时间戳，不要绕过 gate。
- TF 断链或跳变：确认轮式 odom TF 已关闭，LIO-SAM 是 mapping 模式唯一 odom TF 发布者。
- 地图发散：先降低车速和转速，再检查逐点时间、IMU 方向、外参和场景特征。
- 保存失败：确认 LIO-SAM 仍在线、服务存在且目标目录是专用子目录。
