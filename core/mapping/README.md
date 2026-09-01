# Mapping：LIO-SAM 先验图制作

本模块使用 LIO-SAM 消费 `/points_raw` 和 `/imu/data`，生成 GICP 使用的 `~/result/GlobalMap.pcd`，并通过 `save_map.sh` 转换出 Nav2 二维占据地图。

## 上游集成

LIO-SAM 源码克隆到忽略目录 `core/mapping/LIO-SAM`。项目修改只通过 `core/mapping/lio-sam.patch` 交付；算法参数由 `system_bringup/config/templates/lio_sam.yaml` 和所选 Profile 编译为 `lio_sam.generated.yaml`。

```bash
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2 --single-branch --depth 1 \
  --filter=blob:none core/mapping/LIO-SAM
cd core/mapping/LIO-SAM
git fetch origin 08af3f32f01725372d4269838dc44c19c6d9e76b --depth 1
git checkout 08af3f32f01725372d4269838dc44c19c6d9e76b
git apply ../lio-sam.patch
```

## TF 与接口

```text
map ── static ──> odom ── LIO-SAM ──> base_footprint ── URDF ──> base_link
```

轮式里程计的 `enable_odom_tf` 为 false，mapping 模式由 LIO-SAM 独占 `odom -> base_footprint`。输入契约为 `/points_raw`、`/imu/data`、`velodyne` 和 `imu_link`。

## 建图与保存

把 `bringup.yaml` 设为所需 platform 和 `mode: mapping`，再通过正式入口启动。底层平台和共享传感器契约闸门通过后才启动 LIO-SAM；完整 bringup 中通过 Web 人工接管驾驶，不直接发布 `/cmd_vel`。

LIO-SAM 仍在线时从 `core/` 执行：

```bash
bash mapping/save_map.sh
```

脚本调用 `save_map` 服务，把中间结果写入 `~/result/loam/`，复制生成 `~/result/GlobalMap.pcd`，并生成 Nav2 占据栅格。LIO-SAM 会删除并重建目标目录，禁止把目标设为 `~/result/` 或其他包含重要文件的父目录。

独立诊断必须先通过 `compile_runtime_configs` 生成运行目录，再把同目录的 `lio_sam.generated.yaml` 显式传给包级 launch；该路径不替代正式 bringup。

## 验收与排错

- `/lio_sam/mapping/odometry` 持续发布，TF 链连通且没有发布者竞争。
- 累积地图无持续重影、分裂或尺度异常，回环后保持一致。
- `save_map.sh` 生成可加载的 PCD 和二维地图。
- sensor gate 失败时检查点云字段、形状、频率、IMU 和时间戳，不得绕过闸门。
- 地图发散时先降低车速和转速，再检查逐点时间、IMU 方向、外参和场景特征。
