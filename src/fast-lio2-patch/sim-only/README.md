# fast-lio2-patch / sim-only

此目录下的补丁**仅适用于 Gazebo 仿真**，因为仿真传感器的物理特性与真实硬件不同。
**真实雷达切勿应用**——它们会移除真实场景所需的处理。

- `disable-deskew-snapshot-lidar.patch`
  关闭 FAST-LIO 的帧内去畸变。Gazebo velodyne 是**瞬时快照**点云（所有点同一时刻、无运动畸变）；
  FAST-LIO 在缺逐点 time 时会按方位角编造时间并去畸变，旋转时凭空产生拖影/发散。本补丁强制
  `given_offset_time=true` 使去畸变成为恒等变换。
  **真实旋转式雷达扫描有运动畸变、必须去畸变，故真实场景不要应用此补丁。**

应用方式见各补丁文件头部注释。基准 commit：`a4743b0`。

> 对比：上一级 `fast-lio2-patch/` 里的补丁（如 `01-add-gazebo-velodyne-config.patch`）是
> 中性的仿真适配（新增配置文件等），对真实场景无害；本目录的补丁会**改变核心行为**，真实场景应用会出错。
