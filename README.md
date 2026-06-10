## 📌 项目约束与环境要求

> [!CAUTION]
> - **系统**: Ubuntu 22.04 (支持 WSL2)
> - **ROS 版本**: ROS 2 Humble (LTS)
> - **Gazebo**: Gazebo Fortress (LTS) 或 Gazebo 11

---

## 运行环境 - WSL2

### 安装 WSL2

```bash
# 1. 下载并安装最新版本 WSL 安装程序
# https://github.com/microsoft/wsl/releases

# 2. 检查 WSL 版本
wsl --version

# 3. 查看可安装的 linux 系统版本
wsl --list --online

# 4. 创建容器
wsl --install -d Ubuntu-22.04 --name slam

# 5. 进入容器
wsl -d slam
```

### 设置 NVIDIA 显卡加速

```bash
# 1. 进入容器
wsl -d slam

# 2. 确认 NVIDIA 显卡驱动是否正常
# https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_on_WSL2.html
nvidia-smi

# 3. 将当前用户加入 video 和 render 组
sudo usermod -aG video,render $USER

# 4. 更新 mesa 驱动
sudo add-apt-repository ppa:kisak/turtle
sudo apt update
sudo apt install mesa-vulkan-drivers libgl1-mesa-dri mesa-utils
sudo apt install --reinstall libgl1-mesa-dri libglx-mesa0 libgl1 libglapi-mesa mesa-vulkan-drivers

# 5. 设置使用 NVIDIA + D3D12
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc
source ~/.bashrc

# 6. 检查 renderer, 预期输出: OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)
glxinfo | grep "OpenGL renderer"
```

## 运行环境 ROS2 + Gazebo

```bash
# 参考链接: https://docs.ros.org/en/humble/Installation.html
# 参考链接: https://gazebosim.org/docs/fortress/ros_installation
# 参考链接: https://github.com/fishros/install

# 1. 确保系统使用 UTF-8 编码
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # 验证设置

# 2. 添加 Ubuntu Universe 软件源和 ROS 2 GPG 密钥
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. 添加 ROS2 软件源
# - $(dpkg --print-architecture) 自动检测系统架构（如 amd64）
# - $(. /etc/os-release && echo $UBUNTU_CODENAME) 自动获取 Ubuntu 版本代号（jammy）
# (二选一) 官方源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# (二选一) 清华镜像源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. 安装 ROS 2 Humble 桌面版 (含 ROS、RViz、演示、教程)
sudo apt update && sudo apt install ros-humble-desktop ros-dev-tools

# 5. 配置环境变量 (每次打开终端自动生效)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 6. 验证安装, 预期输出: humble
echo $ROS_DISTRO

# 7. 安装 Gazebo
sudo apt-get install ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gazebo-ros-pkgs

# 8. 验证安装
gazebo --version
```

## 建图模块 LIO-SAM

- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM

### 准备项目文件

```bash
# 1. PPA 方式安装 GTSAM 依赖库
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev

# 2. 下载 cache zip, 解压到根目录
curl -C - -O https://github.com/xxf1ow/LiDAR-SLAM-Simulation/releases/download/cache-zip/cache.zip

# 3. 克隆 velodyne_simulator
git clone https://github.com/ToyotaResearchInstitute/velodyne_simulator --depth 1 --filter=blob:none

# 4. 克隆 LIO-SAM 并应用补丁
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2 --single-branch --depth 1 --filter=blob:none 
cd LIO-SAM
git fetch origin 08af3f32f01725372d4269838dc44c19c6d9e76b --depth 1
git checkout 08af3f32f01725372d4269838dc44c19c6d9e76b
git apply ../lio-sam.patch

# 5. 把工厂模型拷到 Gazebo 资源目录
mkdir ~/.gazebo/models
cp -r models/factory_model/* ~/.gazebo/models/

# 其它:
# 0. patch 文件编码格式: 必须是 UTF-8 + Unix(LF)
# 1. 生成 patch 文件: git diff --no-color > ../lio-sam.patch
# 2. 检查 patch 文件: git apply --check ../lio-sam.patch
# 3. 检查工作区状态: git status
# 4. 重置工作区更改: git reset --hard HEAD
```

### 编译运行

```bash
# 1. 编译
# 重新编译指定包: colcon build --packages-select lio_sam
cd src
colcon build

# 2. 终端一运行 (Gazebo Simulation)
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py

# 3. 终端二运行 (LIO-SAM + Rviz2) 
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch lio_sam run.launch.py

# 4. 终端三运行: 保存点云结果
# 调用命令: ros2 service call
# 服务名称: /lio_sam/save_map
# 消息类型: lio_sam/srv/SaveMap
# 请求参数: "{resolution: 0.2, destination: /result}"
# 最终保存路径: ~/result
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: /result}"
```

### 验收标准

> [!NOTE]
> 验收为定性标准。仿真在 WSL2 中运行，`ros2 topic hz` 量到的话题频率会随 Gazebo 实时因子 (RTF ≈ 0.5) 成比例减半，属正常现象（仿真时间内频率正确）。

- **编译运行**：`colcon build` 通过；两条 launch 分别起 Gazebo 与 LIO-SAM + RViz2，无报错。
- **数据流**：`ros2 topic list` 含 `points_raw`、`/imu_plugin/out`、`/lio_sam/...`；点云 ≈ 10 Hz、IMU ≈ 200 Hz（按 RTF 折算）。
- **建图质量**：RViz2（Fixed Frame = `map`）中 `/lio_sam/mapping/cloud_registered` 与 `/lio_sam/mapping/path` 随遥控移动逐步拼出**一致地图**，墙面/物体单层、不重影、不发散。
- **全局一致**：绕场一圈后地图能闭合、不漂飞（已启用回环）。
- **保存地图**：`/lio_sam/save_map` 服务能在 `~/result` 写出可用的 PCD（即后续 FAST-LIO2 定位所用的先验地图来源）。
- 注：实时 TF 的 `base_footprint` 会缓慢漂移（见文末「已知限制」），但**保存的地图正确**，不计为失败。

## 定位模块 FAST-LIO2

- FAST-LIO2: https://github.com/hku-mars/fast_lio
- GICP: https://github.com/koide3/small_gicp

### 准备项目文件

```bash
# 1. 安装 perception_pcl 依赖库
sudo apt install ros-${ROS_DISTRO}-perception-pcl

# 2. 克隆 FAST_LIO2
git clone https://github.com/hku-mars/FAST_LIO.git -b ROS2 --single-branch --depth 1 --filter=blob:none
cd FAST_LIO
git fetch origin a4743b095409588842a5b30ddfa27e29d2f99164 --depth 1
git checkout a4743b095409588842a5b30ddfa27e29d2f99164

# 3. 拉取 ikd-Tree 子模块
git submodule update --init --recursive

# 4. 应用补丁
# 应用补丁: 仿真配置 + (仅仿真) 关闭快照点云去畸变
git apply ../fast-lio2-patch/01-add-gazebo-velodyne-config.patch
git apply ../fast-lio2-patch/sim-only/disable-deskew-snapshot-lidar.patch
cd ..

# 注: 消息桩包 livox_ros_driver2 (src/livox_ros_driver2) 已随仓库提供，仅用于满足
#     FAST-LIO 的编译期依赖 (velodyne-only 场景无需安装 Livox-SDK)。
```

### 编译运行

```bash
# 1. 编译
# colcon build --packages-up-to fast_lio 会先建 livox_ros_driver2 桩包再建 fast_lio
cd src
colcon build --packages-up-to fast_lio

# 2. 终端一运行 (Gazebo Simulation)
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py

# 3. 终端二运行 (FAST_LIO2 + Rviz2) 
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
```

### 验收标准

> [!NOTE]
> 当前为**阶段 1：纯里程计**（不加载先验地图，先验地图定位是后续 GICP 阶段）。纯 LIO 里程计**缓慢漂移属正常**，判据**不是**"估计位姿与真实位姿长期完全重合"。

- **编译运行**：`colcon build --packages-up-to fast_lio` 通过（先建 `livox_ros_driver2` 桩包、再建 `fast_lio`，**无 livox 相关报错**）；launch 起节点 + RViz2。
- **起步**：启动头几秒**保持机器人静止**（FAST-LIO 需静止初始化陀螺零偏与重力方向）。
- **建图质量**：RViz2（Fixed Frame = `camera_init`）给 `/cloud_registered` 显示项设较大 `Decay Time` 后，累积点云**单层、不重影、不发散**；`/Odometry` 轨迹形状正确、绕圈回到起点附近。
- **旋转无拖影**：原地转圈不再产生拖影/发散（依赖已应用仅仿真去畸变补丁 `disable-deskew-snapshot-lidar.patch`）。
- **漂移有界**：长时间缓慢漂移可接受，但**无突跳、不发散**。

---

## 定位增强 GICP（small_gicp）

> [!NOTE]
> 阶段 2：在 LIO-SAM 先验地图中用 GICP 校正 FAST-LIO2 里程计漂移（A 模式，手动初值）。需先用 LIO-SAM 的 `save_map` 生成先验地图 PCD（`~/result`）。

### 准备项目文件

```bash
# 1. 安装依赖 (OpenMP；Eigen/PCL 随 ROS-desktop / perception-pcl 已装)
sudo apt install libomp-dev

# 2. 克隆 small_gicp (跳过 LFS 大文件不影响编译)
export GIT_LFS_SKIP_SMUDGE=1    # linux (跳过 LFS 文件下载)
$env:GIT_LFS_SKIP_SMUDGE=1	    # Windows PowerShell (跳过 LFS 文件下载)
git clone https://github.com/koide3/small_gicp.git --depth 1 --filter=blob:none
cd small_gicp
git fetch origin 78f2e7a221720625eb95271ad9da21a04fb77f86 --depth 1
git checkout 78f2e7a221720625eb95271ad9da21a04fb77f86
```

### 编译运行

```bash
# 1. 编译 (定位节点依赖 small_gicp，colcon 自动先建 small_gicp)
cd src
colcon build --packages-up-to gicp_localization
# 调试带诊断话题:
# colcon build --packages-up-to gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON
# colcon build --packages-select gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON

# 2. 终端一: Gazebo
ros2 launch robot_gazebo robot_sim.launch.py

# 3. 终端二: FAST-LIO2 里程计
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true

# 4. 终端三: GICP 定位 (先验地图路径按实际)
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch gicp_localization localization.launch.py prior_map_path:=~/result/GlobalMap.pcd

# 5. 终端四：查看 GICP 日志打印
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 topic echo /gicp_localization/diagnostics

# 6. Rviz2 使用手册
#    - 左下 Global Options → Fixed Frame 改成 map。
#    - 左下 Add → By topic → /gicp_localization/prior_map 的 PointCloud2 → OK。
#    - 把 /gicp_localization/prior_map 的 PointCloud2 的 Topic → Durability Policy 改成 Transient Local（否则订阅不到 latched 的地图）。 颜色建议设成灰/白（Color Transformer = FlatColor）。
#    - 左下 Add → By topic → /cloud_registered 的 PointCloud2。 把它的 Decay Time 设大（如 0 表示一直留；或 5）、颜色设成醒目色（按 intensity 或 红）。
#    - 现在画面里：灰色=先验地图，彩色=当前帧云。 A 达标 = 开车时彩色云始终贴在灰色地图上、不漂走。
#    - 设置初始位姿 `/initialpose` 就是顶部工具栏的「2D Pose Estimate」按钮（不是单独叫 initialpose）。点它 → 在地图上按住左键点机器人应在的位置、拖动定朝向、松开，就发出 /initialpose。默认话题即 /initialpose（菜单 Panels → Tool Properties 里可确认 "2D Pose Estimate" 的 Topic）。

# 7. 模拟撞击机器人
#    - 先看仿真提供了哪些服务：
#        - ros2 service list | grep -iE "wrench|entity|state"
#    - 温和撞击 / 连续推移（演示阶段 2 正向能力——能自动跟住）：对车体施加一个短促的力。
#        - 若有 /apply_link_wrench（gazebo_msgs/ApplyLinkWrench）：
#        - ros2 service call /apply_link_wrench gazebo_msgs/srv/ApplyLinkWrench "{link_name: 'base_link', wrench: {force: {x: 300.0, y: 0.0, z: 0.0}}, start_time: {sec: 0, nanosec: 0}, duration: {sec: 1, nanosec: 0}}"
#        - link_name 若报找不到，换成全限定名，如 机器人模型名::base_link；力大小按车重微调。
#    - 硬"绑架"（演示能力边界——阶段 2 不自动恢复、需重给 /initialpose）：直接瞬移。
#        - 若有 /set_entity_state（gazebo_msgs/SetEntityState）：
#        - ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState "{state: {name: '机器人模型名', pose: {position: {x: 3.0, y: 2.0, z: 0.1}}}}"
#        - 服务名/模型名按 ros2 service list 与 ros2 topic echo /... 实测为准——不同 Gazebo（Classic 11 / Fortress）插件不一样。
#    - 核心 A 验收其实只要开车看贴合即可，撞击是加分演示。
```

### 验收标准

> [!NOTE]
> 阶段 2 为 A 模式（手动初值的纯位姿跟踪）。本仿真特征丰富、FAST-LIO 受 LiDAR 扫描匹配强约束而**近乎无漂移**（见已知限制），故 A 验收**不以"纠正累积漂移"为判据**，改以"正确锁定 + 稳定跟踪 + 小偏差拉回"验证 GICP 定位有效；C（可量化诊断）为硬标准。RViz Fixed Frame 设 `map`。

- **编译**：默认 `colcon build --packages-up-to gicp_localization` 通过；`-DGICP_DIAGNOSTICS=ON` 也通过。
- **A 正确锁定与跟踪**：RViz Fixed=`map`，叠 `/gicp_localization/prior_map`(Durability=Transient Local) 与 `/cloud_registered`；用「2D Pose Estimate」给正确初值后当前帧云**贴合先验地图**，开车/绕场全程**持续贴合、不重影**。实测正确锁定 `fitness≈1.0`、`mean_residual≈0.05`、`iterations`1–2、`correction_delta_*` 小而稳。
- **A 收敛拉回**：故意给一个**小偏差初值**（约 20–30cm / 10–20°），GICP 应在 1–2 周期内把点云**拉回贴合**（`fitness→1.0`）。
- **C 可量化**：`-DGICP_DIAGNOSTICS=ON` 下 `ros2 topic echo /gicp_localization/diagnostics`，看到锁定时 `fitness≈1.0`、`accepted: true`、`mean_residual≈0.05`、`correction_delta_*` 有界、`iterations`1–2 —— 即配准质量的量化证据。

### 已知限制（GICP 定位）

- **本仿真无可演示的漂移**：特征丰富的工厂环境下 FAST-LIO 受 LiDAR 扫描匹配强约束、近乎无漂移；即使把 IMU 噪声调到极大也诱发不出漂移（LiDAR 几何约束会把 IMU 误差纠回）。故 GICP "纠正累积里程计漂移" 的收益在本仿真里**无法直观演示**——属仿真环境性质、非 GICP 缺陷；真实/退化场景（长走廊、空旷无特征）下漂移才会显现。
- **大偏差不自动恢复 + 环境伪对称假解**：GICP 为局部配准，初值偏差超出收敛域（约 > `gicp_max_corr_dist≈1m` / 大角度）无法恢复；且工厂存在 **90° 旋转伪对称**，会形成 `fitness≈0.8` 的强假解。`fitness_threshold` 默认 **0.9** 即用于拒绝该假解（正确锁定 `fitness≈1.0`）。全自动从任意位姿恢复属阶段 3 全局重定位（Quatro 前端）。
- **bootstrap 需正确初值**：须在机器人实际所在的 map 位姿附近给 `/initialpose`（或在出生点即时启动定位节点）；启动太晚、机器人已驶离初值处，会导致首次配准 0 对应点（`fitness=0`）。
- **velodyne16 单帧稀疏**：单帧 GICP 若不稳，退路是用里程计攒最近 N 帧成局部子图再配（本阶段未实现）。
- **非严格时间同步**：50Hz 融合用校正与里程计各自最新值。

---

## ⚠️ 已知限制

> [!NOTE]
> ### 限制一：仿真点云缺少逐点时间戳（去畸变）
> **根因**：Gazebo 的 velodyne 仿真插件输出的是**瞬时快照点云**，字段只有 `x / y / z / intensity / ring`，**缺少逐点 `time`**。
>
> - **LIO-SAM**：`imageProjection` 检测不到逐点时间会**自动禁用去畸变（deskew）**，启动时打印 `... deskew function disabled, system will drift significantly!`。结果 `imuPreintegration` 发布的 `odom → base_footprint` 实时 TF 随时间缓慢漂移，`base_footprint` 及子节点（`base_link`、`imu_link`、左右轮）会「飘到空中」。**但最终建图由 `mapOptimization` 的点云配准完成，保存的地图不受影响、不发散**，可忽略此现象（可在 RViz2 关闭对应 TF 显示）。
> - **FAST-LIO2**：缺 `time` 时它会按方位角**编造**逐点时间并去畸变，对一份本无运动畸变的快照点云施加伪畸变，**旋转时产生拖影/发散**。本项目用仅仿真补丁 `src/fast-lio2-patch/sim-only/disable-deskew-snapshot-lidar.patch` 关闭帧内去畸变来解决（**真实旋转式雷达扫描确有畸变、必须去畸变，故真实场景勿应用此补丁**）。
>
> **根治方向**：修改 velodyne 仿真插件，使其在点云中额外输出逐点 `time` 字段 —— 改动较大，本项目暂未实现。

> [!NOTE]
> ### 限制二：机器人模型物理不稳定（竖向抖动）
> **现象**：仿真启动约 1 分钟后（即使空闲不操作也会发生），四个轮子连同车体出现并逐渐加剧的**竖向抖动**（只上下抖、无左右抖）；爆发前可见车体内部轻微闪烁。
>
> **根因**：`robot.sdf` 是 URDF 的粗转换产物，物理条件病态——转向连杆惯量仅 `1e-5`、前轮质量 `0.137 kg`（约为车体的 1/80）、底盘采用 STL **网格碰撞**、前转向关节**自由且零阻尼/零摩擦**。综合表现为**数值积分不稳定**（RTF 稳定、振幅随时间指数增长，故非算力问题）。
>
> **已排查无效的尝试**：调整接触 `max_vel`、加大前轮质量/惯量、移除底盘网格碰撞均无改善；把前转向关节锁为 `fixed` 会导致前轮倒车时飞出（进一步印证模型本身病态）。
>
> **影响范围**：抖动会污染 IMU，剧烈运动下定位/建图质量下降；**轻柔驾驶下 FAST-LIO / LIO-SAM 仍可正常工作，不阻断里程计与建图**。
>
> **根治方向**：重建/清理 robot 模型（合理的连杆质量与惯量、基元（box/cylinder）碰撞、带阻尼或直接固定的关节），或全局收紧物理求解器（减小 `max_step_size`，代价是 RTF 下降）。属独立任务，本项目暂未实现。

## nav2 导航（阶段一：打通链路 + costmap）

> [!NOTE]
> 阶段一只验证三件事：**TF 焊成单树**、**2D 先验图（全局 costmap）**、**3D 点云障碍层（局部 costmap）**正确。**不发目标点、不接行为树、机器人不自主移动**。
> 必须压在完整现有栈之上（robot_sim + FAST-LIO + gicp_localization），缺任一则 TF 有断边、costmap 取不到坐标。
> costmap 按 nav2 官方方式托管：`planner_server` 托管 global_costmap（`map` 系，2D 先验图）、`controller_server` 托管 local_costmap（`camera_init` 系，3D 点云）。阶段一**不调用**这两个服务器，只借它们把两张图跑起来。

### 准备项目文件

```bash
# 1. 离线把 LIO-SAM 先验地图 PCD 投影成 2D 占据栅格（需 open3d；产物 ~/result/map.* 不进 git）
pip install open3d
python src/robot_navigation/tools/pcd_to_occupancy.py \
  --pcd ~/result/GlobalMap.pcd --out ~/result/map.yaml \
  --resolution 0.05 --z-min 0.2 --z-max 1.5 --min-pts 2
# 取 0.2~1.5m 高度带投影成墙体障碍；之后看 RViz /map 效果再调 z-min/z-max/min-pts/resolution

# 2. 给 robot_gazebo 打 use_teleop 开关补丁
#    （robot_gazebo 是 clone 上游、改动一律走补丁。该开关供后续导航阶段从源头关掉手动遥控；
#      阶段一保持默认 use_teleop:=true，仍用遥控驱动机器人去看 costmap）
(cd src/robot_gazebo && git apply ../robot_gazebo-patch/01-add-use-teleop-switch.patch)
```

### 编译运行

```bash
# 1. 编译本包（robot_navigation 是纯 launch/config/tools 资源包，无 C++ 编译目标）
cd src    # colcon 工作区根，后续命令都在此执行
colcon build --packages-select robot_navigation
source install/setup.bash
# 若报缺 nav2 组件，补装后重编：
# sudo apt install ros-humble-nav2-planner ros-humble-nav2-controller \
#     ros-humble-nav2-costmap-2d ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

**2. 校正 `body→base_footprint` 静态焊接（关键，先做一次）**

FAST-LIO 的 `body` 帧和机器人 URDF 的 `base_footprint` 分属两棵互不相连的 TF 树，本 launch 用一条静态变换把它们焊成单树。这条变换数值必须对，否则两张 costmap 整体错位。

- **原理**：FAST-LIO 配置里 extrinsic 置零、IMU 与传感器共位，所以 `body` 帧物理上就等于 `imu_link`。于是 `body→base_footprint` 的焊接值 = URDF 里 `imu_link→base_footprint` 的值。
- **直接读出要填的值**（只要终端一的 robot_sim 起来即可，robot_state_publisher 已在发 URDF 的 TF）：

```bash
ros2 run tf2_ros tf2_echo imu_link base_footprint
```
把它打印的 `Translation: [x, y, z]` 与 `RPY (radian): [roll, pitch, yaw]` **原样**填入 launch 的 `tf_x tf_y tf_z tf_roll tf_pitch tf_yaw`（同一旋转的 RPY 表示可能不唯一，原样填即正确）。
本模型实测约 `Translation [0, 0, -0.297]`、旋转为绕 Y 轴 180°（RPY `[π, 0, π]`）——launch 默认 `tf_z=-0.297322, tf_pitch=3.14159274` 即等价于此。**机器人模型没改过，就直接用默认，跳过覆盖。**

> 启动瞬间若打印一行 `[INFO] ... Waiting for transform ... Invalid frame ID "imu_link" ... frame does not exist`，是 tf2_echo 监听器尚未收到 latched 静态 TF 的一次性竞态（INFO 非 ERROR），紧接着就会正常打出变换，**不是错误**。结果显示 `At time 0.0` 也正常（静态 TF 时间戳恒为 0）。

**3. 四个终端依次启动**（每个终端先 `cd src && source install/setup.bash`）

```bash
# 终端一：Gazebo 仿真
ros2 launch robot_gazebo robot_sim.launch.py
# 终端二：FAST-LIO2 里程计（仿真时间）
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true
# 终端三：GICP 先验地图定位（启动后在 RViz 用「2D Pose Estimate」给正确初值，详见上方 GICP 小节）
ros2 launch gicp_localization localization.launch.py prior_map_path:=~/result/GlobalMap.pcd
# 终端四：nav2 阶段一 costmap。TF 默认值不适用你的模型时，用上一步读到的值追加 tf_*:= 覆盖
ros2 launch robot_navigation stage1_costmaps.launch.py map:=~/result/map.yaml
#   覆盖示例：ros2 launch robot_navigation stage1_costmaps.launch.py map:=~/result/map.yaml \
#               tf_x:=0.0 tf_y:=0.0 tf_z:=-0.297 tf_roll:=0.0 tf_pitch:=3.14159 tf_yaw:=0.0
```

终端四的 launch 起 `map_server` + `planner_server`（托管 global_costmap）+ `controller_server`（托管 local_costmap），由 `lifecycle_manager` 自动 configure→activate 三者。阶段一不发目标点、不接 BT，planner/controller 仅作 costmap 宿主空转、不驱动机器人。

> `map:=` 支持 `~` / 相对 / 绝对路径——launch 内部已 `expanduser+abspath` 成绝对路径再交给 map_server（nav2 map_io 对 yaml 展开 `~` 但拼接 image 路径时不展开，直接传 `~/...` 会因找不到 `.pgm` 而 `map_server` configure 失败、lifecycle 全程 `unconfigured`）。

**4. 验证焊接成功**（四个终端都起来后，body 应正好落在 imu_link 上）

```bash
ros2 run tf2_ros tf2_echo imu_link body
```
预期 `Translation ≈ [0, 0, 0]`、`RPY ≈ [0, 0, 0]`（两帧重合）。若明显非零，说明 `tf_*` 填错，回到第 2 步重新覆盖。

### 验收标准

> [!NOTE]
> 阶段一为定性验收：链路通、两张图出得来、3D 障碍层标记/清除/滤地面正确即可。全程机器人不自主移动。

- **TF 单树**：`ros2 run tf2_tools view_frames` 生成的图是一棵根在 `map` 的单树，`base_footprint` 经 `body` 连到 `camera_init`/`map`，无悬空孤岛。
- **节点起齐、无重名**：`ros2 node list` 含 `/map_server`、`/planner_server`、`/controller_server`、`/global_costmap/global_costmap`、`/local_costmap/local_costmap`，且**无 “share an exact name” 警告**。
- **生命周期激活**：`ros2 lifecycle get /map_server`、`/planner_server`、`/controller_server` 均返回 `active`（lifecycle_manager 日志不再反复打印 `Waiting for service ... get_state`）。
- **全局图（2D 先验图）**：`ros2 topic echo /map --once` 有数据；`ros2 topic hz /global_costmap/costmap` 有输出；RViz（Fixed Frame=`map`）正确显示 2D 先验图。
- **局部图（3D 点云障碍）**：`ros2 topic hz /local_costmap/costmap` 有输出；遥控驱动机器人靠近障碍，local costmap 在障碍处标记体素、障碍移开后能清除、**地面不被误标为障碍**。
- **可配置**（见下）：换 STVL 障碍层只需换 `params_file`、切障碍源话题只改 params，均不改代码即生效。
- **不自主移动**：全程无目标点、无行为树，机器人只在遥控下移动。

#### 可配置切换（验证可配置性）

- **换 STVL 障碍层（不改任何文件，已备好现成配置）**：装包后启动终端四时用 `params_file:=` 指向 STVL 版配置即可。该文件 `config/nav2_costmaps_stvl.yaml` 的 local_costmap 已换成 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`，其余与默认完全一致。

  ```bash
  sudo apt install ros-humble-spatio-temporal-voxel-layer
  ros2 launch robot_navigation stage1_costmaps.launch.py map:=~/result/map.yaml \
    params_file:=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/nav2_costmaps_stvl.yaml
  ```
  STVL 的清除模型项（`voxel_decay` / `decay_acceleration` / FOV）为经验值，按清除快慢在该文件里调。
- **切障碍源话题**：改所用 params 文件里 `local_costmap → local_costmap` 障碍层的 `topic`（`/points_raw` ↔ `/cloud_registered`），重启终端四。
