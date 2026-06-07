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


# 5. 克隆 small_gicp
export GIT_LFS_SKIP_SMUDGE=1    # linux (跳过 LFS 文件下载)
$env:GIT_LFS_SKIP_SMUDGE=1	    # Windows PowerShell (跳过 LFS 文件下载)
git clone https://github.com/koide3/small_gicp.git --depth 1 --filter=blob:none
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

# 2. 手动克隆 small_gicp 到 src/small_gicp 并 pin 提交 (跳过 LFS 大文件不影响编译)
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/koide3/small_gicp.git --depth 1 --filter=blob:none
# (可选) pin 到已验证提交: cd small_gicp && git fetch origin <SHA> --depth 1 && git checkout <SHA> && cd ..
```

### 编译运行

```bash
# 1. 编译 (定位节点依赖 small_gicp，colcon 自动先建 small_gicp)
cd src
colcon build --packages-up-to gicp_localization
# 调试带诊断话题:
# colcon build --packages-up-to gicp_localization --cmake-args -DGICP_DIAGNOSTICS=ON

# 2. 终端一: Gazebo
ros2 launch robot_gazebo robot_sim.launch.py

# 3. 终端二: FAST-LIO2 里程计
ros2 launch fast_lio mapping.launch.py config_file:=gazebo_velodyne.yaml use_sim_time:=true

# 4. 终端三: GICP 定位 (先验地图路径按实际)
ros2 launch gicp_localization localization.launch.py prior_map_path:=~/result/GlobalMap.pcd
```

### 验收标准

> [!NOTE]
> 阶段 2 验收：A（漂移钉住）为硬标准，C（可量化诊断）为硬标准。RViz Fixed Frame 设 `map`。

- **编译**：默认 `colcon build --packages-up-to gicp_localization` 通过；`-DGICP_DIAGNOSTICS=ON` 也通过。
- **A 漂移钉住**：RViz Fixed=`map`，叠加先验地图与 `/cloud_registered`；给初值后当前帧云**持续贴合先验地图、不随时间漂走**，绕场一圈回原点仍对齐。
- **C 可量化**：`-DGICP_DIAGNOSTICS=ON` 构建下 `ros2 topic echo /gicp_localization/diagnostics` 或 `rqt_plot` 看到 `fitness`（≳0.8）稳定、`correction_delta_*` 有界；与阶段 1 纯里程计对比漂移改善。
- **扰动恢复（演示）**：对小车施加温和撞击/连续偏移（Gazebo `apply_wrench` 或缓推），定位应自动跟住、不丢。

### 已知限制（GICP 定位）

- **velodyne16 单帧稀疏**：单帧 GICP 若不稳，退路是用里程计攒最近 N 帧成局部子图再配（本阶段未实现）。
- **硬"绑架"不自动恢复**：温和/连续偏移能自动跟住；瞬间大幅 teleport 或偏移超出 GICP 收敛域（约 > `gicp_max_corr_dist≈1m` / 大角度）时**单帧 GICP 局部配准无法恢复**，需在 RViz 重给 `/initialpose`；全自动全局重定位属后续阶段（Quatro 前端）。
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
