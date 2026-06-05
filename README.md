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

## 编译运行

### 准备项目文件

```bash
# 1. 下载 cache zip, 解压到根目录
curl -C - -O https://github.com/xxf1ow/LiDAR-SLAM-Simulation/releases/download/cache-zip/cache.zip

# 2. 克隆 velodyne_simulator
git clone https://github.com/ToyotaResearchInstitute/velodyne_simulator --depth 1 --filter=blob:none

# 3. 克隆 LIO-SAM 并应用补丁
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2 --single-branch --depth 1 --filter=blob:none 
cd LIO-SAM
git fetch origin 08af3f32f01725372d4269838dc44c19c6d9e76b --depth 1
git checkout 08af3f32f01725372d4269838dc44c19c6d9e76b
git apply ../lio-sam.patch

# 4. 把工厂模型拷到 Gazebo 资源目录
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
# 1. PPA 方式安装 GTSAM 依赖库
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev

# 2. 编译
# 重新编译指定包: colcon build --packages-select lio_sam
cd src
colcon build

# 3. 终端一运行 (Gazebo Simulation)
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch robot_gazebo robot_sim.launch.py

# 4. 终端二运行 (LIO-SAM + Rviz2) 
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 launch lio_sam run.launch.py

# 5. 终端三运行: 保存点云结果
# 调用命令: ros2 service call
# 服务名称: /lio_sam/save_map
# 消息类型: lio_sam/srv/SaveMap
# 请求参数: "{resolution: 0.2, destination: /result}"
# 最终保存路径: ~/result
cd LiDAR-SLAM-Simulation/src
source install/setup.bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: /result}"
```
