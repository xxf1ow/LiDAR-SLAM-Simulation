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
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' -r 10
ros2 run tf2_ros tf2_echo odom base_link     # odom->base_link 随命令前进
```

## 4. Gz Harmonic 仿真驱动(核心)
```bash
ros2 launch robot_gz_bringup robot_gz.launch.py
# 另一终端：
ros2 control list_controllers      # 两控制器 active(controller_manager 由 gz 插件提供)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.4}}' -r 10
```
预期:Gz GUI 里机器人沿弧线前进;RViz 里 odom->base_link 随动;`ros2 topic echo /base_controller/odom` 有数据。

## 5. 裁决

**PASS(Phase 1 完成、进 Phase 2)需全部满足:**
1. 四个包 `colcon build` 全绿;`robot_description` 三态 xacro 测试全 PASS。
2. `view_robot` 在 RViz 里模型正确显示。
3. mock 模式:两控制器 active,发 `/cmd_vel` 后 `odom->base_link` 变换随动。
4. **Gz 模式:机器人在 Gazebo Harmonic 里被 `/cmd_vel` 驱动移动(直线 + 转向),控制器 active,里程计/ TF 输出正常。**
5. 同一 `robot.urdf.xacro` 仅靠 arg 完成 gz/mock/real 三态切换(real 分支编译且插件登记,真机通信留作后续)。

**FAIL(回流程，按 systematic-debugging):**
- Gz 模式机器人不动:查 `/cmd_vel` 是否到 `base_controller`(remap 是否生效)、轮子是否打滑(empty.sdf 地面摩擦)、关节命令接口是否 velocity。记录 `ros2 control list_hardware_interfaces` 与 `gz topic -l` 回报。
- 控制器起不来:查 controller_manager 是否由 gz 插件起(`ros2 node list` 应有 `/controller_manager`)、`gz_controllers_file` 路径是否注入正确(看展开后的 URDF `<parameters>` 是否绝对路径)。
- xacro 测试 FAIL:看 `colcon test-result --verbose` 的断言与 check_urdf 报错。

## 已知注意
- Phase 1 用参考 DiffBot 的微小尺寸(轮距 0.10 m),Gz 里机器人很小、移动距离小,看 odom 数值确认即可;真车尺寸在 Phase 2。
- Gz 模式 RTF 可能 <1,`/cmd_vel` 用持续发布(`-r 10`)而非 `--once`。
- 若 Gz 里轮子打滑导致直线跑偏,记为 Phase 2 摩擦/惯量调参项,不在 Phase 1 阻塞判据(只要能被驱动移动即 PASS)。
