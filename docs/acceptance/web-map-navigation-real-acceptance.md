# Web 地图与导航控制真机验收

目标：只验证 Web-B 与 GICP readiness 合并后的真实链路，不重复算法调参或完整 GICP 验收。

获准的组合代码基线：`0628b17fdea960374a068b0ced0e58a9dd1bb48d`。
验收手册可以是该提交之后的 docs-only 提交，但 `core/` 不得再有差异；报告必须记录实际 HEAD。

## 1. 前置与安全

- 机器人位于空旷安全区域，有现场人员、物理急停和可用网络。
- 不现场调参，不保存地图，不修改 Profile/template。
- 导航目标只选短距离安全点；若不具备移动条件，只执行到目标接受后立即取消，并在报告说明。
- 仓库只有一个 worktree，tracked tree 干净，三个地图文件可读。

```bash
cd /home/lxx/workspace/xxsim
git merge-base --is-ancestor \
  0628b17fdea960374a068b0ced0e58a9dd1bb48d HEAD
git diff --quiet 0628b17fdea960374a068b0ced0e58a9dd1bb48d..HEAD -- core
test "$(git worktree list --porcelain | grep -c '^worktree ')" = 1
test -z "$(git status --porcelain --untracked-files=no)"
test -r "$HOME/result/GlobalMap.pcd"
test -r "$HOME/result/factory_map.yaml"
test -r "$HOME/result/factory_map.pgm"
```

修改前备份配置：

```bash
cp core/bringup/system_bringup/config/bringup.yaml \
  /tmp/web-map-navigation-bringup.original.yaml
```

只把 `platform` 设为 `real`、`mode` 设为 `navigation`，确认 diff 仅为预期选择项。
从启动开始，无论中途 PASS/FAIL/BLOCKED，都必须执行第 7 节恢复与清理；以下启动和清理命令
应在同一个 shell 中执行，以保留 `WEB_ACCEPT_PID/PGID`。

## 2. 构建与启动

```bash
cd /home/lxx/workspace/xxsim/core
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  robot_web_ui system_bringup robot_navigation gicp_localization
source install/setup.bash

setsid ros2 launch system_bringup bringup.launch.py \
  > /tmp/web-map-navigation-real.log 2>&1 &
WEB_ACCEPT_PID=$!
WEB_ACCEPT_PGID=$(ps -o pgid= -p "$WEB_ACCEPT_PID" | tr -d ' ')
printf 'PID=%s PGID=%s\n' "$WEB_ACCEPT_PID" "$WEB_ACCEPT_PGID"
```

Web 地址：`http://<机器人IP>:8080`。本验收不依赖 RViz；headless 环境的 RViz 显示失败需记录，
但只有影响 GICP/Web/Nav2 链路时才判为产品失败。

## 3. 首次定位前

页面打开后确认：

- 静态二维地图已经可见；缩放、平移、适配可用。
- 页面显示“等待定位”。
- “设置导航目标”禁用。
- Nav2 五个 lifecycle 节点尚未全部 active。
- GICP 启动并订阅 `/initialpose` 后，“设置初始位姿”变为可用。

发布前在另一终端捕获消息：

```bash
source /opt/ros/humble/setup.bash
source /home/lxx/workspace/xxsim/core/install/setup.bash
timeout 60 ros2 topic echo --once /initialpose \
  > /tmp/web-map-navigation-initialpose.txt
```

在 Web 页面：

1. 点击“设置初始位姿”。
2. 拖动地图，使半透明准星对准实际位置。
3. 拖动箭头端部设置朝向；确认另一根手指不会改变当前箭头。
4. 点击“确认位置”。

核对捕获消息：frame 为 `map`，x/y/yaw 与页面预览一致。页面只应报告请求已发送，不应声称
GICP 已经配准成功。

## 4. 首次定位与 Nav2 释放

等待 GICP 接受后核对：

```bash
timeout 60 ros2 topic echo --once /localization
for node in map_server planner_server controller_server behavior_server bt_navigator; do
  ros2 lifecycle get "/$node"
done
```

期望：

- `/localization` frame 为 `map`，child frame 为 `body`。
- 五个 lifecycle 节点均为 `active [3]`。
- Web 显示机器人位置和朝向，全局/局部代价地图、全局路径图层无明显错位。
- gate 为 automatic 时，“设置导航目标”可用。

## 5. Web 目标与取消

选择一个短距离安全目标：

1. 点击“设置导航目标”。
2. 用准星和箭头设置最终位置、朝向并确认。
3. 确认页面依次显示发送中、导航中和剩余距离；地图出现全局路径。
4. 点击“取消导航”。
5. 确认只发送一次取消，页面收敛到“导航已取消”，机器人停止。

若现场允许，可再发送一个短目标并观察成功；这不是 PASS 的强制条件。重复点击确认或取消
不得产生第二个任务或明显错误状态。

## 6. 人工模式回归

当前无活动导航目标时：

1. 点击“人工接管”：不透明方向控制面板出现，“设置导航目标”禁用。
2. 不具备安全移动条件时不要按方向键；本验收不重复底盘驾驶测试。
3. 点击“恢复自动导航”：manual 面板隐藏，定位/Nav2 正常时导航目标重新可用。
4. mode 或导航错误必须显示在各自区域，manual notice 不得被导航状态覆盖。

## 7. 清理与恢复

```bash
kill -TERM -- "-$WEB_ACCEPT_PGID"
for _ in $(seq 1 20); do
  kill -0 -- "-$WEB_ACCEPT_PGID" 2>/dev/null || break
  sleep 1
done
! kill -0 -- "-$WEB_ACCEPT_PGID" 2>/dev/null

cp /tmp/web-map-navigation-bringup.original.yaml \
  /home/lxx/workspace/xxsim/core/bringup/system_bringup/config/bringup.yaml
cmp /tmp/web-map-navigation-bringup.original.yaml \
  /home/lxx/workspace/xxsim/core/bringup/system_bringup/config/bringup.yaml

cd /home/lxx/workspace/xxsim
test -z "$(git status --porcelain --untracked-files=no)"
```

确认相关 ROS 节点和目标进程已消失；ROS daemon 延迟可短时轮询，但不能遗留实际进程。

## 8. 报告格式

报告写入 ignored 路径：

```text
.superpowers/sdd/web-map-navigation-real-acceptance-report.md
```

至少包含：

- 时间、主机、分支、精确 HEAD、单 worktree/clean 状态；
- 构建结果和启动 PGID；
- 定位前静态地图、按钮状态和 Nav2 未释放证据；
- `/initialpose`、首次 `/localization` 和五个 lifecycle 状态；
- 地图/机器人/代价地图/路径的视觉结论；
- goal 发送、剩余距离、cancel 与停车结果；
- manual/automatic 面板回归；
- cleanup、配置 byte-exact 恢复和最终 clean 状态；
- 环境限制、非产品异常和最终 `PASS` 或 `FAIL`。
