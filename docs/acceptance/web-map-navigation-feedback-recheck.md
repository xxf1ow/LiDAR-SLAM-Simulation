# Web 导航 Feedback 修复真机复测

本流程只复测首次报告失败或未完成的 Web 链路，不重复完整 GICP readiness 验收。

获准代码基线：`5361c6f5f20782a97574aa19a93874befca12878`。
手册可以是其 docs-only 后继，但 `core/` 不得再变化。

## 1. 前置

复用 `web-map-navigation-real-acceptance.md` 的安全、地图、单 worktree、配置备份、real +
navigation 修改、启动、PGID 和清理步骤。额外确认：

```bash
cd /home/lxx/workspace/xxsim
git merge-base --is-ancestor \
  5361c6f5f20782a97574aa19a93874befca12878 HEAD
git diff --quiet 5361c6f5f20782a97574aa19a93874befca12878..HEAD -- core
test -z "$(git status --porcelain --untracked-files=no)"
```

构建受影响组合包：

```bash
cd /home/lxx/workspace/xxsim/core
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  robot_web_ui system_bringup robot_navigation gicp_localization
source install/setup.bash
```

## 2. 栈与 Web 就绪

按原手册启动。等待 `/localization` 和五个 Nav2 lifecycle 节点 active，然后记录 Web PID：

```bash
WEB_RECHECK_PID=$(pgrep -n -f '/robot_web_ui|robot_web_ui =')
test -n "$WEB_RECHECK_PID"
kill -0 "$WEB_RECHECK_PID"
curl --fail --silent http://127.0.0.1:8080/api/navigation-state \
  > /tmp/web-feedback-state-before.json
```

## 3. 补验 `/initialpose`

在另一终端先运行：

```bash
timeout 60 ros2 topic echo --once /initialpose \
  > /tmp/web-feedback-initialpose.txt
```

Web 页面选择当前机器人所在位置和朝向并确认“设置初始位姿”。确认：

- 捕获到一条 frame=`map` 的 `/initialpose`；
- 页面只报告已发送，不声称配准成功；
- `robot_web_ui` PID 仍存活，8080 仍响应；
- GICP 后续仍有 `/localization`，Nav2 lifecycle 保持 active。

## 4. Goal feedback 与 cancel

使用短距离安全目标，或在固定底盘条件下目标接受后立即取消：

1. Web 设置并确认导航目标。
2. 必须至少观察到一次“导航中”以及有限的剩余距离。
3. 必须显示全局路径。
4. 核对 `kill -0 "$WEB_RECHECK_PID"` 成功、8080 仍返回 200。
5. 日志中不得出现 `_navigation_feedback_callback`、`TypeError` 或 Web 进程退出。
6. 点击一次“取消导航”，确认状态收敛为“导航已取消”且机器人停车。
7. 再次核对 Web PID 和 8080 存活；Nav2 goal 不再 active。

可保存状态快照：

```bash
curl --fail --silent http://127.0.0.1:8080/api/navigation-state \
  > /tmp/web-feedback-state-after-cancel.json
```

## 5. Manual 回归

无活动目标时：

1. 点击“人工接管”，确认 manual 面板出现、导航目标禁用。
2. 不具备安全移动条件时不按方向键。
3. 点击“恢复自动导航”，确认面板隐藏、导航目标恢复可用。
4. Web PID 与 8080 必须全程存活。

## 6. 清理与报告

执行原手册第 7 节：有界停止 PGID、恢复配置并 `cmp`、确认目标进程/ROS 节点消失、tracked
tree clean。

复测报告写入 ignored 路径：

```text
.superpowers/sdd/web-map-navigation-feedback-recheck-report.md
```

报告包含实际 HEAD、build/start、initialpose、至少一次 feedback、path、cancel、manual、Web PID/
HTTP 存活、日志无 TypeError、cleanup 和最终 `PASS`/`FAIL`。
