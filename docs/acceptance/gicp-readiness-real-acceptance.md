# GICP 首次配准 Readiness 真机验收

## 1. 边界

本流程只验证 navigation 启动顺序：RViz 先可用，GICP 首次 accepted 后 `/localization`
才进入 ROS graph，随后现有 ready gate 启动 Nav2。禁止发送 Nav2 goal、移动底盘、调参、
保存地图、运行 `save_map.sh` 或 push。

## 2. 复用安全框架并构建

打开同一提交中的 `docs/acceptance/profile-migration-real-acceptance.md`：

1. 执行第 1 节安全边界。
2. 执行第 2 节中的 exact HEAD、single worktree、clean tree、地图可读、原配置 byte-exact
   备份、stack regex、ROS source 和首次 `stack_absent`；`EXPECTED_HEAD` 必须填写本次
   review-approved exact commit。
3. 不重复 full-workspace build/test，改为：

```bash
cd "$CORE"
colcon build --packages-select gicp_localization robot_navigation system_bringup
source install/setup.bash
```

4. 定义原手册第 3 节全部共用函数和 trap，不修改其有界 TERM/KILL、配置恢复和残留检查。

## 3. real + navigation

```bash
MODE=navigation
python3 - "$CONFIG" <<'PY'
from pathlib import Path
import sys, yaml
p = Path(sys.argv[1])
d = yaml.safe_load(p.read_text(encoding="utf-8"))
d["platform"] = "real"
d["mode"] = "navigation"
p.write_text(yaml.safe_dump(d, sort_keys=False), encoding="utf-8")
PY
must start_mode
```

不发送 goal。记录 `$LOG_DIR/navigation.log` 和本次 `$RUNTIME_DIR`，确认：

1. `rviz2` 在 Nav2 server 之前启动且只有一个实例；
2. `/gicp_localization/prior_map` 与 `/cloud_registered_body` 存在；
3. RViz 配置的 fixed frame 为 `map`，能显示 prior map 和 registered cloud；
4. 若初始位姿正确，允许 GICP 自动完成首次 accepted；
5. 若未自动 accepted，在 ready gate 的 discovery timeout 内用 RViz “2D Pose Estimate”发布
   `/initialpose`，不得用临时参数或源码修改绕过；
6. 日志中 `/localization` gate 放行发生在 GICP 启动之后，Nav2 server 启动发生在该 gate
   放行之后；
7. `/localization` 能收到真实消息，五个 lifecycle 节点最终均为 `active [3]`：
   `/map_server`、`/planner_server`、`/controller_server`、`/behavior_server`、`/bt_navigator`。

可使用以下只读命令保存证据：

```bash
ros2 node list | sort | tee "$LOG_DIR/navigation-nodes.txt"
ros2 topic list | sort | tee "$LOG_DIR/navigation-topics.txt"
timeout 30s ros2 topic echo /localization --once \
  >"$LOG_DIR/navigation-localization-once.txt"
for node in /map_server /planner_server /controller_server /behavior_server /bt_navigator; do
  must check_lifecycle "$node"
done
```

最后必须执行并通过：

```bash
must cleanup
must cmp -s "$CONFIG" "$ORIGINAL_CONFIG"
must stack_absent
must test -z "$(git diff -- "$CONFIG")"
must test -z "$(git status --porcelain --untracked-files=no)"
trap - EXIT INT TERM
```

## 4. 报告

报告写入忽略路径 `.superpowers/sdd/gicp-readiness-real-acceptance-report.md`：

```markdown
# GICP Readiness Real Acceptance Report
## Metadata and baseline
## Affected build
## Early RViz evidence
## First accepted and Nav2 ordering
## Lifecycle and localization evidence
## Cleanup and restored state
## Environment limitations
## Verdict
PASS / FAILED / BLOCKED
```

只有提前 RViz、首次 accepted、`/localization` gate、Nav2 启动顺序、五个 lifecycle 和 cleanup
全部通过时才能写 `PASS`。普通日志保留在仓库外。
