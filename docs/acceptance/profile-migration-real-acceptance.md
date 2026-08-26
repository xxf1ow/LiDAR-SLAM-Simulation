# Profile 迁移 6D 真机验收

## 1. 边界与停止条件

不发 goal、不让底盘运动、不保存地图、不运行 `save_map.sh`、不调参、不 push。生成配置、传感器、TF、时钟、算法、地图、生命周期或清理任一失败即停止。RViz 仅在其余链路健康时可记为 headless 环境限制。

## 2. Preflight、原件备份与构建

审查人先填写准确的获准提交。占位符未替换、HEAD 不符、非单 worktree、tracked state 不干净、地图资源不可读、启动前目标栈不为空均为 `BLOCKED`。**任何 `bringup.yaml` 修改前**先保存并核对 byte-exact 原件；此后所有正常/异常出口都由 trap 恢复它。

```bash
set -Eeuo pipefail
REPO="$(git rev-parse --show-toplevel)"; CORE="$REPO/core"
CONFIG="$CORE/bringup/system_bringup/config/bringup.yaml"
LOG_DIR="$HOME/result/acceptance-6d"; mkdir -p "$LOG_DIR"
ORIGINAL_CONFIG="$LOG_DIR/bringup.original.yaml"
EXPECTED_HEAD='<review-approved-exact-commit>'
test "$EXPECTED_HEAD" != '<review-approved-exact-commit>' && test "$(git rev-parse HEAD)" = "$EXPECTED_HEAD"
test "$(git worktree list --porcelain | grep -c '^worktree ')" -eq 1 && test -z "$(git status --porcelain --untracked-files=no)"
cp -- "$CONFIG" "$ORIGINAL_CONFIG" && cmp -s "$CONFIG" "$ORIGINAL_CONFIG"
mapfile -t MAP_PATHS < <(python3 - "$CONFIG" <<'PY'
from pathlib import Path
import sys, yaml
m = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))["map_artifacts"]
print(Path.home() / m["lio_sam_work_dir"].lstrip("/")); print(Path(m["prior_pcd"]).expanduser()); print(Path(m["nav2_map"]).expanduser())
PY
)
test -d "${MAP_PATHS[0]}" && test -r "${MAP_PATHS[0]}" && test -f "${MAP_PATHS[1]}" && test -r "${MAP_PATHS[1]}"
test -f "${MAP_PATHS[2]}" && test -r "${MAP_PATHS[2]}"
STACK_PROC_RE='[b]ringup.launch.py|[c]md_vel_gate|[r]obot_web_ui|[s]ensor_contract_gate|[c]an_driver_8030|[r]os2_control_node|[c]ontroller_manager|[s]pawner|[v]anjee_lidar_node|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2'
STACK_PROC_RE+='|[m]apOptimization|[i]muPreintegration|[i]mageProjection|[f]eatureExtraction|[f]astlio_mapping|[g]icp_localization|[t]wist_stamper|[m]ap_server|[p]lanner_server|[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[l]ifecycle_manager'
STACK_NODE_RE='/(cmd_vel_gate|robot_web_ui|sensor_contract_gate|can_driver_8030|controller_manager|base_controller|joint_state_broadcaster|vanjee_lidar|robot_state_publisher|static_transform_publisher[^/]*|rviz2|lio_sam|lio_sam_imuPreintegration|lio_sam_imageProjection|lio_sam_featureExtraction|lio_sam_mapOptimization|laserMapping|fastlio_mapping|fast_lio_body_to_base_footprint|gicp_localization|twist_stamper|map_server|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager_navigation|global_costmap/global_costmap|local_costmap/local_costmap)$'
source /opt/ros/humble/setup.bash
stack_absent() {
  test -z "$(pgrep -af "$STACK_PROC_RE" || true)" || return 1
  test -z "$(ros2 node list 2>/dev/null | grep -E "$STACK_NODE_RE" || true)" || return 1
}
stack_absent
cd "$CORE" && colcon build && source install/setup.bash
colcon test && colcon test-result --all --verbose
```

构建、测试必须为零 errors、failures、unexpected skips；不安装依赖或修复环境。

## 3. 共用检查与有界清理

以下函数在两个模式中复用。runtime directory 必须来自本次 launch 进程组的 generated YAML 参数，禁止按 `/tmp` 时间猜测。参数比较只投影 generated-owned keys，且类型和值均相等。

```bash
LAUNCH_PID=; LAUNCH_PGID=; MODE=
runtime_dir_for_group() {
  pgrep -a -g "$LAUNCH_PGID" | tee "$LOG_DIR/$MODE-processes-running.txt" | grep -oE '/tmp/system_bringup-runtime-[^ /]+/[^ ]+\.generated\.yaml' | sed 's#/[^/]*$##' | sort -u
}
check_runtime_files() {
  local n
  for n in robot_controllers robot_web_ui nav2 fast_lio lio_sam gicp sensor_gate vanjee_lidar; do test -r "$RUNTIME_DIR/$n.generated.yaml" || return 1; done
  test -r "$RUNTIME_DIR/effective_profile.generated.yaml"
}
compare_params() {
  python3 - "$@" <<'PY'
from pathlib import Path
import sys, yaml
generated_path, selector, live_path, *ignored = sys.argv[1:]
node = yaml.safe_load(Path(generated_path).read_text(encoding="utf-8"))
for key in selector.split("|"): node = node[key]
expected = dict(node["ros__parameters"])
for key in ignored: expected.pop(key, None)
live = yaml.safe_load(Path(live_path).read_text(encoding="utf-8"))
roots = [v["ros__parameters"] for v in live.values()
         if isinstance(v, dict) and isinstance(v.get("ros__parameters"), dict)]
def equal(a, b):
    if type(a) is not type(b): return False
    if isinstance(a, dict): return all(k in b and equal(v, b[k]) for k, v in a.items())
    if isinstance(a, list): return len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    return a == b
ok = len(roots) == 1 and equal(expected, roots[0])
print(f"{'EQUAL' if ok else 'DIFFERENT'} generated={generated_path} live={live_path}")
raise SystemExit(not ok)
PY
}
observe_30s() {
  local label="$1" pattern="$2" rc out="$LOG_DIR/$MODE-$1.txt"; shift 2
  if timeout 30s "$@" >"$out" 2>&1; then rc=0; else rc=$?; fi
  test "$rc" -eq 124 && grep -Eq "$pattern" "$out"
}
dump_params() { ros2 param dump "$1" >"$2"; }
record_nodes() { ros2 node list >"$1"; }
check_lifecycle() {
  local state; state="$(timeout 30s ros2 lifecycle get "$1" 2>&1)" || return 1
  printf '%s: %s\n' "$1" "$state"; test "$state" = 'active [3]'
}
launch_alive() {
  if test -n "$LAUNCH_PGID"; then pgrep -g "$LAUNCH_PGID" >/dev/null
  elif test -n "$LAUNCH_PID"; then kill -0 "$LAUNCH_PID" 2>/dev/null
  else return 1; fi
}
signal_launch() {
  if test -n "$LAUNCH_PGID"; then kill "-$1" -- "-$LAUNCH_PGID" 2>/dev/null || true
  elif test -n "$LAUNCH_PID"; then kill "-$1" "$LAUNCH_PID" 2>/dev/null || true; fi
}
stop_launch() {
  local i
  test -z "$LAUNCH_PGID" || pgrep -a -g "$LAUNCH_PGID" >"$LOG_DIR/$MODE-processes-before-stop.txt" || true
  if launch_alive; then
    signal_launch TERM
    for i in $(seq 1 20); do launch_alive || break; sleep 1; done
    if launch_alive; then signal_launch KILL; for i in $(seq 1 5); do launch_alive || break; sleep 1; done; fi
  fi
  launch_alive && return 1
  test -z "$LAUNCH_PID" || wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=; LAUNCH_PGID=
  stack_absent
}
cleanup() {
  local failed=0
  stop_launch || failed=1; cp -- "$ORIGINAL_CONFIG" "$CONFIG" || failed=1
  cmp -s "$CONFIG" "$ORIGINAL_CONFIG" || failed=1; stack_absent || failed=1
  return "$failed"
}
finish() {
  local rc="$1"; trap - EXIT INT TERM
  if ! cleanup; then echo 'FAILED: cleanup or config restore' >&2; rc=2; fi
  exit "$rc"
}
trap 'finish $?' EXIT
trap 'finish 130' INT
trap 'finish 143' TERM
must() { local rc; "$@" && return 0; rc=$?; echo "FAILED: $*" >&2; exit "$rc"; }
start_mode() {
  cp -- "$CONFIG" "$LOG_DIR/$MODE-bringup.snapshot.yaml" || return 1
  setsid ros2 launch system_bringup bringup.launch.py >"$LOG_DIR/$MODE.log" 2>&1 &
  LAUNCH_PID=$!; LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
  test -n "$LAUNCH_PGID" && cmp -s "$CONFIG" "$LOG_DIR/$MODE-bringup.snapshot.yaml" || return 1
  local dirs=() i
  for i in $(seq 1 30); do mapfile -t dirs < <(runtime_dir_for_group); test "${#dirs[@]}" -ne 0 && break; sleep 1; done
  test "${#dirs[@]}" -eq 1 || return 1
  RUNTIME_DIR="${dirs[0]}"; check_runtime_files
}
```

TERM 后最多等待 20 秒，再按需 KILL 并最多等待 5 秒；仅确认不存活后才 `wait`。无 PGID 时对 launch PID 执行同样流程。cleanup 最后还会检查 PGID/global process/ROS node 均消失并恢复原配置；失败将整次验收置为 `FAILED`。

## 4. real + mapping

先设置 `MODE=mapping`，仅把 `$CONFIG` 顶层改为 `platform: real`、`mode: mapping`，保存本次 manifest 输入快照后启动；launch 期间不得再改配置。

```bash
MODE=mapping
must start_mode

# 先从 `ros2 node list` 确认 gate 已释放，并把实际活动节点填入 LIO_NODE。
must record_nodes "$LOG_DIR/$MODE-nodes.txt"
must observe_30s points_raw 'average rate:' ros2 topic hz /points_raw
must observe_30s imu_data 'average rate:' ros2 topic hz /imu/data
must observe_30s lio_odom 'average rate:' ros2 topic hz /lio_sam/mapping/odometry
must observe_30s map_tf 'Translation:' ros2 run tf2_ros tf2_echo map base_footprint
must grep -q 'Rotation:' "$LOG_DIR/$MODE-map_tf.txt"
must dump_params "$LIO_NODE" "$LOG_DIR/$MODE-lio_sam.live.yaml"
must compare_params "$RUNTIME_DIR/lio_sam.generated.yaml" '/**' "$LOG_DIR/$MODE-lio_sam.live.yaml"
must cleanup
```

记录 `RUNTIME_DIR`；上述持续观察各为至少 30 秒。只有 cleanup 成功才进入下一模式。

## 5. real + navigation

从已恢复的原配置开始，仅改顶层 `platform: real`、`mode: navigation`，再执行以下步骤。确认 FAST-LIO、GICP、Nav2 gate 均已释放：

```bash
MODE=navigation
must start_mode
must observe_30s Odometry 'average rate:' ros2 topic hz /Odometry
must observe_30s cloud_registered_body 'average rate:' ros2 topic hz /cloud_registered_body
must observe_30s map_tf 'Translation:' ros2 run tf2_ros tf2_echo map base_footprint
must grep -q 'Rotation:' "$LOG_DIR/$MODE-map_tf.txt"
for NODE in /map_server /planner_server /controller_server /behavior_server /bt_navigator; do must check_lifecycle "$NODE"; done
must record_nodes "$LOG_DIR/$MODE-nodes.txt"

# 把实际活动节点填入 FAST_LIO_NODE。
must dump_params "$FAST_LIO_NODE" "$LOG_DIR/$MODE-fast_lio.live.yaml"
must dump_params /gicp_localization "$LOG_DIR/$MODE-gicp.live.yaml"
for NAME in map_server planner_server controller_server behavior_server bt_navigator; do must dump_params "/$NAME" "$LOG_DIR/$MODE-$NAME.live.yaml"; done
must dump_params /global_costmap/global_costmap "$LOG_DIR/$MODE-global_costmap.live.yaml"
must dump_params /local_costmap/local_costmap "$LOG_DIR/$MODE-local_costmap.live.yaml"
must compare_params "$RUNTIME_DIR/fast_lio.generated.yaml" '/**' "$LOG_DIR/$MODE-fast_lio.live.yaml"
must compare_params "$RUNTIME_DIR/gicp.generated.yaml" gicp_localization "$LOG_DIR/$MODE-gicp.live.yaml" prior_map_path
must compare_params "$RUNTIME_DIR/nav2.generated.yaml" map_server "$LOG_DIR/$MODE-map_server.live.yaml" yaml_filename
for NAME in planner_server controller_server behavior_server bt_navigator; do must compare_params "$RUNTIME_DIR/nav2.generated.yaml" "$NAME" "$LOG_DIR/$MODE-$NAME.live.yaml"; done
must compare_params "$RUNTIME_DIR/nav2.generated.yaml" 'global_costmap|global_costmap' "$LOG_DIR/$MODE-global_costmap.live.yaml"
must compare_params "$RUNTIME_DIR/nav2.generated.yaml" 'local_costmap|local_costmap' "$LOG_DIR/$MODE-local_costmap.live.yaml"
```

地图路径是 runtime-only 覆盖；只核对本次快照与 live dump 的路径，不写死值：

```bash
must python3 - "$LOG_DIR/$MODE-bringup.snapshot.yaml" \
  "$LOG_DIR/$MODE-gicp.live.yaml" "$LOG_DIR/$MODE-map_server.live.yaml" <<'PY'
from pathlib import Path
import sys, yaml
snapshot, gicp, nav2 = map(Path, sys.argv[1:])
cfg = yaml.safe_load(snapshot.read_text(encoding="utf-8"))["map_artifacts"]
def params(path):
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    roots = [v["ros__parameters"] for v in doc.values() if isinstance(v, dict) and isinstance(v.get("ros__parameters"), dict)]
    if len(roots) != 1: raise SystemExit(f"DIFFERENT {path}")
    return roots[0]
for label, want, got in (("prior_pcd", cfg["prior_pcd"], params(gicp)["prior_map_path"]),
                         ("nav2_map", cfg["nav2_map"], params(nav2)["yaml_filename"])):
    want, got = (str(Path(x).expanduser().resolve()) for x in (want, got))
    print(f"{'EQUAL' if want == got else 'DIFFERENT'} {label} snapshot={want} live={got}")
    if want != got: raise SystemExit(1)
PY
must cleanup
```

记录本模式的 `RUNTIME_DIR`；只有 cleanup 成功才结束运行验收。

## 6. 最终状态与报告

cleanup 后必须确认 mapping/navigation 的 PGID、其 ROS children、global 目标进程/节点均消失，且原配置 byte-exact 恢复：

```bash
must cmp -s "$CONFIG" "$ORIGINAL_CONFIG"
must stack_absent
must test -z "$(git diff -- "$CONFIG")"
must test -z "$(git status --porcelain --untracked-files=no)"
trap - EXIT INT TERM
```

报告写入 `.superpowers/sdd/6d-real-acceptance-report.md`，只用以下标题：

```markdown
# 6D Real Acceptance Report
## Metadata and baseline
## Build/test summary
## Mapping checks
## Navigation checks
## Cleanup and restored state
## Environment limitations
## Verdict
```

Metadata 记录时间（含时区）、主机、分支和 HEAD。两个模式各记录 runtime directory、输入快照、观察时长、typed projection 和 runtime map 的 `EQUAL`/`DIFFERENT`、清理结果及首个失败；列出因依赖失败未执行的步骤。普通日志留在仓库外。Verdict 只能为 `PASS`、`FAILED` 或 `BLOCKED`。
