# Profile 迁移 6D Navigation-only 复验

## 1. 适用边界

本流程只用于复验 `nav2.yaml` 删除无效 `AckermannConstraints` 和 STVL
`origin_z` 后的差异。原 6D 报告中的 mapping、传感器频率、TF、
FAST-LIO、GICP 和 runtime map 路径已经通过，本流程不重复它们。

仍然禁止发送 goal、移动底盘、调参、保存地图、运行 `save_map.sh` 或
push。任一检查或 cleanup 失败即为 `FAILED`；环境或基线不满足为
`BLOCKED`。

## 2. 复用原手册的安全框架

打开同一提交中的
`docs/acceptance/profile-migration-real-acceptance.md`：

1. 执行其第 1 节边界。
2. 执行第 2 节 shell block 中的变量初始化、exact HEAD / single
   worktree / clean tree 检查、原配置备份、stack regex、ROS source 和
   首次 `stack_absent`。把 `EXPECTED_HEAD` 填为本次 review-approved
   exact commit；跳过 `MAP_PATHS` 可读性段和 full-workspace
   `colcon build/test`，它们均已在原 6D 通过且不受本次修改影响。
3. 改为只安装本次受影响包：

```bash
cd "$CORE"
colcon build --packages-select system_bringup
source install/setup.bash
```

4. 定义原手册第 3 节的全部共用函数和 trap。不要修改其有界
   TERM/KILL、byte-exact restore 或 residual-process checks。

## 3. real + navigation 差异复验

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

for node in /map_server /planner_server /controller_server /behavior_server /bt_navigator; do
  must check_lifecycle "$node"
done

must dump_params /controller_server "$LOG_DIR/navigation-controller.yaml"
must dump_params /global_costmap/global_costmap "$LOG_DIR/navigation-global-costmap.yaml"
must dump_params /local_costmap/local_costmap "$LOG_DIR/navigation-local-costmap.yaml"

must compare_params "$RUNTIME_DIR/nav2.generated.yaml" \
  'controller_server' "$LOG_DIR/navigation-controller.yaml"
must compare_params "$RUNTIME_DIR/nav2.generated.yaml" \
  'global_costmap|global_costmap' "$LOG_DIR/navigation-global-costmap.yaml"
must compare_params "$RUNTIME_DIR/nav2.generated.yaml" \
  'local_costmap|local_costmap' "$LOG_DIR/navigation-local-costmap.yaml"

must cleanup
must cmp -s "$CONFIG" "$ORIGINAL_CONFIG"
must stack_absent
must test -z "$(git diff -- "$CONFIG")"
must test -z "$(git status --porcelain --untracked-files=no)"
trap - EXIT INT TERM
```

三个 typed projection 必须全部输出 `EQUAL`。不要把任何 generated-owned
字段加入 ignored 列表，也不要用临时 template 修改取得结果。

## 4. 报告

报告写入忽略路径
`.superpowers/sdd/6d-navigation-recheck-report.md`，使用以下格式：

```markdown
# 6D Navigation-only Recheck Report
## Metadata and baseline
- Time / host / branch / exact HEAD
- Original failed report
## Affected build
- system_bringup build result
## Navigation recheck
- Runtime directory
- Five lifecycle states
- controller_server: EQUAL / DIFFERENT
- global_costmap: EQUAL / DIFFERENT
- local_costmap: EQUAL / DIFFERENT
## Cleanup and restored state
- Config byte-exact restored
- Stack absent
- Tracked tree clean
## Verdict
PASS / FAILED / BLOCKED
```

只有三项 projection 全为 `EQUAL`、五个 lifecycle 节点均为 `active [3]`
且 cleanup 完整通过时，Verdict 才能写 `PASS`。此后才更新
`PROGRESS.md` 将 6D 标记完成。
