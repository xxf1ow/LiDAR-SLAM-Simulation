"""话题就绪闸门(非阻塞)+ settling。支持等单个或多个话题(都出现才放行)。

两段语义:
  ① 就绪:等 needed 话题(单个或列表)都出现在 DDS —— 替代"等启动"的大延迟。
     例如 robot_gz gate 等 ['/points_raw','/joint_states']:lidar 和 controller 都就绪。
  ② settling:都出现后再 sleep 一小段(可配) —— 给模块稳定(controller 激活、gicp
     收敛等)。第一条消息 ≠ 完全就绪,稍等更稳。

实现:ExecuteProcess 跑 python 子进程,轮询 get_topic_names();全部出现则 sleep settling
后 exit 0 → OnProcessExit 触发 then_actions;轮询到 timeout 仍缺则 exit 1 → raise。
全程异步(子进程),不阻塞 launch event loop。

返回值是 list [waiter, handler],调用处用 + 拼接进 actions。settling 在第 5 参数覆盖。
"""
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def _fail_action(label, needed, timeout):
    def _fn(context, *args, **kwargs):
        raise RuntimeError(
            f"就绪闸门超时:{label} —— 等了 {timeout:.0f}s 仍缺话题 {needed}"
            f"(上游未就绪),已中止整个 launch。")
    return OpaqueFunction(function=_fn)


def _gate_script(needed, timeout, settling):
    needed_list = [needed] if isinstance(needed, str) else list(needed)
    return (
        "import sys, time, rclpy\n"
        "from rclpy.node import Node\n"
        "rclpy.init()\n"
        "n = Node('ready_gate')\n"
        f"NEEDED = {needed_list!r}\n"
        f"END = time.monotonic() + {timeout}\n"
        f"SETTLING = {settling}\n"
        "ok = False\n"
        "missing = NEEDED[:]\n"
        "while time.monotonic() < END:\n"
        "    have = set(dict(n.get_topic_names_and_types()))\n"
        "    missing = [t for t in NEEDED if t not in have]\n"
        "    if not missing:\n"
        "        ok = True\n"
        "        break\n"
        "    time.sleep(0.5)\n"
        "if ok:\n"
        "    print('[ready_gate]', NEEDED, 'seen, settling', SETTLING, 's')\n"
        "    time.sleep(SETTLING)\n"
        "else:\n"
        f"    print('[ready_gate]', NEEDED, 'TIMEOUT after', {timeout}, 's, missing=', missing)\n"
        "sys.exit(0 if ok else 1)\n"
    )


def ready_gate(needed, timeout, label, then_actions, settling=3.0):
    """返回 [waiter, handler]。needed(str 或 list)全部出现 + settling 后触发 then_actions;超时 raise。"""
    waiter = ExecuteProcess(
        cmd=["/usr/bin/python3", "-c", _gate_script(needed, timeout, settling)],
        name=f"gate_{abs(hash(str(needed))) % 100000}", output="screen")
    handler = RegisterEventHandler(OnProcessExit(
        target_action=waiter,
        on_exit=lambda event, config: (then_actions if event.returncode == 0
                                       else [_fail_action(label, needed, timeout)]),
    ))
    return [waiter, handler]
