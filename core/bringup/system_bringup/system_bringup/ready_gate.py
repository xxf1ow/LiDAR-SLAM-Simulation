"""话题就绪闸门(非阻塞)+ settling。支持等单个或多个话题(都出现才放行)。

两段语义:
  ① 就绪:等 needed 话题(单个或列表)都出现在 DDS —— 替代"等启动"的大延迟。
     例如 robot_gz gate 等 ['/points_raw','/joint_states']:lidar 和 controller 都就绪。
  ② settling:都出现后按 ROS 节点时钟等待一小段(可配) —— 给模块稳定
     (controller 激活、gicp 收敛等)。第一条消息 ≠ 完全就绪,稍等更稳。

实现:ExecuteProcess 跑 python 子进程,轮询 get_topic_names();全部出现则按节点时钟
settling 后 exit 0 → OnProcessExit 触发 then_actions;话题发现达到 discovery timeout
则 exit 1 → raise。settling 不使用墙钟 timeout,仿真暂停时保持等待,恢复后继续。
全程异步(子进程),不阻塞 launch event loop。

返回值是 list [waiter, handler],调用处用 + 拼接进 actions。
"""
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def _fail_action(label, needed, discovery_timeout):
    def _fn(context, *args, **kwargs):
        raise RuntimeError(
            f"就绪闸门超时:{label} —— 等了 {discovery_timeout:.0f}s 仍缺话题 {needed}"
            f"(上游未就绪),已中止整个 launch。")
    return OpaqueFunction(function=_fn)


def _gate_script(needed, discovery_timeout, settling):
    needed_list = [needed] if isinstance(needed, str) else list(needed)
    return (
        "import sys, time, rclpy\n"
        "from rclpy.node import Node\n"
        "rclpy.init()\n"
        "n = Node('ready_gate')\n"
        f"NEEDED = {needed_list!r}\n"
        f"DISCOVERY_TIMEOUT = {discovery_timeout}\n"
        f"SETTLING = {settling}\n"
        "DISCOVERY_DEADLINE = time.monotonic() + DISCOVERY_TIMEOUT\n"
        "ok = False\n"
        "missing = NEEDED[:]\n"
        "while time.monotonic() < DISCOVERY_DEADLINE:\n"
        "    have = set(dict(n.get_topic_names_and_types()))\n"
        "    missing = [t for t in NEEDED if t not in have]\n"
        "    if not missing:\n"
        "        ok = True\n"
        "        break\n"
        "    time.sleep(0.5)\n"
        "if ok:\n"
        "    print('\\n======== [ready_gate]', NEEDED, '就绪 → settling', SETTLING, 's ========')\n"
        "    SETTLING_NS = int(SETTLING * 1000000000)\n"
        "    while n.get_clock().now().nanoseconds == 0:\n"
        "        rclpy.spin_once(n, timeout_sec=0.1)\n"
        "if ok:\n"
        "    SETTLING_START = n.get_clock().now().nanoseconds\n"
        "    while n.get_clock().now().nanoseconds - SETTLING_START < SETTLING_NS:\n"
        "        rclpy.spin_once(n, timeout_sec=0.1)\n"
        "if not ok:\n"
        "    print('\\n======== [ready_gate]', NEEDED, 'TIMEOUT after', DISCOVERY_TIMEOUT, 's, missing=', missing, ' ========')\n"
        "sys.exit(0 if ok else 1)\n"
    )


def ready_gate(
    needed, discovery_timeout, label, then_actions, use_sim_time, settling=3.0
):
    """话题需在 discovery_timeout 内出现;settling 完成后触发 then_actions。"""
    use_sim_time_value = str(use_sim_time).lower()
    waiter = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", "-c",
            _gate_script(needed, discovery_timeout, settling),
            "--ros-args", "-p", f"use_sim_time:={use_sim_time_value}",
        ],
        name=f"gate_{abs(hash(str(needed))) % 100000}", output="screen")
    handler = RegisterEventHandler(OnProcessExit(
        target_action=waiter,
        on_exit=lambda event, config: (then_actions if event.returncode == 0
                                       else [_fail_action(
                                           label, needed, discovery_timeout
                                       )]),
    ))
    return [waiter, handler]
