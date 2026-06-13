"""SDF 模板渲染：占位符替换 + 模板与 planar_move 插件结构健全性。"""
import os
import xml.etree.ElementTree as ET

from sim_obstacles.sdf_template import render_sdf

TEMPLATE = os.path.join(os.path.dirname(__file__), "..", "models", "obstacle.sdf.in")


def _rendered(name="obs_test"):
    with open(TEMPLATE, "r", encoding="utf-8") as f:
        return render_sdf(f.read(), name)


def test_no_placeholder_left():
    assert "@NAME@" not in _rendered()


def test_model_name_and_namespace_injected():
    root = ET.fromstring(_rendered("obs_7"))
    assert root.find("model").get("name") == "obs_7"
    ns = root.find("model/plugin/ros/namespace")
    assert ns.text == "/obs_7"


def test_planar_move_plugin_publishes_nothing():
    """odom/TF 必须关闭（spec §3：障碍不进 TF 树）。"""
    root = ET.fromstring(_rendered())
    plugin = root.find("model/plugin")
    assert plugin.get("filename") == "libgazebo_ros_planar_move.so"
    assert plugin.find("publish_odom").text == "false"
    assert plugin.find("publish_odom_tf").text == "false"


def test_uses_box_geometry():
    """障碍是立方体（非圆柱）：实测圆柱细高易翻倒，改矮胖立方体。"""
    root = ET.fromstring(_rendered())
    assert root.find("model/link/collision/geometry/cylinder") is None
    box = root.find("model/link/collision/geometry/box")
    assert box is not None
    assert box.find("size").text == "0.8 0.8 0.8"
    vbox = root.find("model/link/visual/geometry/box")
    assert vbox.find("size").text == "0.8 0.8 0.8"


def test_gravity_disabled():
    """关重力：根除 planar_move 100Hz 插值窗口内的渐进翻倒（实测反馈）。"""
    root = ET.fromstring(_rendered())
    assert root.find("model/link/gravity").text == "false"
