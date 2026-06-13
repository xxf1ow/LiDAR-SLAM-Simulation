"""把 Classic Gazebo 世界(SDF 1.5/1.6)机械转换成 Gazebo Harmonic 世界(SDF 1.9)。
本机纯 stdlib 工具,无 ROS/Gz 依赖。用法见文件末 CLI。"""
import sys
import xml.etree.ElementTree as ET

# 不被 Harmonic 支持的 Classic 插件标记(出现即整模型删除)
CLASSIC_PLUGIN_MARKERS = ("ObjectDisposalPlugin",)

# 与 worlds/test_world.sdf 第 8-14 行一致的 Harmonic 系统插件(已验证可用)
_SYSTEM_PLUGINS_XML = """
<root>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
</root>"""


def get_world(tree):
    """取 <world> 元素(根 <sdf> 的唯一 world 子节点)。"""
    return tree.getroot().find("world")


def drop_state(world):
    """删除 Classic 运行时 <state> 快照块(Harmonic 不需要)。"""
    for state in world.findall("state"):
        world.remove(state)


def drop_classic_plugin_models(world):
    """删除任何 link/model 内挂了 Classic 专有插件的整个 <model>。"""
    for model in list(world.findall("model")):
        for plugin in model.iter("plugin"):
            filename = plugin.get("filename", "")
            if any(marker in filename for marker in CLASSIC_PLUGIN_MARKERS):
                world.remove(model)
                break


def strip_frame_attrs(elem):
    """剥掉所有元素上的 frame='' 属性(Classic 遗留,Harmonic 解析告警)。"""
    for node in elem.iter():
        node.attrib.pop("frame", None)


def neutralize_script_materials(world):
    """把含 <script> 的 <material> 替换为中性纯色(Harmonic Ogre2 不支持 Ogre1 script)。
    保留几何不动;仅去渲染告警并给可见灰色。"""
    for material in world.iter("material"):
        if material.find("script") is None:
            continue
        for child in list(material):
            material.remove(child)
        ET.SubElement(material, "ambient").text = "0.5 0.5 0.5 1"
        ET.SubElement(material, "diffuse").text = "0.6 0.6 0.6 1"
        ET.SubElement(material, "specular").text = "0.2 0.2 0.2 1"


def apply_harmonic_header(world):
    """世界名→factory;删 Classic <physics> 及多余环境块;在最前插入 5 系统插件 + 干净 physics。"""
    world.set("name", "factory")
    # 删 Classic 物理与环境噪声块(Harmonic 多余/告警)
    for tag in ("physics", "atmosphere", "magnetic_field", "wind", "spherical_coordinates"):
        for node in world.findall(tag):
            world.remove(node)
    # 组装要插入到 world 最前的节点:系统插件 + 干净 physics(默认引擎,1ms 步长)
    inserts = list(ET.fromstring(_SYSTEM_PLUGINS_XML))
    physics = ET.Element("physics", {"name": "default_physics"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    inserts.append(physics)
    for i, node in enumerate(inserts):
        world.insert(i, node)


def serialize(root):
    """序列化为带 XML 声明、SDF 1.9 的字符串。"""
    root.set("version", "1.9")
    body = ET.tostring(root, encoding="unicode")
    return '<?xml version="1.0" ?>\n' + body + "\n"


def convert_string(xml_str):
    """完整管线:Classic 世界 XML 字符串 → Harmonic 世界 XML 字符串。"""
    root = ET.fromstring(xml_str)
    tree = ET.ElementTree(root)
    world = get_world(tree)
    drop_state(world)
    drop_classic_plugin_models(world)
    neutralize_script_materials(world)
    apply_harmonic_header(world)
    strip_frame_attrs(root)          # 最后剥,确保新插入节点也干净
    return serialize(root)


def convert_file(src_path, dst_path):
    with open(src_path, "r", encoding="utf-8") as f:
        out = convert_string(f.read())
    with open(dst_path, "w", encoding="utf-8") as f:
        f.write(out)
    return out


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("用法: python convert_classic_world.py <src lio_world.model> <dst factory.sdf>")
    convert_file(sys.argv[1], sys.argv[2])
    print(f"已生成: {sys.argv[2]}")
