"""把 Classic Gazebo 世界(SDF 1.5/1.6)机械转换成 Gazebo Harmonic 世界(SDF 1.9)。
本机纯 stdlib 工具,无 ROS/Gz 依赖。用法见文件末 CLI。"""
import re
import sys
import xml.etree.ElementTree as ET

# 不被 Harmonic 支持的 Classic 插件标记(出现即整模型删除)
CLASSIC_PLUGIN_MARKERS = ("ObjectDisposalPlugin",)

# 高三角面、低 SLAM 价值的小杂物:删以提 RTF(软件渲染下 gpu_lidar 每周期渲全场景)。
# 可乐罐×4(5120 面/个)+ 垃圾桶×3(5764 面/个)≈ 全场景 50% 三角面,删之砍半、SLAM 无损。
DROP_CLUTTER_TYPES = {"coke_can", "first_2015_trash_can"}

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


def apply_state_poses(world):
    """用 <state> 块里的最终位姿覆盖顶层 <model> 的 <pose>。
    Classic 存盘后,物体被编辑器摆动后的"真实最终布局"在 <state> 里,顶层 <model><pose>
    可能是过时/初始值(本工厂 62 个里 47 个两处不一致)。必须在 drop_state 之前调用。"""
    state_pose = {}
    for state in world.findall("state"):
        for sm in state.findall("model"):
            pe = sm.find("pose")
            if pe is not None and pe.text and pe.text.strip():
                state_pose[sm.get("name")] = pe.text.strip()
    for model in world.findall("model"):
        name = model.get("name")
        if name not in state_pose:
            continue
        pe = model.find("pose")
        if pe is None:
            pe = ET.SubElement(model, "pose")
        pe.text = state_pose[name]


def drop_state(world):
    """删除 Classic 运行时 <state> 快照块(Harmonic 不需要)。"""
    for state in world.findall("state"):
        world.remove(state)


def _base_type(name):
    """实例名 -> 基础类型(剥 _clone* 与尾部 _数字)。"""
    return re.sub(r"_\d+$", "", re.sub(r"_clone.*$", "", name or ""))


def drop_clutter_models(world):
    """删除 DROP_CLUTTER_TYPES 里的高面数低价值小杂物(提 RTF)。"""
    for model in list(world.findall("model")):
        if _base_type(model.get("name")) in DROP_CLUTTER_TYPES:
            world.remove(model)


def drop_classic_plugin_models(world):
    """删除任何 link/model 内挂了 Classic 专有插件的整个 <model>。"""
    for model in list(world.findall("model")):
        for plugin in model.iter("plugin"):
            filename = plugin.get("filename", "")
            if any(marker in filename for marker in CLASSIC_PLUGIN_MARKERS):
                world.remove(model)
                break


def drop_includes(world):
    """删除世界里的 <include>(本世界仅嵌了老 robot 模型;新机器人由 launch spawn)。"""
    for inc in world.findall("include"):
        world.remove(inc)


def ensure_ground(world):
    """若世界无地面(无 plane 几何),追加一个 Harmonic 地面平面(机器人需落地)。
    100x100 覆盖整个工厂(x∈[-6,30], y∈[-9.5,8])。"""
    if world.find(".//plane") is not None:
        return
    # 下沉 1cm:避免与模型自带 floor(shipping_container 等)在 z=0 共面引发 z-fighting
    # (拖动视角时地面闪烁)。机器人落差 1cm 可忽略。
    ground_xml = (
        '<model name="ground"><static>true</static><pose>0 0 -0.01 0 0 0</pose><link name="link">'
        '<collision name="collision"><geometry>'
        '<plane><normal>0 0 1</normal><size>100 100</size></plane>'
        '</geometry></collision>'
        '<visual name="visual"><geometry>'
        '<plane><normal>0 0 1</normal><size>100 100</size></plane>'
        '</geometry><material>'
        '<ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse>'
        '</material></visual>'
        '</link></model>'
    )
    world.append(ET.fromstring(ground_xml))


def strip_frame_attrs(elem):
    """剥掉所有元素上的 frame='' 属性(Classic 遗留,Harmonic 解析告警)。"""
    for node in elem.iter():
        node.attrib.pop("frame", None)


def neutralize_script_materials(world):
    """处理 Ogre1 `<material><script>`(Harmonic Ogre2 不支持):
      - **基本体(box/cylinder)visual**:把 script 材质换成中性纯色(墙/柜本就灰,无碍);
      - **mesh visual**:直接删整个 <material>,露出 mesh(.dae/.obj)自带的纹理材质
        (否则会把 Dumpster/disk_part/plastic_cup 等带纹理 mesh 误盖成灰)。
    只动 visual 里的 <material>(SDF 中 material 仅存在于 visual)。"""
    for visual in world.iter("visual"):
        material = visual.find("material")
        if material is None or material.find("script") is None:
            continue
        if visual.find(".//mesh") is not None:
            visual.remove(material)          # mesh:删材质,保留自带纹理
            continue
        for child in list(material):         # 基本体:script → 中性纯色
            material.remove(child)
        ET.SubElement(material, "ambient").text = "0.5 0.5 0.5 1"
        ET.SubElement(material, "diffuse").text = "0.6 0.6 0.6 1"
        ET.SubElement(material, "specular").text = "0.2 0.2 0.2 1"


def apply_harmonic_header(world):
    """世界名→factory;删 Classic <physics> 及多余环境块;在最前插入 5 系统插件。
    不写 <physics> 元素——gz-sim-physics-system 插件即提供物理(默认 1ms 步长);
    且 SDF 的 <physics> 必须带 type 属性,显式写易在 Gz8 解析报错
    (Required attribute[type] in element[physics]),与已验证的 test_world.sdf 一致(其无 physics 块)。"""
    world.set("name", "factory")
    # 删 Classic 物理与环境噪声块(Harmonic 多余/告警)
    for tag in ("physics", "atmosphere", "magnetic_field", "wind", "spherical_coordinates"):
        for node in world.findall(tag):
            world.remove(node)
    # 仅在 world 最前插入 5 个 Harmonic 系统插件(不写 <physics>)
    inserts = list(ET.fromstring(_SYSTEM_PLUGINS_XML))
    for i, node in enumerate(inserts):
        world.insert(i, node)


def dedupe_link_geometry_names(world):
    """每个 <link> 内,collision/visual 的 name 必须唯一(Gz Sim 严格,Classic 宽松)。
    重名追加 _dup<n> 后缀。修源工厂 workcell 里 conveyor_frame_*_collision 同名导致
    Gz8 'collision with name X already exists' → 世界加载失败。collision/visual 名不被
    其它元素引用,改名安全。"""
    for link in world.iter("link"):
        for tag in ("collision", "visual"):
            seen = set()
            for elem in link.findall(tag):
                name = elem.get("name", tag)
                if name in seen:
                    n = 2
                    new = f"{name}_dup{n}"
                    while new in seen:
                        n += 1
                        new = f"{name}_dup{n}"
                    elem.set("name", new)
                    seen.add(new)
                else:
                    seen.add(name)


def strip_use_parent_model_frame(world):
    """删除 Classic <use_parent_model_frame>(modern SDF 无此元素,Gz 仅警告;清掉去噪)。"""
    for axis in world.iter("axis"):
        for upmf in axis.findall("use_parent_model_frame"):
            axis.remove(upmf)
    for axis2 in world.iter("axis2"):
        for upmf in axis2.findall("use_parent_model_frame"):
            axis2.remove(upmf)


def strip_gui(world):
    """删除 Classic <gui> 块(含无法转换的 <camera user_camera>,Gz8 警告 'can't be converted')。
    Gz 用默认 GUI,与已验证的 test_world.sdf(无 <gui> 块)一致。"""
    for gui in world.findall("gui"):
        world.remove(gui)


def strip_extra_lights(world):
    """删除非 directional 光源(spot/point),保留 directional 的 sun。
    源工厂有 6 个 user_spot_light(聚光灯,投影开销大、软件渲染/演示卡顿),
    对 LiDAR 无意义;directional sun 便宜且够照明。世界级与模型内一并清。"""
    parent_map = {child: parent for parent in world.iter() for child in parent}
    for light in list(world.iter("light")):
        if light.get("type") != "directional":
            parent_map[light].remove(light)


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
    apply_state_poses(world)         # 先用 <state> 最终位姿覆盖顶层 pose,再删 state
    drop_state(world)
    drop_clutter_models(world)       # 删高面数低价值杂物(提 RTF)
    drop_classic_plugin_models(world)
    drop_includes(world)
    neutralize_script_materials(world)
    dedupe_link_geometry_names(world)
    strip_use_parent_model_frame(world)
    strip_gui(world)
    strip_extra_lights(world)
    ensure_ground(world)
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
