import os
import xml.etree.ElementTree as ET

import convert_classic_world as cc

MINIMAL = """<sdf version='1.6'><world name='default'>
  <model name='keep_me'><static>1</static><link name='l'/></model>
  <state world_name='factory'><model name='keep_me'><pose>1 2 3 0 0 0</pose></model></state>
</world></sdf>"""

PLUGIN_WORLD = """<sdf version='1.6'><world name='default'>
  <model name='deletion_wall'><link name='l'>
    <plugin name='p' filename='libObjectDisposalPlugin.so'/></link></model>
  <model name='grey_wall'><static>1</static><link name='l'/></model>
</world></sdf>"""

FRAME_WORLD = """<sdf version='1.6'><world name='default'>
  <model name='m'><pose frame=''>1 2 3 0 0 0</pose>
  <link name='l'><pose frame=''>0 0 0 0 0 0</pose></link></model>
</world></sdf>"""

SCRIPT_WORLD = """<sdf version='1.6'><world name='default'>
  <model name='grey_wall'><link name='l'><visual name='v'>
    <material><script>
      <uri>model://grey_wall/materials/scripts</uri><name>vrc/grey_wall</name>
    </script></material>
  </visual></link></model>
</world></sdf>"""

HEADER_WORLD = """<sdf version='1.6'><world name='default'>
  <physics name='dp' type='ode'><max_step_size>0.001</max_step_size></physics>
  <model name='grey_wall'><static>1</static><link name='l'/></model>
</world></sdf>"""

FULL = """<sdf version='1.6'><world name='default'>
  <physics name='dp' type='ode'/>
  <state world_name='factory'><model name='grey_wall'><pose>1 1 0 0 0 0</pose></model></state>
  <model name='deletion_wall'><link name='l'>
    <plugin name='p' filename='libObjectDisposalPlugin.so'/></link></model>
  <model name='grey_wall'><static>1</static><link name='l'><pose frame=''>0 0 1 0 0 0</pose>
    <visual name='v'><material><script><name>vrc/grey_wall</name></script></material></visual>
  </link></model>
  <include><uri>model://robot</uri></include>
</world></sdf>"""

ROBOT_INCLUDE_WORLD = """<sdf version='1.6'><world name='default'>
  <model name='grey_wall'><static>1</static><link name='l'/></model>
  <include><uri>model://robot</uri><pose>0 3.5 0.36 0 3.14 0</pose></include>
</world></sdf>"""

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def test_apply_state_poses_overrides_top_level_pose():
    src = ("<sdf version='1.6'><world name='d'>"
           "<model name='box'><pose>1 1 0 0 0 0</pose><link name='l'/></model>"
           "<model name='nostate'><pose>9 9 0 0 0 0</pose><link name='l'/></model>"
           "<state world_name='d'>"
           "<model name='box'><pose>5 6 0 0 0 1.57</pose></model>"
           "</state></world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(src)))
    cc.apply_state_poses(world)
    assert world.find("model[@name='box']").findtext("pose").strip() == "5 6 0 0 0 1.57"
    assert world.find("model[@name='nostate']").findtext("pose").strip() == "9 9 0 0 0 0"


def test_drop_clutter_models_removes_listed_types():
    src = ("<sdf version='1.6'><world name='d'>"
           "<model name='coke_can'><link name='l'/></model>"
           "<model name='coke_can_clone_0'><link name='l'/></model>"
           "<model name='first_2015_trash_can_clone'><link name='l'/></model>"
           "<model name='grey_wall'><link name='l'/></model>"
           "</world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(src)))
    cc.drop_clutter_models(world)
    names = [m.get("name") for m in world.findall("model")]
    assert names == ["grey_wall"]      # coke_can/trash_can(含 clone)全删,墙保留


def test_drop_state_removes_all_state_blocks():
    world = cc.get_world(ET.ElementTree(ET.fromstring(MINIMAL)))
    cc.drop_state(world)
    assert world.find("state") is None
    assert [m.get("name") for m in world.findall("model")] == ["keep_me"]


def test_drop_classic_plugin_models_removes_only_marked():
    world = cc.get_world(ET.ElementTree(ET.fromstring(PLUGIN_WORLD)))
    cc.drop_classic_plugin_models(world)
    names = [m.get("name") for m in world.findall("model")]
    assert names == ["grey_wall"]


def test_strip_frame_attrs_removes_every_frame_attribute():
    root = ET.fromstring(FRAME_WORLD)
    cc.strip_frame_attrs(root)
    assert all("frame" not in e.attrib for e in root.iter())


def test_neutralize_script_materials_replaces_script_with_color():
    # 基本体(box)上的 script → 中性纯色
    world = cc.get_world(ET.ElementTree(ET.fromstring(SCRIPT_WORLD)))
    cc.neutralize_script_materials(world)
    mat = world.find(".//material")
    assert mat.find("script") is None
    assert mat.find("ambient") is not None
    assert mat.find("diffuse") is not None


def test_neutralize_skips_mesh_visuals_removes_material_to_keep_texture():
    # mesh visual 上的 script → 删整个 <material>,露出 mesh 自带纹理(勿误盖成灰)
    src = ("<sdf version='1.6'><world name='d'><model name='dumpster'><link name='l'>"
           "<visual name='v'><geometry><mesh><uri>model://dumpster/meshes/dumpster.dae</uri></mesh></geometry>"
           "<material><script><name>Gazebo/Grey</name></script></material></visual>"
           "</link></model></world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(src)))
    cc.neutralize_script_materials(world)
    vis = world.find(".//visual")
    assert vis.find("material") is None          # mesh 的材质被整体删除
    assert vis.find(".//mesh") is not None        # 几何保留


def test_ensure_ground_sinks_plane_below_zero():
    world = cc.get_world(ET.ElementTree(ET.fromstring(MINIMAL)))
    cc.ensure_ground(world)
    g = world.find("model[@name='ground']")
    assert g.findtext("pose").split()[2] == "-0.01"   # 地面下沉 1cm 防 z-fighting


def test_apply_harmonic_header_sets_name_plugins_and_drops_physics():
    tree = ET.ElementTree(ET.fromstring(HEADER_WORLD))
    world = cc.get_world(tree)
    cc.apply_harmonic_header(world)
    assert world.get("name") == "factory"
    # 不写 <physics>:gz-sim-physics-system 插件提供物理;SDF physics 需 type 属性,
    # 显式写会在 Gz8 解析报错。与已验证的 test_world.sdf(无 physics 块)一致。
    assert world.findall("physics") == []
    filenames = {p.get("filename") for p in world.findall("plugin")}
    assert filenames == {
        "gz-sim-physics-system", "gz-sim-sensors-system", "gz-sim-imu-system",
        "gz-sim-scene-broadcaster-system", "gz-sim-user-commands-system",
    }
    assert world.find("model[@name='grey_wall']") is not None


def test_serialize_sets_sdf_version_1_9():
    tree = ET.ElementTree(ET.fromstring(HEADER_WORLD))
    out = cc.serialize(tree.getroot())
    assert 'version="1.9"' in out or "version='1.9'" in out


def test_convert_pipeline_end_to_end():
    out = cc.convert_string(FULL)
    assert "ObjectDisposalPlugin" not in out
    assert "<state" not in out
    assert "frame=" not in out
    assert "<script" not in out
    assert 'name="factory"' in out
    assert "gz-sim-sensors-system" in out
    assert "grey_wall" in out
    assert "model://robot" not in out      # 老机器人 include 已删
    assert "<plane" in out                  # 地面已补


def test_drop_includes_removes_embedded_robot():
    world = cc.get_world(ET.ElementTree(ET.fromstring(ROBOT_INCLUDE_WORLD)))
    cc.drop_includes(world)
    assert world.find("include") is None
    assert world.find("model[@name='grey_wall']") is not None


def test_ensure_ground_adds_plane_when_missing():
    world = cc.get_world(ET.ElementTree(ET.fromstring(MINIMAL)))
    cc.ensure_ground(world)
    assert world.find(".//plane") is not None


def test_ensure_ground_idempotent_when_present():
    world = ET.fromstring(
        "<world name='d'><model name='ground'><link name='l'><visual name='v'>"
        "<geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>"
        "</visual></link></model></world>"
    )
    cc.ensure_ground(world)
    assert len(world.findall(".//plane")) == 1


def test_dedupe_link_geometry_names_makes_collisions_unique():
    dup = ("<sdf version='1.6'><world name='d'><model name='m'><link name='l'>"
           "<visual name='v1'/><collision name='c'/>"
           "<visual name='v2'/><collision name='c'/><collision name='c'/>"
           "</link></model></world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(dup)))
    cc.dedupe_link_geometry_names(world)
    names = [c.get("name") for c in world.iter("collision")]
    assert len(names) == len(set(names))            # 全唯一
    assert {"c", "c_dup2", "c_dup3"} == set(names)


def test_strip_use_parent_model_frame_removes_it():
    upmf = ("<sdf version='1.6'><world name='d'><model name='m'><link name='l'/>"
            "<joint name='j' type='revolute'><axis><xyz>0 0 1</xyz>"
            "<use_parent_model_frame>1</use_parent_model_frame></axis></joint>"
            "</model></world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(upmf)))
    cc.strip_use_parent_model_frame(world)
    assert world.find(".//use_parent_model_frame") is None
    assert world.find(".//axis/xyz") is not None    # axis 其它内容保留


def test_strip_gui_removes_gui_block():
    gui = ("<sdf version='1.6'><world name='d'>"
           "<gui fullscreen='0'><camera name='user_camera'><pose>1 2 3 0 0 0</pose></camera></gui>"
           "<model name='m'><link name='l'/></model></world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(gui)))
    cc.strip_gui(world)
    assert world.find("gui") is None
    assert world.find("model[@name='m']") is not None


def test_strip_extra_lights_keeps_only_directional_sun():
    w = ("<sdf version='1.6'><world name='d'>"
         "<light name='sun' type='directional'/>"
         "<light name='spot1' type='spot'/><light name='spot2' type='spot'/>"
         "<model name='m'><link name='l'><light name='lamp' type='point'/></link></model>"
         "</world></sdf>")
    world = cc.get_world(ET.ElementTree(ET.fromstring(w)))
    cc.strip_extra_lights(world)
    lights = list(world.iter("light"))
    assert len(lights) == 1 and lights[0].get("name") == "sun"   # 仅留 directional sun


def test_generated_factory_sdf_is_sane():
    path = os.path.join(REPO, "core", "simulation", "robot_gz_bringup", "worlds", "factory.sdf")
    if not os.path.exists(path):
        import pytest
        pytest.skip("factory.sdf 尚未生成")
    tree = ET.parse(path)
    world = cc.get_world(tree)
    assert world.get("name") == "factory"
    assert world.find("state") is None
    assert "ObjectDisposalPlugin" not in ET.tostring(tree.getroot(), encoding="unicode")
    assert len(world.findall("model")) >= 50   # 62 - 2 插件 - 7 杂物(coke/trash)+ ground 余量
    serialized = ET.tostring(tree.getroot(), encoding="unicode")
    assert "model://robot" not in serialized   # 不嵌老机器人
    assert world.find(".//plane") is not None   # 有地面
    assert "use_parent_model_frame" not in serialized   # Classic 元素已清
    assert world.find("gui") is None                     # Classic <gui> 块已删
    # 模型内光源已清,仅留世界级 sun
    assert all(l.get("type") == "directional" for l in world.findall("light"))
    assert len(list(world.iter("light"))) == len(world.findall("light"))  # 无模型内嵌 light
    # 每个 link 内 collision/visual 名唯一(Gz Sim 严格)
    for link in world.iter("link"):
        for tag in ("collision", "visual"):
            names = [e.get("name") for e in link.findall(tag)]
            assert len(names) == len(set(names)), f"{tag} 重名 in link {link.get('name')}"
