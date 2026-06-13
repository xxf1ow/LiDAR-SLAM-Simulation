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
</world></sdf>"""

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


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
    world = cc.get_world(ET.ElementTree(ET.fromstring(SCRIPT_WORLD)))
    cc.neutralize_script_materials(world)
    mat = world.find(".//material")
    assert mat.find("script") is None
    assert mat.find("ambient") is not None
    assert mat.find("diffuse") is not None


def test_apply_harmonic_header_sets_name_plugins_and_physics():
    tree = ET.ElementTree(ET.fromstring(HEADER_WORLD))
    world = cc.get_world(tree)
    cc.apply_harmonic_header(world)
    assert world.get("name") == "factory"
    physics = world.findall("physics")
    assert len(physics) == 1 and physics[0].get("type") in (None, "")
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
    assert len(world.findall("model")) >= 55
