"""nav2_costmaps.yaml 与 nav2_costmaps_stvl.yaml 一致性抽查（spec §6.3）。

两文件约定：唯一结构差异 = local_costmap 障碍层；其余各段逐字一致。
改主 yaml 共享参数（如 DWB 调速）忘了同步 STVL 版 → 本测试失败。
"""
import os

import yaml

_CFG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def _load(name):
    with open(os.path.join(_CFG_DIR, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_shared_sections_identical():
    """除 local_costmap 外的所有顶层段必须深度相等。"""
    base, stvl = _load("nav2_costmaps.yaml"), _load("nav2_costmaps_stvl.yaml")
    assert set(base) == set(stvl)
    for section in base:
        if section == "local_costmap":
            continue
        assert base[section] == stvl[section], f"{section} 段与主 yaml 失同步"


def test_local_costmap_shared_params_identical():
    """local_costmap 内除插件列表与障碍层本体外的共享参数也须一致。"""
    base = _load("nav2_costmaps.yaml")["local_costmap"]["local_costmap"]["ros__parameters"]
    stvl = _load("nav2_costmaps_stvl.yaml")["local_costmap"]["local_costmap"]["ros__parameters"]
    for key in ("global_frame", "robot_base_frame", "rolling_window",
                "width", "height", "resolution", "footprint",
                "update_frequency", "publish_frequency"):
        assert base[key] == stvl[key], f"local_costmap.{key} 失同步"
    assert base["inflation_layer"] == stvl["inflation_layer"]


def test_stvl_layer_uses_body_cloud():
    """STVL 观测源必须是 body 系点云（阶段三修复；勿回退到 /points_raw）。"""
    p = _load("nav2_costmaps_stvl.yaml")["local_costmap"]["local_costmap"]["ros__parameters"]
    assert p["plugins"] == ["stvl_layer", "inflation_layer"]
    layer = p["stvl_layer"]
    assert layer["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert layer["pointcloud_mark"]["topic"] == "/cloud_registered_body"
    assert layer["pointcloud_clear"]["topic"] == "/cloud_registered_body"
    assert layer["pointcloud_clear"]["model_type"] == 1   # 3D Lidar 视锥
