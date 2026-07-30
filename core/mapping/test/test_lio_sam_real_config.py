from pathlib import Path

import yaml


PATCH = Path(__file__).parents[1] / "lio-sam.patch"
README = Path(__file__).parents[1] / "README.md"


def _added_file(patch_text: str, relative_path: str) -> str:
    marker = f"diff --git a/{relative_path} b/{relative_path}"
    try:
        section = patch_text.split(marker, 1)[1].split("diff --git ", 1)[0]
    except IndexError:
        return ""

    return "\n".join(
        line[1:]
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )


def test_real_config_matches_accepted_vanjee_722_interface():
    patch_text = PATCH.read_text(encoding="utf-8")
    real_config = _added_file(patch_text, "config/params_real.yaml")

    assert real_config, "lio-sam.patch must add config/params_real.yaml"
    params = yaml.safe_load(real_config)["/**"]["ros__parameters"]

    assert params["use_sim_time"] is False
    assert params["pointCloudTopic"] == "/points_raw"
    assert params["imuTopic"] == "/imu/data"
    assert params["lidarFrame"] == "velodyne"
    assert params["baselinkFrame"] == "base_footprint"
    assert params["sensor"] == "velodyne"
    assert params["N_SCAN"] == 32
    assert params["Horizon_SCAN"] == 1200
    assert params["lidarMinRange"] == 0.3
    assert params["lidarMaxRange"] == 40.0
    assert params["savePCD"] is True
    assert params["savePCDDirectory"] == "/result/loam/"
    assert params["extrinsicTrans"] == [0.0, 0.0, 0.0]
    assert params["extrinsicRot"] == [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
    assert params["extrinsicRPY"] == [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]

    unsupported_vendor_keys = {
        "maxFeatureNum", "maxVel", "maxIter",
        "maxNoise", "accGating", "gyrGating",
    }
    assert unsupported_vendor_keys.isdisjoint(params)


def test_rviz_inherits_use_sim_time_from_selected_params_file():
    patch_text = PATCH.read_text(encoding="utf-8")

    assert "+            parameters=[parameter_file]," in patch_text
    assert "+            parameters=[{'use_sim_time': True}]" not in patch_text


def test_readme_documents_the_real_lio_sam_entrypoint():
    readme = README.read_text(encoding="utf-8")

    assert "params_real.yaml" in readme
    assert "params_file:=" in readme
    assert "32×1200" in readme
    assert "外参" in readme and "待实测" in readme
