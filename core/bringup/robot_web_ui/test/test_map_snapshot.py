import gzip
import struct
from pathlib import Path

import pytest

from robot_web_ui.map_snapshot import (
    GridInfo,
    load_nav2_pgm,
    update_grid_snapshot,
    update_path_snapshot,
)


def write_map(tmp_path, pixels=b"\x00\xfe\xcd\xfe", width=2, height=2):
    pgm = tmp_path / "map.pgm"
    pgm.write_bytes(b"P5\n2 2\n255\n" + pixels)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\n"
        "resolution: 0.2\n"
        "origin: [1.5, -2.0, 0.25]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "mode: trinary\n",
        encoding="utf-8",
    )
    return yaml_path


def test_load_nav2_pgm_preserves_geometry_and_flips_image_rows(tmp_path):
    yaml_path = write_map(tmp_path)

    snapshot = load_nav2_pgm(yaml_path)

    assert snapshot.info == GridInfo(2, 2, 0.2, 1.5, -2.0, 0.25, "map")
    assert snapshot.info.as_dict() == {
        "width": 2,
        "height": 2,
        "resolution": 0.2,
        "origin": [1.5, -2.0, 0.25],
        "frame_id": "map",
    }

    # Nav2 uses image rows from top to bottom while ROS grid rows are bottom first.
    thresholds = (0.65, 0.25)

    def category(pixel):
        occupancy = (255 - pixel) / 255
        if occupancy > thresholds[0]:
            return 100
        if occupancy < thresholds[1]:
            return 0
        return 255

    assert snapshot.binary.data == bytes(
        category(pixel) for pixel in (0xCD, 0xFE, 0x00, 0xFE)
    )


def test_load_nav2_pgm_resolves_image_relative_to_yaml(tmp_path, monkeypatch):
    yaml_path = write_map(tmp_path)
    monkeypatch.chdir(tmp_path.parent)

    snapshot = load_nav2_pgm(yaml_path)

    assert snapshot.info.width == 2
    assert len(snapshot.binary.data) == 4


def test_load_nav2_pgm_preserves_hash_raster_byte(tmp_path):
    pgm = tmp_path / "hash.pgm"
    pgm.write_bytes(b"P5\n1 1\n255\n#")
    yaml_path = tmp_path / "hash.yaml"
    yaml_path.write_text(
        "image: hash.pgm\n"
        "resolution: 0.2\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "mode: trinary\n",
        encoding="utf-8",
    )

    snapshot = load_nav2_pgm(yaml_path)

    assert snapshot.info.width == 1
    assert snapshot.info.height == 1
    assert snapshot.binary.data == b"\x64"


@pytest.mark.parametrize(
    ("case", "expected"),
    [
        ("missing_image", "image"),
        ("wrong_resolution_type", "resolution"),
        ("bad_origin", "origin"),
        ("unsupported_mode", "mode"),
        ("bad_magic", "P5"),
        ("wrong_pixel_count", "pixel"),
    ],
)
def test_load_nav2_pgm_rejects_invalid_project_maps(tmp_path, case, expected):
    yaml_path = write_map(tmp_path)
    pgm = tmp_path / "map.pgm"

    if case == "missing_image":
        yaml_path.write_text(
            yaml_path.read_text(encoding="utf-8").replace("image: map.pgm\n", ""),
            encoding="utf-8",
        )
    elif case == "wrong_resolution_type":
        yaml_path.write_text(
            yaml_path.read_text(encoding="utf-8").replace("resolution: 0.2", "resolution: true"),
            encoding="utf-8",
        )
    elif case == "bad_origin":
        yaml_path.write_text(
            yaml_path.read_text(encoding="utf-8").replace("origin: [1.5, -2.0, 0.25]", "origin: [1.5, -2.0]"),
            encoding="utf-8",
        )
    elif case == "unsupported_mode":
        yaml_path.write_text(
            yaml_path.read_text(encoding="utf-8").replace("mode: trinary", "mode: scale"),
            encoding="utf-8",
        )
    elif case == "bad_magic":
        pgm.write_bytes(b"P2\n2 2\n255\n" + b"\x00\xfe\xcd\xfe")
    elif case == "wrong_pixel_count":
        pgm.write_bytes(b"P5\n2 2\n255\n" + b"\x00\xfe\xcd")

    with pytest.raises(ValueError, match=expected):
        load_nav2_pgm(yaml_path)


def test_equal_grid_content_and_metadata_reuses_revision_and_compressed_bytes():
    info = GridInfo(2, 2, 0.2, 1.5, -2.0, 0.25, "map")
    first = update_grid_snapshot(None, info, b"\x00\x64\xff\x00")

    second = update_grid_snapshot(first, info, b"\x00\x64\xff\x00")

    assert second is first
    assert second.binary.revision == 1
    assert gzip.decompress(second.binary.gzip_data) == second.binary.data


def test_grid_data_or_metadata_change_increments_revision():
    info = GridInfo(2, 2, 0.2, 1.5, -2.0, 0.25, "map")
    first = update_grid_snapshot(None, info, b"\x00\x64\xff\x00")
    data_changed = update_grid_snapshot(first, info, b"\x00\x64\xff\x64")
    metadata_changed = update_grid_snapshot(data_changed, GridInfo(2, 2, 0.3, 1.5, -2.0, 0.25, "map"), data_changed.binary.data)

    assert data_changed.binary.revision == first.binary.revision + 1
    assert metadata_changed.binary.revision == data_changed.binary.revision + 1


def test_gzip_body_round_trips_exact_bytes():
    info = GridInfo(1, 2, 0.2, 0.0, 0.0, 0.0, "map")
    snapshot = update_grid_snapshot(None, info, b"\x00\xff")

    assert gzip.decompress(snapshot.binary.gzip_data) == snapshot.binary.data


def test_generic_grid_snapshot_preserves_inflated_cost_value():
    info = GridInfo(1, 1, 0.2, 0.0, 0.0, 0.0, "map")

    snapshot = update_grid_snapshot(None, info, b"\x25")

    assert snapshot.binary.data == b"\x25"


def test_path_snapshot_is_little_endian_float32_and_revisioned():
    first = update_path_snapshot(None, "map", [(1.25, -2.5), (3.5, 4.25)])

    assert struct.unpack("<ffff", first.binary.data) == (1.25, -2.5, 3.5, 4.25)
    assert update_path_snapshot(first, "map", [(1.25, -2.5), (3.5, 4.25)]) is first

    changed = update_path_snapshot(first, "map", [(1.25, -2.5), (3.5, 4.5)])
    assert changed.binary.revision == first.binary.revision + 1
    assert changed.frame_id == "map"
