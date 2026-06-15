"""pcd_to_occupancy 纯逻辑单测（本机 numpy 可跑；不触碰 open3d IO）。"""
import numpy as np
import pytest

from robot_navigation import pcd_to_occupancy as p


def test_filter_height_band_keeps_only_band():
    pts = np.array([[0, 0, -1.0], [1, 1, 0.5], [2, 2, 1.0], [3, 3, 3.0]])
    out = p.filter_height_band(pts, z_min=0.1, z_max=2.0)
    assert out.shape == (2, 3)
    assert set(out[:, 2].tolist()) == {0.5, 1.0}


def test_filter_height_band_is_inclusive():
    pts = np.array([[0, 0, 0.1], [0, 0, 2.0]])
    out = p.filter_height_band(pts, z_min=0.1, z_max=2.0)
    assert out.shape == (2, 3)


def test_points_to_occupancy_origin_is_min_corner():
    xy = np.array([[1.0, 2.0], [1.4, 2.4]])
    grid, origin = p.points_to_occupancy(xy, resolution=0.5, min_pts=1)
    assert origin.tolist() == [1.0, 2.0]
    assert grid.shape == (2, 2)
    assert grid[0, 0] == 100


def test_points_to_occupancy_marks_and_thresholds():
    xy = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.5]])
    grid, origin = p.points_to_occupancy(xy, resolution=0.5, min_pts=2)
    assert grid[0, 0] == 100
    assert grid[1, 0] == 0


def test_points_to_occupancy_raises_on_empty():
    with pytest.raises(ValueError):
        p.points_to_occupancy(np.zeros((0, 2)), resolution=0.5, min_pts=1)


def test_occupancy_to_pgm_flips_rows_and_maps_values():
    grid = np.array([[100], [0]], dtype=np.uint8)
    data = p.occupancy_to_pgm_bytes(grid)
    header = b"P5\n1 2\n255\n"
    assert data.startswith(header)
    body = data[len(header):]
    assert body == bytes([254, 0])  # 第一行=最大y=空闲(254)，第二行=占据(0)


def test_make_map_yaml_has_origin_and_thresholds():
    y = p.make_map_yaml("map.pgm", resolution=0.05, origin_xy=np.array([1.0, 2.0]))
    assert "image: map.pgm" in y
    assert "resolution: 0.05" in y
    assert "origin: [1.0, 2.0, 0.0]" in y
    assert "occupied_thresh: 0.65" in y
    assert "free_thresh: 0.25" in y
