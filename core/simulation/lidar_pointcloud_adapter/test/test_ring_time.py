import numpy as np
from lidar_pointcloud_adapter.ring_time import compute_ring_time


def test_first_point_row0_time0():
    ring, t = compute_ring_time(0, width=1800, scan_period=0.1)
    assert ring == 0
    assert t == 0.0


def test_end_of_row0_time_near_period():
    ring, t = compute_ring_time(1799, width=1800, scan_period=0.1)
    assert ring == 0
    assert abs(t - 0.1 * 1799 / 1800) < 1e-9


def test_start_of_row1_resets_time():
    ring, t = compute_ring_time(1800, width=1800, scan_period=0.1)
    assert ring == 1
    assert t == 0.0


def test_midrow_row1():
    ring, t = compute_ring_time(1800 + 900, width=1800, scan_period=0.1)
    assert ring == 1
    assert abs(t - 0.05) < 1e-9


def test_vectorized_matches_scalar():
    idx = np.array([0, 1799, 1800, 2700], dtype=np.int64)
    ring, t = compute_ring_time(idx, width=1800, scan_period=0.1)
    assert ring.tolist() == [0, 0, 1, 1]
    np.testing.assert_allclose(t, [0.0, 0.1 * 1799 / 1800, 0.0, 0.05])
