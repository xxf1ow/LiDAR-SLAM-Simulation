import numpy as np
from lidar_pointcloud_adapter.ring_time import compute_ring_time


SYNTHETIC_SCAN_WIDTH = 317
SYNTHETIC_SCAN_PERIOD = 0.073
SYNTHETIC_MID_COLUMN = 113


def test_first_point_row0_time0():
    ring, t = compute_ring_time(
        0, width=SYNTHETIC_SCAN_WIDTH, scan_period=SYNTHETIC_SCAN_PERIOD
    )
    assert ring == 0
    assert t == 0.0


def test_end_of_row0_time_near_period():
    index = SYNTHETIC_SCAN_WIDTH - 1
    ring, t = compute_ring_time(
        index, width=SYNTHETIC_SCAN_WIDTH, scan_period=SYNTHETIC_SCAN_PERIOD
    )
    assert ring == 0
    assert abs(
        t - SYNTHETIC_SCAN_PERIOD * index / SYNTHETIC_SCAN_WIDTH
    ) < 1e-9


def test_start_of_row1_resets_time():
    ring, t = compute_ring_time(
        SYNTHETIC_SCAN_WIDTH,
        width=SYNTHETIC_SCAN_WIDTH,
        scan_period=SYNTHETIC_SCAN_PERIOD,
    )
    assert ring == 1
    assert t == 0.0


def test_midrow_row1():
    ring, t = compute_ring_time(
        SYNTHETIC_SCAN_WIDTH + SYNTHETIC_MID_COLUMN,
        width=SYNTHETIC_SCAN_WIDTH,
        scan_period=SYNTHETIC_SCAN_PERIOD,
    )
    assert ring == 1
    assert abs(
        t
        - SYNTHETIC_SCAN_PERIOD
        * SYNTHETIC_MID_COLUMN
        / SYNTHETIC_SCAN_WIDTH
    ) < 1e-9


def test_vectorized_matches_scalar():
    last_column = SYNTHETIC_SCAN_WIDTH - 1
    idx = np.array(
        [
            0,
            last_column,
            SYNTHETIC_SCAN_WIDTH,
            SYNTHETIC_SCAN_WIDTH + SYNTHETIC_MID_COLUMN,
        ],
        dtype=np.int64,
    )
    ring, t = compute_ring_time(
        idx, width=SYNTHETIC_SCAN_WIDTH, scan_period=SYNTHETIC_SCAN_PERIOD
    )
    assert ring.tolist() == [0, 0, 1, 1]
    np.testing.assert_allclose(
        t,
        [
            0.0,
            SYNTHETIC_SCAN_PERIOD * last_column / SYNTHETIC_SCAN_WIDTH,
            0.0,
            SYNTHETIC_SCAN_PERIOD
            * SYNTHETIC_MID_COLUMN
            / SYNTHETIC_SCAN_WIDTH,
        ],
    )
