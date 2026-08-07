import pytest

from system_bringup.sensor_gate_logic import (
    IMU_FRAME,
    POINT_FIELDS,
    POINT_FRAME,
    SensorGateState,
)


@pytest.fixture
def state():
    return SensorGateState(
        expected_points_per_scan=16 * 1800,
        minimum_point_hz=8.0,
        minimum_imu_hz=150.0,
        max_stamp_age=0.5,
        rate_window=2.0,
        stable_duration=2.0,
    )


def feed_healthy(state, duration, point_hz=10.0, imu_hz=200.0, start=0.0):
    events = [
        (start + index / point_hz, "point")
        for index in range(int(duration * point_hz) + 1)
    ]
    events += [
        (start + index / imu_hz, "imu")
        for index in range(int(duration * imu_hz) + 1)
    ]
    for received, kind in sorted(events):
        if kind == "point":
            state.observe_point(
                received=received,
                stamp=100.0 + received,
                now_ros=100.1 + received,
                frame_id=POINT_FRAME,
                height=16,
                width=1800,
                fields=POINT_FIELDS,
            )
        else:
            state.observe_imu(
                received=received,
                stamp=100.0 + received,
                now_ros=100.001 + received,
                frame_id=IMU_FRAME,
            )
        state.status(received)


def test_healthy_streams_become_ready_after_two_stable_seconds(state):
    feed_healthy(state, duration=2.3)
    ready, reason = state.status(2.3)
    assert ready, reason


@pytest.mark.parametrize("height, width", [(1, 28800), (16, 1800)])
def test_expected_total_points_accepts_organized_or_unorganized_clouds(state, height, width):
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id=POINT_FRAME,
        height=height,
        width=width,
        fields=POINT_FIELDS,
    )
    assert state.point_problem is None


def test_wrong_total_points_never_becomes_ready_without_fixed_shape_assumption(state):
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id="velodyne",
        height=16,
        width=1799,
        fields=POINT_FIELDS,
    )
    ready, reason = state.status(0.1)
    assert not ready
    assert "28800" in reason
    assert "32x1200" not in reason


def test_wrong_frames_are_reported(state):
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id="wrong_lidar",
        height=16,
        width=1800,
        fields=POINT_FIELDS,
    )
    state.observe_imu(
        received=0.0,
        stamp=100.0,
        now_ros=100.001,
        frame_id="wrong_imu",
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "wrong_lidar" in reason
    assert "wrong_imu" in reason


def test_stale_headers_are_reported(state):
    state.observe_imu(
        received=0.0,
        stamp=100.0,
        now_ros=100.6,
        frame_id=IMU_FRAME,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "0.5" in reason


def test_low_point_or_imu_rate_resets_stability(state):
    feed_healthy(state, duration=2.3, point_hz=5.0, imu_hz=100.0)
    ready, reason = state.status(2.3)
    assert not ready
    assert "8" in reason
    assert "150" in reason


def test_stream_interruption_clears_ready_state(state):
    feed_healthy(state, duration=2.3)
    assert state.status(2.3)[0]
    ready, reason = state.status(4.6)
    assert not ready
    assert "Hz" in reason


def test_dense_burst_followed_by_silence_never_becomes_ready(state):
    feed_healthy(state, duration=0.1, point_hz=200.0, imu_hz=2000.0)
    ready, reason = state.status(2.01)
    assert not ready
    assert "Hz" in reason


def test_identical_received_timestamps_are_unhealthy_without_crashing(state):
    for _ in range(2):
        state.observe_point(
            received=1.0,
            stamp=101.0,
            now_ros=101.1,
            frame_id=POINT_FRAME,
            height=16,
            width=1800,
            fields=POINT_FIELDS,
        )
        state.observe_imu(
            received=1.0,
            stamp=101.0,
            now_ros=101.001,
            frame_id=IMU_FRAME,
        )
    ready, reason = state.status(1.0)
    assert not ready
    assert "Hz" in reason


def test_invalid_contract_resets_stability_before_a_fresh_two_seconds(state):
    feed_healthy(state, duration=1.0)
    state.observe_point(
        received=1.05,
        stamp=101.05,
        now_ros=101.15,
        frame_id="wrong_lidar",
        height=16,
        width=1800,
        fields=POINT_FIELDS,
    )
    assert not state.status(1.05)[0]

    feed_healthy(state, duration=1.9, start=1.1)
    assert not state.status(3.0)[0]
    feed_healthy(state, duration=0.2, start=3.1)
    assert state.status(3.3)[0]


@pytest.mark.parametrize(
    "fields",
    [
        ("x", "y", "z", "intensity", "ring"),
        ("y", "x", "z", "intensity", "ring", "time"),
    ],
)
def test_wrong_or_reordered_point_fields_are_reported(state, fields):
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id=POINT_FRAME,
        height=16,
        width=1800,
        fields=fields,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "fields" in reason


@pytest.mark.parametrize(
    "stamp, now_ros",
    [(100.0, 100.6), (100.1, 100.0)],
)
def test_stale_or_future_point_headers_are_reported(state, stamp, now_ros):
    state.observe_point(
        received=0.0,
        stamp=stamp,
        now_ros=now_ros,
        frame_id=POINT_FRAME,
        height=16,
        width=1800,
        fields=POINT_FIELDS,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "header age" in reason


def test_custom_rate_window_stable_duration_and_stamp_age_change_results():
    state = SensorGateState(
        expected_points_per_scan=16 * 1800,
        minimum_point_hz=20.0,
        minimum_imu_hz=300.0,
        max_stamp_age=0.1,
        rate_window=0.5,
        stable_duration=0.1,
    )
    feed_healthy(state, duration=0.6, point_hz=10.0, imu_hz=200.0)
    ready, reason = state.status(0.6)
    assert not ready
    assert "20" in reason
    assert "300" in reason

    state.observe_point(
        received=0.7,
        stamp=100.5,
        now_ros=100.7,
        frame_id=POINT_FRAME,
        height=16,
        width=1800,
        fields=POINT_FIELDS,
    )
    assert "outside [0, 0.1]s" in state.point_problem


def test_custom_rate_window_and_stable_duration_change_ready_timing():
    state = SensorGateState(
        expected_points_per_scan=16 * 1800,
        minimum_point_hz=8.0,
        minimum_imu_hz=150.0,
        max_stamp_age=10.0,
        rate_window=0.5,
        stable_duration=0.1,
    )
    feed_healthy(state, duration=0.2, point_hz=20.0, imu_hz=400.0)
    assert state.status(0.2)[0]

    ready, reason = state.status(0.71)
    assert not ready
    assert "Hz" in reason


def test_nominal_ros_time_rates_pass_even_when_wall_time_is_slow(state):
    feed_healthy(state, duration=2.3, start=1000.0)
    ready, reason = state.status(1002.3)
    assert ready, reason
