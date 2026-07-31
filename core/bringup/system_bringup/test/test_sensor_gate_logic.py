import pytest

from system_bringup.sensor_gate_logic import (
    IMU_FRAME,
    POINT_FIELDS,
    POINT_FRAME,
    SensorGateState,
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
                height=32,
                width=1200,
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


def test_healthy_streams_become_ready_after_two_stable_seconds():
    state = SensorGateState()
    feed_healthy(state, duration=2.3)
    ready, reason = state.status(2.3)
    assert ready, reason


def test_wrong_point_shape_never_becomes_ready():
    state = SensorGateState()
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id="velodyne",
        height=16,
        width=1200,
        fields=POINT_FIELDS,
    )
    ready, reason = state.status(0.1)
    assert not ready
    assert "32x1200" in reason


def test_wrong_frames_are_reported():
    state = SensorGateState()
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id="wrong_lidar",
        height=32,
        width=1200,
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


def test_stale_headers_are_reported():
    state = SensorGateState()
    state.observe_imu(
        received=0.0,
        stamp=100.0,
        now_ros=100.6,
        frame_id=IMU_FRAME,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "0.5" in reason


def test_low_point_or_imu_rate_resets_stability():
    state = SensorGateState()
    feed_healthy(state, duration=2.3, point_hz=5.0, imu_hz=100.0)
    ready, reason = state.status(2.3)
    assert not ready
    assert "8" in reason
    assert "150" in reason


def test_stream_stopping_drops_observed_rate():
    state = SensorGateState()
    feed_healthy(state, duration=2.3)
    assert state.status(2.3)[0]
    ready, reason = state.status(4.6)
    assert not ready
    assert "Hz" in reason


def test_dense_burst_followed_by_silence_never_becomes_ready():
    state = SensorGateState()
    feed_healthy(state, duration=0.1, point_hz=200.0, imu_hz=2000.0)
    ready, reason = state.status(2.01)
    assert not ready
    assert "Hz" in reason


def test_identical_received_timestamps_are_unhealthy_without_crashing():
    state = SensorGateState()
    for _ in range(2):
        state.observe_point(
            received=1.0,
            stamp=101.0,
            now_ros=101.1,
            frame_id=POINT_FRAME,
            height=32,
            width=1200,
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


def test_invalid_contract_resets_stability_before_a_fresh_two_seconds():
    state = SensorGateState()
    feed_healthy(state, duration=1.0)
    state.observe_point(
        received=1.05,
        stamp=101.05,
        now_ros=101.15,
        frame_id="wrong_lidar",
        height=32,
        width=1200,
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
def test_wrong_or_reordered_point_fields_are_reported(fields):
    state = SensorGateState()
    state.observe_point(
        received=0.0,
        stamp=100.0,
        now_ros=100.1,
        frame_id=POINT_FRAME,
        height=32,
        width=1200,
        fields=fields,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "fields" in reason


@pytest.mark.parametrize(
    "stamp, now_ros",
    [(100.0, 100.6), (100.1, 100.0)],
)
def test_stale_or_future_point_headers_are_reported(stamp, now_ros):
    state = SensorGateState()
    state.observe_point(
        received=0.0,
        stamp=stamp,
        now_ros=now_ros,
        frame_id=POINT_FRAME,
        height=32,
        width=1200,
        fields=POINT_FIELDS,
    )
    ready, reason = state.status(0.0)
    assert not ready
    assert "header age" in reason
