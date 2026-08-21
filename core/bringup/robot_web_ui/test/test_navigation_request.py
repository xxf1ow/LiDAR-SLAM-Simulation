import importlib
import math

import pytest

from robot_web_ui.map_snapshot import GridInfo


def _navigation_request():
    return importlib.import_module("robot_web_ui.navigation_request")


def _world_point(info, local_x, local_y):
    cosine = math.cos(info.origin_yaw)
    sine = math.sin(info.origin_yaw)
    return (
        info.origin_x + local_x * cosine - local_y * sine,
        info.origin_y + local_x * sine + local_y * cosine,
    )


def _payload(info, local_x=0.5, local_y=0.5, yaw=0.0, revision=1):
    x, y = _world_point(info, local_x, local_y)
    return {"x": x, "y": y, "yaw": yaw, "map_revision": revision}


def test_exact_payload_is_normalized_and_preserved():
    navigation_request = _navigation_request()
    info = GridInfo(4, 3, 0.5, 10.0, -5.0, 0.0, "map")
    payload = {"x": 10.5, "y": -4.25, "yaw": 5 * math.pi / 2, "map_revision": 7}

    pose = navigation_request.parse_navigation_pose(payload, info, 7)

    assert pose.x == payload["x"]
    assert pose.y == payload["y"]
    assert pose.map_revision == payload["map_revision"]
    assert -math.pi <= pose.yaw <= math.pi
    assert pose.yaw == pytest.approx(math.pi / 2)


def test_yaw_quaternion_matches_normalized_pose():
    navigation_request = _navigation_request()
    yaw = 5 * math.pi / 2

    z, w = navigation_request.yaw_quaternion(yaw)

    normalized = math.atan2(math.sin(yaw), math.cos(yaw))
    assert z == pytest.approx(math.sin(normalized / 2.0))
    assert w == pytest.approx(math.cos(normalized / 2.0))
    assert math.hypot(z, w) == pytest.approx(1.0)


def test_rotated_map_bounds_accept_inside_and_reject_each_edge():
    navigation_request = _navigation_request()
    info = GridInfo(2, 3, 0.5, 10.0, -5.0, math.pi / 4, "map")
    width, height = info.width * info.resolution, info.height * info.resolution

    for local_x, local_y in ((0.0, 0.0), (width - 1e-12, height - 1e-12)):
        pose = navigation_request.parse_navigation_pose(
            _payload(info, local_x, local_y), info, 1
        )
        assert pose.x == pytest.approx(_world_point(info, local_x, local_y)[0])

    for local_x, local_y in (
        (-1e-9, height / 2),
        (width + 1e-9, height / 2),
        (width / 2, -1e-9),
        (width / 2, height + 1e-9),
    ):
        with pytest.raises(ValueError, match="outside"):
            navigation_request.parse_navigation_pose(
                _payload(info, local_x, local_y), info, 1
            )


def test_rotated_map_bounds_reject_exact_upper_boundaries():
    navigation_request = _navigation_request()
    info = GridInfo(2, 3, 0.5, 10.0, -5.0, math.pi / 2, "map")
    width, height = info.width * info.resolution, info.height * info.resolution

    for local_x, local_y in ((width, height / 2), (width / 2, height)):
        with pytest.raises(ValueError, match="outside"):
            navigation_request.parse_navigation_pose(
                _payload(info, local_x, local_y), info, 1
            )


def test_revision_must_be_current_positive_non_boolean_integer():
    navigation_request = _navigation_request()
    info = GridInfo(2, 2, 1.0, 0.0, 0.0, 0.0, "map")

    stale = _payload(info, revision=2)
    with pytest.raises(navigation_request.MapRevisionConflict, match="changed"):
        navigation_request.parse_navigation_pose(stale, info, 1)

    for revision in (0, -1, 1.0, "1", True):
        payload = _payload(info, revision=revision)
        with pytest.raises(ValueError, match="positive integer"):
            navigation_request.parse_navigation_pose(payload, info, 1)


def test_pose_numbers_are_finite_native_numbers_not_bool():
    navigation_request = _navigation_request()
    info = GridInfo(2, 2, 1.0, 0.0, 0.0, 0.0, "map")

    for field, value in (
        ("x", math.nan),
        ("x", math.inf),
        ("y", -math.inf),
        ("yaw", "0"),
        ("yaw", True),
    ):
        payload = _payload(info)
        payload[field] = value
        with pytest.raises(ValueError, match=field):
            navigation_request.parse_navigation_pose(payload, info, 1)


def test_pose_payload_rejects_missing_and_extra_keys():
    navigation_request = _navigation_request()
    info = GridInfo(2, 2, 1.0, 0.0, 0.0, 0.0, "map")
    payload = _payload(info)

    for malformed in (
        {key: value for key, value in payload.items() if key != "yaw"},
        {**payload, "extra": 0},
    ):
        with pytest.raises(ValueError, match="pose request keys"):
            navigation_request.parse_navigation_pose(malformed, info, 1)


def test_zero_sized_or_nonfinite_map_geometry_is_rejected():
    navigation_request = _navigation_request()
    valid = GridInfo(2, 2, 1.0, 0.0, 0.0, 0.0, "map")
    malformed_infos = (
        GridInfo(0, 2, 1.0, 0.0, 0.0, 0.0, "map"),
        GridInfo(2, 0, 1.0, 0.0, 0.0, 0.0, "map"),
        GridInfo(2, 2, math.nan, 0.0, 0.0, 0.0, "map"),
        GridInfo(2, 2, 1.0, math.inf, 0.0, 0.0, "map"),
        GridInfo(2, 2, 1.0, 0.0, -math.inf, 0.0, "map"),
        GridInfo(2, 2, 1.0, 0.0, 0.0, math.nan, "map"),
        GridInfo(True, 2, 1.0, 0.0, 0.0, 0.0, "map"),
    )

    for info in malformed_infos:
        with pytest.raises(ValueError):
            navigation_request.parse_navigation_pose(_payload(valid), info, 1)
