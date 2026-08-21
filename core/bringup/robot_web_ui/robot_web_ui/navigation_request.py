"""Validation helpers for navigation poses submitted by the web UI."""

from dataclasses import dataclass
import math

from .map_snapshot import GridInfo


class MapRevisionConflict(ValueError):
    pass


@dataclass(frozen=True)
class NavigationPose:
    x: float
    y: float
    yaw: float
    map_revision: int


def _finite_number(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def parse_navigation_pose(
    payload: dict[str, object],
    map_info: GridInfo,
    current_revision: int,
) -> NavigationPose:
    if type(payload) is not dict or set(payload) != {
        "x", "y", "yaw", "map_revision"
    }:
        raise ValueError("pose request keys must be x, y, yaw, map_revision")

    revision = payload["map_revision"]
    if type(revision) is not int or revision <= 0:
        raise ValueError("map_revision must be a positive integer")
    if revision != current_revision:
        raise MapRevisionConflict("map revision changed")

    if (
        type(map_info.width) is not int
        or type(map_info.height) is not int
        or map_info.width <= 0
        or map_info.height <= 0
    ):
        raise ValueError("invalid map dimensions")
    resolution = _finite_number(map_info.resolution, "map resolution")
    if resolution <= 0:
        raise ValueError("invalid map resolution")
    origin_x = _finite_number(map_info.origin_x, "map origin x")
    origin_y = _finite_number(map_info.origin_y, "map origin y")
    origin_yaw = _finite_number(map_info.origin_yaw, "map origin yaw")

    x = _finite_number(payload["x"], "x")
    y = _finite_number(payload["y"], "y")
    payload_yaw = _finite_number(payload["yaw"], "yaw")
    yaw = math.atan2(math.sin(payload_yaw), math.cos(payload_yaw))

    dx, dy = x - origin_x, y - origin_y
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    local_x = dx * cosine + dy * sine
    local_y = -dx * sine + dy * cosine
    if not (
        0.0 <= local_x < map_info.width * resolution
        and 0.0 <= local_y < map_info.height * resolution
    ):
        raise ValueError("pose is outside the static map")
    return NavigationPose(x, y, yaw, revision)


def yaw_quaternion(yaw: float) -> tuple[float, float]:
    normalized = math.atan2(math.sin(yaw), math.cos(yaw))
    return math.sin(normalized / 2.0), math.cos(normalized / 2.0)
