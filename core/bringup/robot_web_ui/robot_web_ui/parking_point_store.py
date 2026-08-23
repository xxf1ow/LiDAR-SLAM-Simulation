"""Persistent, validated parking points for a map."""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile


class ParkingPointError(Exception):
    """Base class for parking-point errors."""


class ParkingPointStorageError(ParkingPointError):
    """The parking-point sidecar could not be read or written."""


class ParkingPointCorruptError(ParkingPointStorageError):
    """The parking-point sidecar has an invalid format or schema."""


class ParkingPointValidationError(ParkingPointError):
    """A parking point or name is invalid."""


class ParkingPointDuplicateError(ParkingPointValidationError):
    """A parking-point name is already present."""


class ParkingPointNotFoundError(ParkingPointError):
    """A requested parking-point name is absent."""


@dataclass(frozen=True)
class ParkingPoint:
    name: str
    x: float
    y: float
    yaw: float

    def as_dict(self) -> dict[str, object]:
        return {"name": self.name, "x": self.x, "y": self.y, "yaw": self.yaw}


def parking_points_path(path: Path | str) -> Path:
    """Return the parking-point sidecar path for a map path."""

    map_path = Path(path).expanduser()
    return map_path.with_name(f"{map_path.stem}.parking_points.json")


def normalize_name(value: object) -> str:
    if not isinstance(value, str):
        raise ParkingPointValidationError("name must be a string")
    name = value.strip()
    if not 1 <= len(name) <= 40:
        raise ParkingPointValidationError("name must contain 1 to 40 characters")
    return name


def _coordinate(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParkingPointValidationError("coordinates must be numbers")
    try:
        converted = float(value)
    except OverflowError as exc:
        raise ParkingPointValidationError("coordinates must be finite") from exc
    if not math.isfinite(converted):
        raise ParkingPointValidationError("coordinates must be finite")
    return converted


def _validated_point(point: object) -> ParkingPoint:
    if not isinstance(point, ParkingPoint):
        raise ParkingPointValidationError("point must be a ParkingPoint")
    return ParkingPoint(
        normalize_name(point.name),
        _coordinate(point.x),
        _coordinate(point.y),
        _coordinate(point.yaw),
    )


class ParkingPointStore:
    """Serial, lock-free persistence for ordered parking points."""

    def __init__(self, path: Path | str):
        self._path = parking_points_path(path)
        self._points: tuple[ParkingPoint, ...] = self._read_points()

    def list(self) -> tuple[ParkingPoint, ...]:
        return tuple(self._points)

    def save(self, point: ParkingPoint) -> None:
        normalized = _validated_point(point)
        if any(existing.name == normalized.name for existing in self._points):
            raise ParkingPointDuplicateError(normalized.name)
        points = (*self._points, normalized)
        self._write_points(points)
        self._points = points

    def get(self, name: object) -> ParkingPoint:
        normalized_name = normalize_name(name)
        for point in self._points:
            if point.name == normalized_name:
                return point
        raise ParkingPointNotFoundError(normalized_name)

    def delete(self, name: object) -> None:
        normalized_name = normalize_name(name)
        points = tuple(point for point in self._points if point.name != normalized_name)
        if len(points) == len(self._points):
            raise ParkingPointNotFoundError(normalized_name)
        self._write_points(points)
        self._points = points

    def _read_points(self) -> tuple[ParkingPoint, ...]:
        if not self._path.exists():
            return ()
        try:
            raw = self._path.read_text(encoding="utf-8")
        except UnicodeError as exc:
            raise ParkingPointCorruptError(
                f"invalid parking-point sidecar {self._path}"
            ) from exc
        except OSError as exc:
            raise ParkingPointStorageError(f"cannot read {self._path}") from exc
        try:
            root = json.loads(raw)
            if (
                not isinstance(root, dict)
                or set(root) != {"version", "points"}
                or type(root["version"]) is not int
                or root["version"] != 1
                or not isinstance(root["points"], list)
            ):
                raise ValueError("invalid root schema")
            points = []
            names = set()
            for item in root["points"]:
                if not isinstance(item, dict) or set(item) != {"name", "x", "y", "yaw"}:
                    raise ValueError("invalid item schema")
                name = item["name"]
                if not isinstance(name, str) or normalize_name(name) != name:
                    raise ValueError("invalid name")
                point = ParkingPoint(
                    name,
                    _coordinate(item["x"]),
                    _coordinate(item["y"]),
                    _coordinate(item["yaw"]),
                )
                if name in names:
                    raise ValueError("duplicate name")
                names.add(name)
                points.append(point)
            return tuple(points)
        except (
            json.JSONDecodeError,
            UnicodeDecodeError,
            OverflowError,
            TypeError,
            ValueError,
            ParkingPointError,
        ) as exc:
            raise ParkingPointCorruptError(f"invalid parking-point sidecar {self._path}") from exc

    def _write_points(self, points: tuple[ParkingPoint, ...]) -> None:
        target = self._path
        temporary = None
        temporary_path: Path | None = None
        try:
            temporary = NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=target.parent,
                delete=False,
            )
            temporary_path = Path(temporary.name)
            json.dump(
                {"version": 1, "points": [point.as_dict() for point in points]},
                temporary,
                indent=2,
            )
            temporary.write("\n")
            temporary.flush()
            temporary.close()
            temporary = None
            os.replace(temporary_path, target)
        except (OSError, TypeError, ValueError) as exc:
            if temporary is not None:
                try:
                    temporary.close()
                except OSError:
                    pass
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except OSError:
                    pass
            raise ParkingPointStorageError(f"cannot write {target}") from exc


__all__ = [
    "ParkingPoint",
    "ParkingPointStore",
    "ParkingPointError",
    "ParkingPointStorageError",
    "ParkingPointCorruptError",
    "ParkingPointValidationError",
    "ParkingPointDuplicateError",
    "ParkingPointNotFoundError",
    "normalize_name",
    "parking_points_path",
]
