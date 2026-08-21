"""Pure Nav2 map parsing and immutable binary snapshots."""

from __future__ import annotations

import gzip
import hashlib
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml


@dataclass(frozen=True)
class GridInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str

    def as_dict(self) -> dict[str, object]:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, self.origin_yaw],
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class BinarySnapshot:
    revision: int
    etag: str
    media_type: str
    data: bytes
    gzip_data: bytes


@dataclass(frozen=True)
class GridSnapshot:
    info: GridInfo
    binary: BinarySnapshot


@dataclass(frozen=True)
class PathSnapshot:
    frame_id: str
    binary: BinarySnapshot


_MEDIA_TYPE = "application/octet-stream"


def _canonical_metadata(metadata: dict[str, object]) -> bytes:
    return json.dumps(
        metadata, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _make_binary(
    revision: int,
    media_type: str,
    metadata: dict[str, object],
    data: bytes,
) -> BinarySnapshot:
    body = bytes(data)
    digest = hashlib.sha256(
        media_type.encode("utf-8") + _canonical_metadata(metadata) + body
    ).hexdigest()
    return BinarySnapshot(
        revision=revision,
        etag=f'"{digest}"',
        media_type=media_type,
        data=body,
        gzip_data=gzip.compress(body, mtime=0),
    )


def _number(root: dict[str, object], field: str) -> float:
    value = root.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"invalid {field}: expected a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"invalid {field}: expected a finite number")
    return value


def _pgm_token(raw: bytes, index: int) -> tuple[bytes, int]:
    length = len(raw)
    while index < length:
        byte = raw[index]
        if byte in b" \t\r\n\f\v":
            index += 1
            continue
        if byte == ord("#"):
            newline = raw.find(b"\n", index)
            index = length if newline < 0 else newline + 1
            continue
        break
    start = index
    while index < length and raw[index] not in b" \t\r\n\f\v#":
        index += 1
    if start == index:
        raise ValueError("invalid PGM header: missing token")
    return raw[start:index], index


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ValueError(f"invalid image: cannot read {path}") from exc

    index = 0
    try:
        magic, index = _pgm_token(raw, index)
        width_token, index = _pgm_token(raw, index)
        height_token, index = _pgm_token(raw, index)
        maxval_token, index = _pgm_token(raw, index)
    except ValueError as exc:
        raise ValueError(f"invalid PGM header: {exc}") from exc

    if magic != b"P5":
        raise ValueError("invalid PGM magic: expected P5")
    try:
        width = int(width_token)
        height = int(height_token)
        maxval = int(maxval_token)
    except ValueError as exc:
        raise ValueError("invalid PGM header dimensions or max value") from exc
    if width <= 0 or height <= 0:
        raise ValueError("invalid PGM dimensions")
    if maxval != 255:
        raise ValueError("invalid PGM max value: expected 255")

    if index >= len(raw) or raw[index] not in b" \t\r\n\f\v":
        raise ValueError("invalid PGM header: missing raster separator")
    if raw[index] == ord("\r") and index + 1 < len(raw) and raw[index + 1] == ord("\n"):
        index += 2
    else:
        index += 1

    pixels = raw[index:]
    expected = width * height
    if len(pixels) != expected:
        raise ValueError(
            f"invalid PGM pixel count: expected {expected}, got {len(pixels)}"
        )
    return width, height, pixels


def load_nav2_pgm(yaml_path: Path) -> GridSnapshot:
    path = Path(yaml_path).expanduser()
    try:
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"invalid YAML: {exc}") from exc
    if not isinstance(root, dict):
        raise ValueError("invalid YAML root: expected a mapping")

    image = root.get("image")
    if not isinstance(image, str) or not image:
        raise ValueError("invalid image: expected a non-empty path")
    resolution = _number(root, "resolution")
    if resolution <= 0:
        raise ValueError("invalid resolution: expected a positive number")
    origin = root.get("origin")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in origin)
    ):
        raise ValueError("invalid origin: expected three numbers")
    origin_values = [float(item) for item in origin]
    if any(not math.isfinite(item) for item in origin_values):
        raise ValueError("invalid origin: expected finite numbers")

    negate = root.get("negate")
    if isinstance(negate, bool) or not isinstance(negate, int) or negate not in (0, 1):
        raise ValueError("invalid negate: expected 0 or 1")
    occupied_thresh = _number(root, "occupied_thresh")
    free_thresh = _number(root, "free_thresh")
    if not 0.0 <= free_thresh <= 1.0:
        raise ValueError("invalid free_thresh: expected a value from 0 to 1")
    if not 0.0 <= occupied_thresh <= 1.0:
        raise ValueError("invalid occupied_thresh: expected a value from 0 to 1")
    if free_thresh >= occupied_thresh:
        raise ValueError("invalid thresholds: free_thresh must be below occupied_thresh")
    if root.get("mode") != "trinary":
        raise ValueError("invalid mode: only trinary is supported")

    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    width, height, pixels = _read_pgm(image_path)

    rows = []
    for row in range(height - 1, -1, -1):
        row_start = row * width
        converted = bytearray()
        for pixel in pixels[row_start : row_start + width]:
            occupancy = pixel / 255.0 if negate else (255 - pixel) / 255.0
            if occupancy > occupied_thresh:
                converted.append(100)
            elif occupancy < free_thresh:
                converted.append(0)
            else:
                converted.append(255)
        rows.append(bytes(converted))
    data = b"".join(rows)
    info = GridInfo(width, height, resolution, *origin_values, "map")
    return update_grid_snapshot(None, info, data)


def update_grid_snapshot(
    current: GridSnapshot | None, info: GridInfo, data: bytes
) -> GridSnapshot:
    body = bytes(data)
    expected = info.width * info.height
    if len(body) != expected:
        raise ValueError(f"invalid grid data length: expected {expected}, got {len(body)}")
    if current is not None and current.info == info and current.binary.data == body:
        return current
    revision = 1 if current is None else current.binary.revision + 1
    return GridSnapshot(
        info,
        _make_binary(revision, _MEDIA_TYPE, info.as_dict(), body),
    )


def update_path_snapshot(
    current: PathSnapshot | None,
    frame_id: str,
    points: Iterable[tuple[float, float]],
) -> PathSnapshot:
    body = b"".join(struct.pack("<ff", float(x), float(y)) for x, y in points)
    if current is not None and current.frame_id == frame_id and current.binary.data == body:
        return current
    revision = 1 if current is None else current.binary.revision + 1
    return PathSnapshot(
        frame_id,
        _make_binary(revision, _MEDIA_TYPE, {"frame_id": frame_id}, body),
    )
