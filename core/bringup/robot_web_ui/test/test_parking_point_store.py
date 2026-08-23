import json
import math
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

import robot_web_ui.parking_point_store as parking_point_store
from robot_web_ui.parking_point_store import (
    ParkingPoint,
    ParkingPointCorruptError,
    ParkingPointDuplicateError,
    ParkingPointNotFoundError,
    ParkingPointStorageError,
    ParkingPointValidationError,
    ParkingPointStore,
    normalize_name,
    parking_points_path,
)


def test_parking_point_store_persists_exact_order_and_preserves_state_on_failures(
    tmp_path, monkeypatch
):
    map_path = tmp_path / "maps" / "warehouse.yaml"
    map_path.parent.mkdir()
    target = parking_points_path(map_path)
    assert target == map_path.with_name("warehouse.parking_points.json")

    store = ParkingPointStore(map_path)
    assert store.list() == ()

    first = ParkingPoint("  Dock 1  ", 1, 2.5, -math.pi)
    second = ParkingPoint("Dock 2", -3.0, 4, 0.25)
    store.save(first)
    store.save(second)
    assert target.read_text(encoding="utf-8") == json.dumps(
        {
            "version": 1,
            "points": [
                {"name": "Dock 1", "x": 1.0, "y": 2.5, "yaw": -math.pi},
                {"name": "Dock 2", "x": -3.0, "y": 4.0, "yaw": 0.25},
            ],
        },
        indent=2,
    ) + "\n"
    assert [point.as_dict() for point in store.list()] == [
        {"name": "Dock 1", "x": 1.0, "y": 2.5, "yaw": -math.pi},
        {"name": "Dock 2", "x": -3.0, "y": 4.0, "yaw": 0.25},
    ]
    listed = store.list()
    assert isinstance(listed, tuple)
    with pytest.raises(AttributeError):
        listed.append(first)
    with pytest.raises(FrozenInstanceError):
        listed[0].x = 99

    assert store.get(" Dock 1 ") == ParkingPoint("Dock 1", 1.0, 2.5, -math.pi)
    with pytest.raises(ParkingPointNotFoundError):
        store.get("missing")
    store.delete(" Dock 1 ")
    assert [point.name for point in store.list()] == ["Dock 2"]
    with pytest.raises(ParkingPointNotFoundError):
        store.delete("Dock 1")

    assert normalize_name("  A name  ") == "A name"
    with pytest.raises(ParkingPointValidationError):
        normalize_name(42)
    for invalid_name in ("", " " * 41, 42):
        with pytest.raises(ParkingPointValidationError):
            store.save(ParkingPoint(invalid_name, 0, 0, 0))
    for invalid_coordinate in (math.nan, math.inf, -math.inf):
        with pytest.raises(ParkingPointValidationError):
            store.save(ParkingPoint("bad", invalid_coordinate, 0, 0))
    store.save(ParkingPoint("Unique", 0, 0, 0))
    with pytest.raises(ParkingPointDuplicateError):
        store.save(ParkingPoint(" Unique ", 1, 1, 1))

    target_bytes = target.read_bytes()
    target.write_bytes(b"not json")
    with pytest.raises(ParkingPointCorruptError):
        ParkingPointStore(map_path)
    assert target.read_bytes() == b"not json"
    target.write_bytes(target_bytes)

    original_read_text = Path.read_text
    monkeypatch.setattr(
        Path,
        "read_text",
        lambda self, *args, **kwargs: (
            (_ for _ in ()).throw(OSError("injected read failure"))
            if self == target
            else original_read_text(self, *args, **kwargs)
        ),
    )
    with pytest.raises(ParkingPointStorageError) as read_error:
        ParkingPointStore(map_path)
    assert not isinstance(read_error.value, ParkingPointCorruptError)
    assert target.read_bytes() == target_bytes
    monkeypatch.undo()

    store = ParkingPointStore(map_path)
    original_points = store.list()
    original_bytes = target.read_bytes()
    before_overflow_paths = set(target.parent.iterdir())
    with pytest.raises(ParkingPointValidationError):
        store.save(ParkingPoint("overflow", 10**1000, 0, 0))
    assert store.list() == original_points
    assert target.read_bytes() == original_bytes
    assert set(target.parent.iterdir()) == before_overflow_paths

    def assert_write_failure(failure, expected_type=ParkingPointStorageError):
        before_paths = set(target.parent.iterdir())
        failure(monkeypatch)
        with pytest.raises(expected_type):
            store.save(ParkingPoint("replacement", 9, 9, 9))
        assert store.list() == original_points
        assert target.read_bytes() == original_bytes
        assert set(target.parent.iterdir()) == before_paths
        monkeypatch.undo()

    def fail_temp_creation(patch):
        patch.setattr(
            parking_point_store,
            "NamedTemporaryFile",
            lambda *args, **kwargs: (_ for _ in ()).throw(OSError("temp")),
        )

    def fail_json_write(patch):
        patch.setattr(
            parking_point_store.json,
            "dump",
            lambda *args, **kwargs: (_ for _ in ()).throw(OSError("write")),
        )

    class FailingFlush:
        def __init__(self, handle):
            self._handle = handle
            self.name = handle.name

        def __enter__(self):
            return self

        def __exit__(self, *args):
            self._handle.close()

        def write(self, value):
            return self._handle.write(value)

        def flush(self):
            raise OSError("flush")

        def close(self):
            self._handle.close()

    def fail_flush(patch):
        original = parking_point_store.NamedTemporaryFile

        def create_failing(*args, **kwargs):
            return FailingFlush(original(*args, **kwargs))

        patch.setattr(parking_point_store, "NamedTemporaryFile", create_failing)

    def fail_replace(patch):
        patch.setattr(
            parking_point_store.os,
            "replace",
            lambda *args, **kwargs: (_ for _ in ()).throw(OSError("replace")),
        )

    assert_write_failure(fail_temp_creation)
    assert_write_failure(fail_json_write)
    assert_write_failure(fail_flush)
    assert_write_failure(fail_replace)


@pytest.mark.parametrize(
    ("sidecar_bytes", "cause_type"),
    [
        (b"{\"version\": 1, \"points\": [\xff]}", UnicodeDecodeError),
        (
            json.dumps(
                {
                    "version": 1,
                    "points": [
                        {"name": "Huge", "x": 10**1000, "y": 0, "yaw": 0}
                    ],
                }
            ).encode("utf-8"),
            ParkingPointValidationError,
        ),
        (
            json.dumps({"version": 1.0, "points": []}).encode("utf-8"),
            ValueError,
        ),
    ],
    ids=["invalid-utf8", "coordinate-integer-overflow", "float-version"],
)
def test_malformed_sidecars_are_typed_and_preserve_bytes_and_name(
    tmp_path, sidecar_bytes, cause_type
):
    map_path = tmp_path / "warehouse.yaml"
    sidecar = parking_points_path(map_path)
    sidecar.write_bytes(sidecar_bytes)

    with pytest.raises(ParkingPointCorruptError) as error:
        ParkingPointStore(map_path)

    assert isinstance(error.value.__cause__, cause_type)
    assert sidecar.read_bytes() == sidecar_bytes
    assert sidecar.name in str(error.value)
