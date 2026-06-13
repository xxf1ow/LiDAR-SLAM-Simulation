"""障碍清单加载与校验（spec §3 错误处理：fail fast）。"""
import os

import pytest

from sim_obstacles.config_loader import load_obstacles

DEFAULT = os.path.join(os.path.dirname(__file__), "..", "config", "obstacles.yaml")


def _write(tmp_path, text):
    p = tmp_path / "obstacles.yaml"
    p.write_text(text, encoding="utf-8")
    return str(p)


def test_default_config_loads_and_is_dense():
    obs = load_obstacles(DEFAULT)
    assert len(obs) >= 8                      # spec：密度要够
    assert all(o["phase"] >= 0.0 for o in obs)  # phase 默认补 0


def test_rejects_nonpositive_speed(tmp_path):
    path = _write(tmp_path, """\
obstacles:
  - {name: a, x: 0.0, y: 0.0, yaw: 0.0, speed: 0.0, period: 10.0}
""")
    with pytest.raises(ValueError, match="speed"):
        load_obstacles(path)


def test_rejects_nonpositive_period(tmp_path):
    path = _write(tmp_path, """\
obstacles:
  - {name: a, x: 0.0, y: 0.0, yaw: 0.0, speed: 0.5, period: -1.0}
""")
    with pytest.raises(ValueError, match="period"):
        load_obstacles(path)


def test_rejects_duplicate_names(tmp_path):
    path = _write(tmp_path, """\
obstacles:
  - {name: a, x: 0.0, y: 0.0, yaw: 0.0, speed: 0.5, period: 10.0}
  - {name: a, x: 1.0, y: 0.0, yaw: 0.0, speed: 0.5, period: 10.0}
""")
    with pytest.raises(ValueError, match="重复"):
        load_obstacles(path)


def test_rejects_missing_field(tmp_path):
    path = _write(tmp_path, """\
obstacles:
  - {name: a, x: 0.0, y: 0.0, speed: 0.5, period: 10.0}
""")
    with pytest.raises(ValueError, match="yaw"):
        load_obstacles(path)


def test_rejects_empty_list(tmp_path):
    path = _write(tmp_path, "obstacles: []\n")
    with pytest.raises(ValueError, match="obstacles"):
        load_obstacles(path)
