from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"


def _load(name):
    return yaml.safe_load((CONFIG / name).read_text(encoding="utf-8"))


def _strip_use_sim_time(value):
    if isinstance(value, dict):
        return {
            key: _strip_use_sim_time(item)
            for key, item in value.items()
            if key != "use_sim_time"
        }
    if isinstance(value, list):
        return [_strip_use_sim_time(item) for item in value]
    return value


def _use_sim_time_values(value):
    found = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key == "use_sim_time":
                found.append(item)
            found.extend(_use_sim_time_values(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(_use_sim_time_values(item))
    return found


def test_real_nav2_changes_only_clock_domain():
    sim = _load("nav2_params.yaml")
    real = _load("nav2_params_real.yaml")
    assert _strip_use_sim_time(real) == _strip_use_sim_time(sim)
    assert _use_sim_time_values(sim)
    assert set(_use_sim_time_values(sim)) == {True}
    assert set(_use_sim_time_values(real)) == {False}
