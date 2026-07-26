import pytest

from cmd_vel_gate.gate_logic import GateState, Mode


def test_starts_automatic_without_a_fresh_command():
    state = GateState()

    assert state.mode is Mode.AUTOMATIC
    assert state.selected_source_is_stale(now=0.0, timeout=0.5)


def test_automatic_accepts_only_automatic():
    state = GateState()

    assert not state.accept(Mode.MANUAL, received_at=1.0)
    assert state.selected_source_is_stale(now=1.0, timeout=0.5)
    assert state.accept(Mode.AUTOMATIC, received_at=2.0)
    assert not state.selected_source_is_stale(now=2.49, timeout=0.5)


def test_switch_exposes_stopped_then_clears_freshness():
    state = GateState()
    assert state.accept(Mode.AUTOMATIC, received_at=1.0)

    state.stop()
    assert state.mode is Mode.STOPPED
    assert state.selected_source_is_stale(now=1.0, timeout=0.5)

    state.select(Mode.MANUAL)
    assert state.mode is Mode.MANUAL
    assert state.selected_source_is_stale(now=1.0, timeout=0.5)


def test_timeout_does_not_change_mode():
    state = GateState()
    assert state.accept(Mode.AUTOMATIC, received_at=10.0)

    assert state.selected_source_is_stale(now=10.5, timeout=0.5)
    assert state.mode is Mode.AUTOMATIC


def test_select_requires_stopped_and_a_routable_target():
    state = GateState()

    with pytest.raises(ValueError):
        state.select(Mode.MANUAL)

    state.stop()
    with pytest.raises(ValueError):
        state.select(Mode.STOPPED)

    state.select(Mode.MANUAL)
    assert state.mode is Mode.MANUAL

    with pytest.raises(ValueError):
        state.select(Mode.AUTOMATIC)
