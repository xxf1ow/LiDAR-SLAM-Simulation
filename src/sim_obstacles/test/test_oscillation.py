"""velocity_at 时间开环往复逻辑（spec §3：前半周期正向、后半周期反向）。"""
from sim_obstacles.oscillation import velocity_at


def test_first_half_period_forward():
    assert velocity_at(t=0.0, speed=0.5, period=16.0) == 0.5
    assert velocity_at(t=7.9, speed=0.5, period=16.0) == 0.5


def test_second_half_period_backward():
    assert velocity_at(t=8.0, speed=0.5, period=16.0) == -0.5
    assert velocity_at(t=15.9, speed=0.5, period=16.0) == -0.5


def test_wraps_after_full_period():
    assert velocity_at(t=16.0, speed=0.5, period=16.0) == 0.5
    assert velocity_at(t=40.0, speed=0.3, period=16.0) == -0.3  # 40 % 16 = 8 → 后半


def test_phase_offset_shifts_reversal():
    # phase=8 把翻转点提前半周期：t=0 时已处于后半周期
    assert velocity_at(t=0.0, speed=0.5, period=16.0, phase=8.0) == -0.5
