"""时间开环往复速度：无位置闭环（spec §3 关键决策，漂移可接受）。"""


def velocity_at(t: float, speed: float, period: float, phase: float = 0.0) -> float:
    """返回 t 时刻沿运动轴的有符号速度。

    前半周期 +speed、后半周期 -speed；phase（秒）整体平移时间轴，
    用于错开多个障碍的翻转时刻。
    """
    tau = (t + phase) % period
    return speed if tau < period / 2.0 else -speed
