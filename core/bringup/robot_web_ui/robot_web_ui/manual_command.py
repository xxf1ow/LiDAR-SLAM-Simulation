from __future__ import annotations

import math
from numbers import Real


def command_values(
    direction: str,
    speed_percent: Real,
    max_linear: Real,
    max_angular: Real,
) -> tuple[float, float]:
    values = (speed_percent, max_linear, max_angular)
    if any(
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError("speed values must be finite numbers")
    if not 0 <= speed_percent <= 100:
        raise ValueError("speed_percent must be between 0 and 100")

    scale = speed_percent / 100.0
    mapping = {
        "forward": (max_linear * scale, 0.0),
        "backward": (-max_linear * scale, 0.0),
        "left": (0.0, max_angular * scale),
        "right": (0.0, -max_angular * scale),
        "stop": (0.0, 0.0),
    }
    try:
        return mapping[direction]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"unknown direction: {direction}") from exc
