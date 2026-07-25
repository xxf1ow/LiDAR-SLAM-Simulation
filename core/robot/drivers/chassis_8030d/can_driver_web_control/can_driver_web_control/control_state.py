from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence


class ControlError(ValueError):
    """The request cannot be represented as a valid motor command."""


class ControlConflict(RuntimeError):
    """The request is valid but conflicts with the disabled driver state."""


# SDK topic order is [right wheel, left wheel]. The precompiled driver then
# negates its first motor channel internally to compensate for mirrored motors.
_MAPPING = {
    "forward": (1, 1),
    "backward": (-1, -1),
    "left": (1, -1),
    "right": (-1, 1),
    "stop": (0, 0),
}


class ControlState:
    def __init__(
        self,
        default_rpm: int = 20,
        max_rpm: int = 100,
        timeout_sec: float = 0.3,
        feedback_timeout_sec: float = 1.0,
        clock: Callable[[], float] | None = None,
    ) -> None:
        if isinstance(default_rpm, bool) or not isinstance(default_rpm, int):
            raise ControlError("default_rpm must be an integer")
        if not 0 <= default_rpm <= max_rpm:
            raise ControlError("default_rpm must be within the RPM limit")
        self._max_rpm = max_rpm
        self._timeout_sec = timeout_sec
        self._feedback_timeout_sec = feedback_timeout_sec
        self._clock = clock or time.monotonic
        self._lock = threading.Lock()
        self._enabled = False
        self._direction = "stop"
        self._speed_rpm = default_rpm
        self._last_refresh: float | None = None
        self._feedback: tuple[int, int] | None = None
        self._last_feedback: float | None = None
        self._driver_connected = False

    def set_enabled(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            raise ControlError("enabled must be a boolean")
        with self._lock:
            self._enabled = enabled
            self._direction = "stop"
            self._last_refresh = None

    def set_command(self, direction: str, speed_rpm: int) -> None:
        if not isinstance(direction, str) or direction not in _MAPPING:
            raise ControlError(f"unknown direction: {direction}")
        if isinstance(speed_rpm, bool) or not isinstance(speed_rpm, int):
            raise ControlError("speed_rpm must be an integer")
        if not 0 <= speed_rpm <= self._max_rpm:
            raise ControlError(f"speed_rpm must be between 0 and {self._max_rpm}")
        now = self._clock()
        with self._lock:
            if direction != "stop" and not self._enabled:
                raise ControlConflict("enable the driver before moving")
            self._direction = direction
            self._speed_rpm = speed_rpm
            self._last_refresh = now if direction != "stop" else None

    def set_driver_connected(self, connected: bool) -> None:
        with self._lock:
            self._driver_connected = bool(connected)

    def update_feedback(self, values: Sequence[int]) -> None:
        if len(values) < 2:
            raise ControlError("current_speed must contain at least two values")
        feedback = (int(values[0]), int(values[1]))
        with self._lock:
            self._feedback = feedback
            self._last_feedback = self._clock()

    def _has_fresh_feedback_locked(self, now: float) -> bool:
        return (
            self._last_feedback is not None
            and now - self._last_feedback <= self._feedback_timeout_sec
        )

    def _safe_command_locked(self, now: float) -> tuple[int, int]:
        if not self._enabled or self._direction == "stop":
            return (0, 0)
        if self._last_refresh is None or now - self._last_refresh > self._timeout_sec:
            self._direction = "stop"
            self._last_refresh = None
            return (0, 0)
        left_sign, right_sign = _MAPPING[self._direction]
        return (left_sign * self._speed_rpm, right_sign * self._speed_rpm)

    def safe_command(self) -> tuple[int, int]:
        with self._lock:
            return self._safe_command_locked(self._clock())

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            now = self._clock()
            command = self._safe_command_locked(now)
            return {
                # This is only the requested state. The vendor SDK exposes no
                # ROS topic or service that confirms the 8030D statusword.
                "enabled": self._enabled,
                "enable_confirmed": None,
                "direction": self._direction,
                "speed_rpm": self._speed_rpm,
                "command": list(command),
                "feedback": list(self._feedback) if self._feedback is not None else None,
                "driver_connected": self._driver_connected,
                "hardware_feedback": self._has_fresh_feedback_locked(now),
            }
