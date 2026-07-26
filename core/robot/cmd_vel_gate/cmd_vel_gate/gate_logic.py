from enum import Enum


class Mode(Enum):
    STOPPED = "stopped"
    MANUAL = "manual"
    AUTOMATIC = "automatic"


class GateState:
    def __init__(self) -> None:
        self.mode = Mode.AUTOMATIC
        self._last_accepted_at = None

    def stop(self) -> None:
        self.mode = Mode.STOPPED
        self._last_accepted_at = None

    def select(self, target: Mode) -> None:
        if self.mode is not Mode.STOPPED:
            raise ValueError("a source can only be selected while stopped")
        if target not in (Mode.MANUAL, Mode.AUTOMATIC):
            raise ValueError("target must be manual or automatic")
        self.mode = target

    def accept(self, source: Mode, received_at: float) -> bool:
        if source is not self.mode:
            return False
        self._last_accepted_at = received_at
        return True

    def selected_source_is_stale(self, now: float, timeout: float) -> bool:
        return (
            self._last_accepted_at is None
            or now - self._last_accepted_at >= timeout
        )
