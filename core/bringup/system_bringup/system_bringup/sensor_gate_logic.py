from collections import deque


POINT_FIELDS = ("x", "y", "z", "intensity", "ring", "time")
POINT_FRAME = "velodyne"
IMU_FRAME = "imu_link"
POINT_HEIGHT = 32
POINT_WIDTH = 1200
MIN_POINT_HZ = 8.0
MIN_IMU_HZ = 150.0
MAX_STAMP_AGE = 0.5
RATE_WINDOW = 2.0
STABLE_DURATION = 2.0


class RateWindow:
    def __init__(self, window=RATE_WINDOW):
        self.window = window
        self.samples = deque()

    def observe(self, received):
        self.samples.append(received)
        self._prune(received)

    def hz(self, now):
        self._prune(now)
        if len(self.samples) < 2:
            return 0.0
        if now - self.samples[-1] > MAX_STAMP_AGE:
            return 0.0
        duration = self.samples[-1] - self.samples[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.samples) - 1) / duration

    def _prune(self, now):
        cutoff = now - self.window
        while self.samples and self.samples[0] < cutoff:
            self.samples.popleft()


class SensorGateState:
    def __init__(self):
        self.point_rate = RateWindow()
        self.imu_rate = RateWindow()
        self.point_problem = "point cloud not received"
        self.imu_problem = "IMU not received"
        self.stable_since = None

    def observe_point(self, received, stamp, now_ros, frame_id, height, width, fields):
        self.point_rate.observe(received)
        problems = self._header_problems(stamp, now_ros)
        if frame_id != POINT_FRAME:
            problems.append(f"point frame {frame_id!r}, expected {POINT_FRAME!r}")
        if (height, width) != (POINT_HEIGHT, POINT_WIDTH):
            problems.append(
                f"point shape {height}x{width}, expected {POINT_HEIGHT}x{POINT_WIDTH}"
            )
        if fields != POINT_FIELDS:
            problems.append(f"point fields {fields!r}, expected {POINT_FIELDS!r}")
        self.point_problem = "; ".join(problems) if problems else None
        if self.point_problem:
            self.stable_since = None

    def observe_imu(self, received, stamp, now_ros, frame_id):
        self.imu_rate.observe(received)
        problems = self._header_problems(stamp, now_ros)
        if frame_id != IMU_FRAME:
            problems.append(f"IMU frame {frame_id!r}, expected {IMU_FRAME!r}")
        self.imu_problem = "; ".join(problems) if problems else None
        if self.imu_problem:
            self.stable_since = None

    def status(self, now):
        point_hz = self.point_rate.hz(now)
        imu_hz = self.imu_rate.hz(now)
        problems = []

        if self.point_problem:
            problems.append(self.point_problem)
        if point_hz < MIN_POINT_HZ:
            problems.append(f"point rate {point_hz:.1f} Hz below {MIN_POINT_HZ:g} Hz")
        if self.imu_problem:
            problems.append(self.imu_problem)
        if imu_hz < MIN_IMU_HZ:
            problems.append(f"IMU rate {imu_hz:.1f} Hz below {MIN_IMU_HZ:g} Hz")

        if problems:
            self.stable_since = None
            return False, "; ".join(problems)

        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since >= STABLE_DURATION:
            return True, "ready"
        return False, "sensor streams are stabilizing"

    def _header_problems(self, stamp, now_ros):
        age = now_ros - stamp
        if 0.0 <= age <= MAX_STAMP_AGE:
            return []
        return [
            f"header age {age:.3f}s outside [0, {MAX_STAMP_AGE:g}]s"
        ]
