import math
from collections import deque
import statistics

class VelocityFilter:
    """
    A time-based hybrid Moving Median + Exponential Moving Average (EMA) filter
    designed to eliminate spikes and smooth velocity vector outputs (North, East, Down).
    """
    def __init__(self, window_duration_s=0.2, ema_alpha=0.3, max_dt_s=0.5, max_velocity_ms=None, enabled=True):
        self.window_duration_s = max(0.0, float(window_duration_s))
        self.ema_alpha = max(0.0, min(1.0, float(ema_alpha)))
        self.max_dt_s = float(max_dt_s)
        self.max_velocity_ms = float(max_velocity_ms) if max_velocity_ms is not None else None
        self.enabled = enabled
        
        self.buffer = deque()  # stores (timestamp, vn, ve, vd)
        self.last_time = None
        self.ema_vn = None
        self.ema_ve = None
        self.ema_vd = None

    def reset(self):
        """Clears buffer and resets EMA history."""
        self.buffer.clear()
        self.last_time = None
        self.ema_vn = None
        self.ema_ve = None
        self.ema_vd = None

    def _clamp_velocity(self, vn, ve, vd):
        if self.max_velocity_ms is None or self.max_velocity_ms <= 0:
            return vn, ve, vd
        speed = math.sqrt(vn * vn + ve * ve + vd * vd)
        if speed > self.max_velocity_ms and speed > 0:
            scale = self.max_velocity_ms / speed
            return vn * scale, ve * scale, vd * scale
        return vn, ve, vd

    def update(self, current_time, vn, ve, vd):
        """
        Updates the filter with a new raw velocity vector and timestamp.
        Returns filtered (vn, ve, vd).
        """
        if not self.enabled:
            return vn, ve, vd

        # Check for gap/stream interruption
        if self.last_time is not None and (current_time - self.last_time > self.max_dt_s or current_time < self.last_time):
            self.reset()

        self.last_time = current_time

        # 1. Optional speed clamping
        vn_c, ve_c, vd_c = self._clamp_velocity(vn, ve, vd)

        # 2. Append to time-based buffer
        self.buffer.append((current_time, vn_c, ve_c, vd_c))

        # 3. Purge samples older than current_time - window_duration_s
        cutoff_time = current_time - self.window_duration_s
        while self.buffer and self.buffer[0][0] < cutoff_time:
            self.buffer.popleft()

        # 4. Compute moving median for each axis
        vns = [sample[1] for sample in self.buffer]
        ves = [sample[2] for sample in self.buffer]
        vds = [sample[3] for sample in self.buffer]

        median_vn = statistics.median(vns)
        median_ve = statistics.median(ves)
        median_vd = statistics.median(vds)

        # 5. Apply Exponential Moving Average (EMA)
        if self.ema_vn is None:
            self.ema_vn = median_vn
            self.ema_ve = median_ve
            self.ema_vd = median_vd
        else:
            a = self.ema_alpha
            self.ema_vn = a * median_vn + (1.0 - a) * self.ema_vn
            self.ema_ve = a * median_ve + (1.0 - a) * self.ema_ve
            self.ema_vd = a * median_vd + (1.0 - a) * self.ema_vd

        return self.ema_vn, self.ema_ve, self.ema_vd
