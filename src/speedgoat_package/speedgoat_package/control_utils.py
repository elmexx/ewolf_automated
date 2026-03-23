from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, List, Optional, Sequence, Tuple
import math

Point = Tuple[float, float]


@dataclass
class SteeringProtectionConfig:
    max_angle_rad: float
    smoothing_alpha: float = 0.25
    rate_limit_rad_per_sec: float = math.radians(90.0)
    deadband_rad: float = math.radians(0.5)
    nominal_dt: float = 0.1


@dataclass
class SteeringProtectionMetrics:
    raw_angle_rad: float
    smoothed_angle_rad: float
    final_angle_rad: float
    rate_limited: bool
    deadband_applied: bool
    applied_delta_rad: float


class SteeringProtector:
    def __init__(self, config: SteeringProtectionConfig):
        self.config = config
        self._last_output = 0.0
        self._initialized = False

    def reset(self) -> None:
        self._last_output = 0.0
        self._initialized = False

    def apply(self, raw_angle_rad: float, dt: Optional[float] = None) -> SteeringProtectionMetrics:
        if dt is None or dt <= 0.0:
            dt = self.config.nominal_dt

        raw_clamped = clamp(raw_angle_rad, -self.config.max_angle_rad, self.config.max_angle_rad)
        if not self._initialized:
            smoothed = raw_clamped
            rate_limited = False
            self._initialized = True
        else:
            smoothed = (
                self.config.smoothing_alpha * raw_clamped
                + (1.0 - self.config.smoothing_alpha) * self._last_output
            )
            max_delta = self.config.rate_limit_rad_per_sec * dt
            delta = smoothed - self._last_output
            rate_limited = abs(delta) > max_delta
            if rate_limited:
                smoothed = self._last_output + math.copysign(max_delta, delta)

        final_angle = smoothed
        deadband_applied = abs(final_angle) < self.config.deadband_rad
        if deadband_applied:
            final_angle = 0.0

        metrics = SteeringProtectionMetrics(
            raw_angle_rad=raw_clamped,
            smoothed_angle_rad=smoothed,
            final_angle_rad=final_angle,
            rate_limited=rate_limited,
            deadband_applied=deadband_applied,
            applied_delta_rad=final_angle - self._last_output,
        )
        self._last_output = final_angle
        return metrics


class DelayedPathBuffer:
    def __init__(self, delay_frames: int = 0):
        self.delay_frames = max(0, delay_frames)
        self._queue: Deque[List[Point]] = deque()
        self._last_path: List[Point] = []

    def push(self, path: Sequence[Point]) -> List[Point]:
        path_list = list(path)
        self._queue.append(path_list)
        if self.delay_frames <= 0:
            self._last_path = path_list
            return path_list

        if len(self._queue) <= self.delay_frames:
            self._last_path = path_list
            return path_list

        delayed = self._queue.popleft()
        self._last_path = delayed
        return delayed

    @property
    def last_path(self) -> List[Point]:
        return list(self._last_path)


def mean_abs_path_delta(path_a: Sequence[Point], path_b: Sequence[Point]) -> float:
    if not path_a or not path_b:
        return 0.0
    size = min(len(path_a), len(path_b))
    total = 0.0
    for i in range(size):
        total += math.hypot(path_a[i][0] - path_b[i][0], path_a[i][1] - path_b[i][1])
    return total / float(size)


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)
