"""Data records, JSON serialization, and asynchronous persistence."""

from dataclasses import asdict, dataclass
import json
from queue import Queue
from threading import Thread
from typing import List, Optional, Sequence


@dataclass(frozen=True)
class LanePoint:
    longitudinal: float
    lateral: float


@dataclass(frozen=True)
class ObservedLane:
    id: int
    role: str
    confidence: Optional[float]
    points: List[LanePoint]


@dataclass(frozen=True)
class LaneObservation:
    timestamp_ns: int
    frame_id: str
    status: str
    lane_marking_count: int
    left_available: bool
    right_available: bool
    lanes: List[ObservedLane]
    lane_width: Optional[float]
    curvature: Optional[float]
    radius: Optional[float]
    direction: str
    processing_time_ms: float
    coordinate_unit: str = "metre"

    def to_dict(self):
        """Return the stable wire/file representation."""
        value = asdict(self)
        value["header"] = {
            "stamp": {"sec": self.timestamp_ns // 1_000_000_000,
                      "nanosec": self.timestamp_ns % 1_000_000_000},
            "frame_id": self.frame_id,
        }
        return value

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"), allow_nan=False)


def make_lanes(projected_lanes: Sequence, ego_left, ego_right) -> List[ObservedLane]:
    """Convert already-projected LaneDet arrays without projecting again."""
    result = []
    for lane_id, lane in enumerate(projected_lanes):
        role = "other"
        if getattr(ego_left, "size", 0) and lane is ego_left:
            role = "ego_left"
        elif getattr(ego_right, "size", 0) and lane is ego_right:
            role = "ego_right"
        points = [LanePoint(float(point[0]), float(point[1])) for point in lane]
        if points:
            result.append(ObservedLane(lane_id, role, None, points))
    return result


class JsonlWriter:
    """Single background writer; callback only performs a non-blocking enqueue."""

    def __init__(self, path: str):
        self._queue = Queue()
        self._thread = Thread(target=self._run, args=(path,), daemon=True)
        self._thread.start()

    def write(self, observation: LaneObservation) -> None:
        self._queue.put_nowait(observation.to_json())

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=2.0)

    def _run(self, path: str) -> None:
        with open(path, "a", encoding="utf-8") as stream:
            while True:
                line = self._queue.get()
                if line is None:
                    return
                stream.write(line + "\n")
                stream.flush()
