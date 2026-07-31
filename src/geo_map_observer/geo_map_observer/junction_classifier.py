"""ROS-independent static classification of topology junction candidates."""

import math
from dataclasses import dataclass
from enum import Enum
from itertools import combinations
from typing import Iterable, Optional, Tuple


class JunctionType(str, Enum):
    T_JUNCTION = 'T_JUNCTION'
    Y_JUNCTION = 'Y_JUNCTION'
    CROSS_INTERSECTION = 'CROSS_INTERSECTION'
    THREE_WAY_UNKNOWN = 'THREE_WAY_UNKNOWN'
    FOUR_WAY_UNKNOWN = 'FOUR_WAY_UNKNOWN'
    MULTI_WAY = 'MULTI_WAY'


@dataclass(frozen=True)
class ClassifiedJunction:
    node_id: int
    latitude: float
    longitude: float
    junction_type: JunctionType
    physical_branch_count: int
    physical_branch_bearings_deg: Tuple[float, ...]
    connected_way_ids: Tuple[int, ...]
    has_traffic_signals: bool


def circular_angle_difference_deg(a: float, b: float) -> float:
    """Return the smallest unsigned circular difference in ``[0, 180]``."""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _opposite(a: float, b: float, tolerance_deg: float) -> bool:
    return abs(circular_angle_difference_deg(a, b) - 180.0) <= tolerance_deg


def _valid_distinct_bearings(values: Iterable[float], count: int):
    try:
        bearings = tuple(float(value) % 360.0 for value in values)
    except (TypeError, ValueError):
        return None
    if (len(bearings) != count or not all(math.isfinite(v) for v in bearings)
            or len(set(bearings)) != count):
        return None
    return bearings


def classify_junction_candidate(
        candidate: object, opposite_tolerance_deg: float = 25.0,
) -> Optional[ClassifiedJunction]:
    """Classify one candidate without modifying it; skip fewer than 3 branches."""
    if not 0.0 < opposite_tolerance_deg < 90.0:
        raise ValueError(
            'opposite_tolerance_deg must be greater than 0 and less than 90')
    count = int(candidate.physical_branch_count)
    if count < 3:
        return None
    bearings = _valid_distinct_bearings(
        candidate.physical_branch_bearings_deg, count)
    if count == 3:
        junction_type = JunctionType.THREE_WAY_UNKNOWN
        if bearings is not None:
            junction_type = (JunctionType.T_JUNCTION if any(
                _opposite(a, b, opposite_tolerance_deg)
                for a, b in combinations(bearings, 2))
                else JunctionType.Y_JUNCTION)
    elif count == 4:
        junction_type = JunctionType.FOUR_WAY_UNKNOWN
        if bearings is not None:
            pairings = (((0, 1), (2, 3)), ((0, 2), (1, 3)),
                        ((0, 3), (1, 2)))
            if any(all(_opposite(bearings[a], bearings[b],
                                 opposite_tolerance_deg) for a, b in pairing)
                   for pairing in pairings):
                junction_type = JunctionType.CROSS_INTERSECTION
    else:
        junction_type = JunctionType.MULTI_WAY
    return ClassifiedJunction(
        node_id=int(candidate.node_id), latitude=float(candidate.latitude),
        longitude=float(candidate.longitude), junction_type=junction_type,
        physical_branch_count=count,
        physical_branch_bearings_deg=tuple(
            candidate.physical_branch_bearings_deg),
        connected_way_ids=tuple(candidate.connected_way_ids),
        has_traffic_signals=bool(candidate.has_traffic_signals))


def classify_junction_candidates(
        candidates: Iterable[object], opposite_tolerance_deg: float = 25.0,
) -> Tuple[ClassifiedJunction, ...]:
    """Classify a collection, safely omitting non-junction inputs."""
    results = []
    for candidate in candidates:
        result = classify_junction_candidate(candidate, opposite_tolerance_deg)
        if result is not None:
            results.append(result)
    return tuple(results)
