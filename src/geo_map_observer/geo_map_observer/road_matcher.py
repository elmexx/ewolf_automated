"""ROS-independent nearest-road matching for retained OSM highway ways."""

import math
import re
from collections import Counter
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Sequence, Tuple

from geo_map_observer.osm_loader import OsmHighwayWay, OsmMapData


# Conservative defaults: highway features intended primarily for walking,
# cycling, or uncertain access (including path and track) remain contextual.
DEFAULT_DRIVABLE_HIGHWAY_TYPES = (
    'motorway', 'motorway_link', 'trunk', 'trunk_link',
    'primary', 'primary_link', 'secondary', 'secondary_link',
    'tertiary', 'tertiary_link', 'unclassified', 'residential',
    'living_street', 'service',
)

EARTH_RADIUS_M = 6_371_008.8


@dataclass(frozen=True)
class RoadCandidateStatistics:
    """Counts for the drivable/contextual partition of retained highways."""

    all_way_count: int
    drivable_way_count: int
    contextual_way_count: int
    drivable_type_counts: Mapping[str, int]
    contextual_type_counts: Mapping[str, int]


@dataclass(frozen=True)
class RoadMatch:
    """Nearest-road result for one accepted GNSS measurement."""

    timestamp_ns: int
    latitude: float
    longitude: float
    matched: bool
    osm_way_id: Optional[int]
    highway: Optional[str]
    name: Optional[str]
    ref: Optional[str]
    maxspeed_raw: Optional[str]
    maxspeed_kmh: Optional[float]
    lanes_raw: Optional[str]
    oneway_raw: Optional[str]
    distance_m: Optional[float]
    nearest_latitude: Optional[float]
    nearest_longitude: Optional[float]


class RoadMatchLogThrottle:
    """Stateful output policy kept separate from the matching operation."""

    def __init__(self, interval_sec: float) -> None:
        if interval_sec < 0.0:
            raise ValueError('interval_sec must be greater than or equal to zero')
        self._interval_ns = int(interval_sec * 1_000_000_000)
        self._last_log_time_ns = None  # type: Optional[int]
        self._last_attributes = None

    def should_log(self, result: RoadMatch, now_ns: int) -> bool:
        """Log immediately on road attributes changing, otherwise throttle."""
        attributes = (result.osm_way_id, result.maxspeed_raw, result.highway)
        unchanged = attributes == self._last_attributes
        if (unchanged and self._last_log_time_ns is not None and
                now_ns - self._last_log_time_ns < self._interval_ns):
            return False
        self._last_log_time_ns = now_ns
        self._last_attributes = attributes
        return True


def classify_highway_ways(
        map_data: OsmMapData,
        drivable_highway_types: Iterable[str] = DEFAULT_DRIVABLE_HIGHWAY_TYPES,
) -> Tuple[Tuple[OsmHighwayWay, ...], Tuple[OsmHighwayWay, ...]]:
    """Partition, but never remove, the map's complete highway collection."""
    enabled = set(drivable_highway_types)
    drivable = tuple(way for way in map_data.highway_ways
                     if way.highway in enabled)
    contextual = tuple(way for way in map_data.highway_ways
                       if way.highway not in enabled)
    return drivable, contextual


def candidate_statistics(
        map_data: OsmMapData,
        drivable_highway_types: Iterable[str] = DEFAULT_DRIVABLE_HIGHWAY_TYPES,
) -> RoadCandidateStatistics:
    """Describe the candidate partition for startup diagnostics."""
    drivable, contextual = classify_highway_ways(
        map_data, drivable_highway_types)
    return RoadCandidateStatistics(
        all_way_count=len(map_data.highway_ways),
        drivable_way_count=len(drivable),
        contextual_way_count=len(contextual),
        drivable_type_counts=dict(Counter(way.highway for way in drivable)),
        contextual_type_counts=dict(Counter(way.highway for way in contextual)),
    )


def parse_maxspeed_kmh(value: Optional[str]) -> Optional[float]:
    """Parse simple numeric OSM maxspeed values without guessing symbolic tags."""
    if value is None:
        return None
    match = re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*(km/h|kmh|kph)?\s*', value,
                         flags=re.IGNORECASE)
    if match:
        return float(match.group(1))
    mph = re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*mph\s*', value,
                       flags=re.IGNORECASE)
    if mph:
        return float(mph.group(1)) * 1.609344
    return None


def _nearest_point_on_segment(
        latitude: float, longitude: float,
        start: Tuple[float, float], end: Tuple[float, float],
) -> Tuple[float, float, float]:
    """Return distance and nearest coordinate using a local metric projection."""
    origin_lat_rad = math.radians(latitude)
    longitude_scale = EARTH_RADIUS_M * math.cos(origin_lat_rad) * math.pi / 180.0
    latitude_scale = EARTH_RADIUS_M * math.pi / 180.0

    ax = (start[1] - longitude) * longitude_scale
    ay = (start[0] - latitude) * latitude_scale
    bx = (end[1] - longitude) * longitude_scale
    by = (end[0] - latitude) * latitude_scale
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    fraction = 0.0 if length_squared == 0.0 else -(ax * dx + ay * dy) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    nearest_x = ax + fraction * dx
    nearest_y = ay + fraction * dy
    nearest_latitude = latitude + nearest_y / latitude_scale
    nearest_longitude = longitude if longitude_scale == 0.0 else (
        longitude + nearest_x / longitude_scale)
    return math.hypot(nearest_x, nearest_y), nearest_latitude, nearest_longitude


def match_nearest_road(
        timestamp_ns: int,
        latitude: float,
        longitude: float,
        highway_ways: Sequence[OsmHighwayWay],
        max_match_distance_m: float = 20.0,
) -> RoadMatch:
    """Match one point against a pre-classified sequence of drivable ways."""
    if max_match_distance_m < 0.0:
        raise ValueError('max_match_distance_m must be greater than or equal to zero')
    nearest_way = None
    nearest_distance = None
    nearest_coordinate = None
    for way in highway_ways:
        for start, end in zip(way.coordinates, way.coordinates[1:]):
            distance, nearest_latitude, nearest_longitude = (
                _nearest_point_on_segment(latitude, longitude, start, end))
            if nearest_distance is None or distance < nearest_distance:
                nearest_way = way
                nearest_distance = distance
                nearest_coordinate = (nearest_latitude, nearest_longitude)

    matched = (nearest_way is not None and nearest_distance is not None and
               nearest_distance <= max_match_distance_m)
    if not matched:
        return RoadMatch(timestamp_ns, latitude, longitude, False, None, None,
                         None, None, None, None, None, None, nearest_distance,
                         None if nearest_coordinate is None else nearest_coordinate[0],
                         None if nearest_coordinate is None else nearest_coordinate[1])
    return RoadMatch(
        timestamp_ns=timestamp_ns, latitude=latitude, longitude=longitude,
        matched=True, osm_way_id=nearest_way.way_id,
        highway=nearest_way.highway, name=nearest_way.name, ref=nearest_way.ref,
        maxspeed_raw=nearest_way.maxspeed,
        maxspeed_kmh=parse_maxspeed_kmh(nearest_way.maxspeed),
        lanes_raw=nearest_way.lanes, oneway_raw=nearest_way.oneway,
        distance_m=nearest_distance,
        nearest_latitude=nearest_coordinate[0],
        nearest_longitude=nearest_coordinate[1],
    )
