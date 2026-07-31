"""ROS-independent nearest classified junction lookup."""

import math
from dataclasses import dataclass
from typing import Iterable, Optional


EARTH_RADIUS_M = 6371008.8


@dataclass(frozen=True)
class NearestJunction:
    """A classified junction and its great-circle distance from a position."""

    node_id: int
    junction_type: str
    distance_m: float
    latitude: float
    longitude: float


class JunctionLookup:
    """Hold a startup snapshot of classified junctions for linear searches."""

    def __init__(self, junctions: Iterable[object]) -> None:
        self._junctions = tuple(junctions)

    @property
    def junction_count(self) -> int:
        return len(self._junctions)

    def nearest(
            self, latitude: float,
            longitude: float) -> Optional[NearestJunction]:
        """Return the nearest junction, resolving exact ties by lowest node ID."""
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            raise ValueError('latitude and longitude must be finite')
        candidates = []  # type: list
        for junction in self._junctions:
            junction_type = getattr(junction, 'junction_type')
            candidates.append((
                _distance_m(latitude, longitude, float(junction.latitude),
                            float(junction.longitude)),
                int(junction.node_id),
                str(getattr(junction_type, 'value', junction_type)),
                float(junction.latitude), float(junction.longitude)))
        if not candidates:
            return None
        distance, node_id, junction_type, junction_lat, junction_lon = min(
            candidates, key=lambda item: (item[0], item[1]))
        return NearestJunction(
            node_id, junction_type, distance, junction_lat, junction_lon)


def _distance_m(
        latitude_a: float, longitude_a: float,
        latitude_b: float, longitude_b: float) -> float:
    """Calculate haversine distance between two WGS84 latitude/longitude pairs."""
    lat_a, lat_b = math.radians(latitude_a), math.radians(latitude_b)
    delta_lat = lat_b - lat_a
    delta_lon = math.radians(longitude_b - longitude_a)
    haversine = (math.sin(delta_lat / 2.0) ** 2 + math.cos(lat_a) *
                 math.cos(lat_b) * math.sin(delta_lon / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(haversine)))
