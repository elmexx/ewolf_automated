"""ROS-independent GeoJSON and runtime-state export for the local viewer."""

import json
import math
import os
import shutil
import time
from collections import deque
from pathlib import Path
from typing import Callable, Iterable, Mapping, Optional

from geo_map_observer.road_matcher import RoadMatch, parse_maxspeed_kmh


class VisualizationExporter:
    """Own bounded visualization state and atomically publish browser files."""

    def __init__(
            self, output_dir: str, update_interval_sec: float,
            max_track_points: int, follow_vehicle_default: bool = True,
            clock_ns: Callable[[], int] = time.monotonic_ns) -> None:
        if update_interval_sec <= 0.0:
            raise ValueError(
                'visualization_update_interval_sec must be greater than zero')
        if max_track_points <= 0:
            raise ValueError(
                'visualization_max_track_points must be greater than zero')
        self.output_dir = Path(output_dir).expanduser().resolve()
        self._interval_ns = int(update_interval_sec * 1_000_000_000)
        self._clock_ns = clock_ns
        self._last_write_ns = None  # type: Optional[int]
        self._last_match_identity = None
        self._latest_gnss = None
        self._latest_match = self._empty_match()
        self._gnss_track = deque(maxlen=max_track_points)
        self._matched_track = deque(maxlen=max_track_points)
        self._counters = self._empty_counters()
        self._follow_vehicle_default = bool(follow_vehicle_default)
        self.runtime_write_count = 0

    @staticmethod
    def _empty_counters():
        return dict(gnss_received_count=0, match_attempt_count=0,
                    matched_count=0, unmatched_count=0)

    @staticmethod
    def _empty_match():
        return dict(matched=False, way_id=None, highway=None, name=None,
                    ref=None, maxspeed_raw=None, maxspeed_kmh=None,
                    lanes_raw=None, oneway_raw=None, distance_m=None,
                    nearest_latitude=None, nearest_longitude=None)

    def initialize(self, asset_dir: str) -> None:
        """Create the owned output files and copy the three static assets."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for name in ('index.html', 'viewer.js', 'viewer.css'):
            shutil.copyfile(Path(asset_dir) / name, self.output_dir / name)
        self._atomic_json('runtime_state.json', self.runtime_state())

    def export_highways(
            self, drivable_ways: Iterable[object], contextual_ways: Iterable[object],
            export_drivable: bool = True,
            export_contextual: bool = True) -> Mapping[str, int]:
        """Write each selected OSM way once without modifying source objects."""
        features = []
        seen = set()
        counts = {'drivable': 0, 'contextual': 0}
        groups = ((drivable_ways, 'drivable', export_drivable),
                  (contextual_ways, 'contextual', export_contextual))
        for ways, group, enabled in groups:
            if not enabled:
                continue
            for way in ways:
                if way.way_id in seen:
                    continue
                seen.add(way.way_id)
                features.append({
                    'type': 'Feature',
                    'geometry': {'type': 'LineString', 'coordinates': [
                        [longitude, latitude]
                        for latitude, longitude in way.coordinates]},
                    'properties': {
                        'way_id': way.way_id, 'highway': way.highway,
                        'road_group': group, 'name': getattr(way, 'name', None),
                        'ref': getattr(way, 'ref', None),
                        'maxspeed_raw': getattr(way, 'maxspeed', None),
                        'maxspeed_kmh': parse_maxspeed_kmh(
                            getattr(way, 'maxspeed', None)),
                        'lanes_raw': getattr(way, 'lanes', None),
                        'oneway_raw': getattr(way, 'oneway', None),
                        'junction': getattr(way, 'junction', None),
                    },
                })
                counts[group] += 1
        self._atomic_json('osm_highways.geojson',
                          {'type': 'FeatureCollection', 'features': features})
        return {'total': len(features), **counts}

    def observe(
            self, gnss: object, match: Optional[RoadMatch],
            counters: Mapping[str, int], now_ns: Optional[int] = None,
            force: bool = False) -> bool:
        """Accumulate every observation and publish if due or match identity changed."""
        self._latest_gnss = {
            'timestamp_ns': gnss.timestamp_ns, 'latitude': gnss.latitude,
            'longitude': gnss.longitude,
            'altitude': gnss.altitude if math.isfinite(gnss.altitude) else None,
            'status': gnss.status,
        }
        self._gnss_track.append({
            'timestamp_ns': gnss.timestamp_ns, 'latitude': gnss.latitude,
            'longitude': gnss.longitude,
        })
        if match is None:
            latest_match = self._empty_match()
        else:
            latest_match = {
                'matched': match.matched, 'way_id': match.osm_way_id,
                'highway': match.highway, 'name': match.name, 'ref': match.ref,
                'maxspeed_raw': match.maxspeed_raw,
                'maxspeed_kmh': match.maxspeed_kmh,
                'lanes_raw': match.lanes_raw, 'oneway_raw': match.oneway_raw,
                'distance_m': match.distance_m,
                'nearest_latitude': match.nearest_latitude,
                'nearest_longitude': match.nearest_longitude,
            }
            if match.matched:
                self._matched_track.append({
                    'timestamp_ns': match.timestamp_ns,
                    'latitude': match.nearest_latitude,
                    'longitude': match.nearest_longitude,
                    'way_id': match.osm_way_id,
                })
        self._latest_match = latest_match
        self._counters = dict(counters)
        identity = (latest_match['matched'], latest_match['way_id'],
                    latest_match['highway'])
        important = (self._last_match_identity is not None and
                     identity != self._last_match_identity)
        self._last_match_identity = identity
        current = self._clock_ns() if now_ns is None else now_ns
        due = (self._last_write_ns is None or
               current - self._last_write_ns >= self._interval_ns)
        if force or important or due:
            self.write_runtime(now_ns=current)
            return True
        return False

    def runtime_state(self):
        return {
            'updated_at_ns': time.time_ns(), **self._counters,
            'follow_vehicle_default': self._follow_vehicle_default,
            'latest_gnss': self._latest_gnss,
            'latest_match': self._latest_match,
            'gnss_track': list(self._gnss_track),
            'matched_track': list(self._matched_track),
        }

    def write_runtime(self, now_ns: Optional[int] = None,
                      counters: Optional[Mapping[str, int]] = None) -> None:
        if counters is not None:
            self._counters = dict(counters)
        self._atomic_json('runtime_state.json', self.runtime_state())
        self._last_write_ns = self._clock_ns() if now_ns is None else now_ns
        self.runtime_write_count += 1

    def _atomic_json(self, filename: str, value: object) -> None:
        destination = self.output_dir / filename
        temporary = self.output_dir / (filename + '.tmp')
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(value, stream, separators=(',', ':'), allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(destination))
