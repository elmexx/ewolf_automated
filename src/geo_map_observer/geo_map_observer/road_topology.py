"""ROS-independent construction of topology from drivable OSM ways."""

import math
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple

from geo_map_observer.osm_loader import OsmHighwayWay, OsmMapData
from geo_map_observer.road_matcher import EARTH_RADIUS_M, parse_maxspeed_kmh


@dataclass(frozen=True)
class RoadSegment:
    segment_id: str
    way_id: int
    start_node_id: int
    end_node_id: int
    start_latitude: float
    start_longitude: float
    end_latitude: float
    end_longitude: float
    forward_bearing_deg: float
    length_m: float
    highway: str
    name: Optional[str]
    ref: Optional[str]
    maxspeed_raw: Optional[str]
    maxspeed_kmh: Optional[float]
    lanes_raw: Optional[str]
    oneway_raw: Optional[str]


@dataclass(frozen=True)
class TopologyNode:
    node_id: int
    latitude: float
    longitude: float
    incident_segment_ids: Tuple[str, ...]
    connected_way_ids: Tuple[int, ...]
    outgoing_branch_bearings_deg: Tuple[float, ...]
    physical_branch_bearings_deg: Tuple[float, ...]
    branch_groups: Tuple[Tuple[float, ...], ...]
    tags: Mapping[str, str]


@dataclass(frozen=True)
class JunctionCandidate:
    node_id: int
    latitude: float
    longitude: float
    connected_way_ids: Tuple[int, ...]
    incident_segment_ids: Tuple[str, ...]
    raw_branch_count: int
    physical_branch_count: int
    raw_branch_bearings_deg: Tuple[float, ...]
    physical_branch_bearings_deg: Tuple[float, ...]
    highway_types: Tuple[str, ...]
    road_names: Tuple[str, ...]
    road_refs: Tuple[str, ...]
    has_traffic_signals: bool
    explicit_junction_tags: Mapping[str, str]


@dataclass(frozen=True)
class TopologyBuildStatistics:
    drivable_way_count: int
    topology_node_count: int
    segment_count: int
    skipped_invalid_segment_count: int
    missing_node_reference_count: int
    zero_length_segment_count: int
    junction_candidate_count: int
    candidate_counts_by_physical_branch_count: Mapping[int, int]
    traffic_signal_candidate_count: int
    stop_node_count: int
    give_way_node_count: int
    roundabout_way_ids: Tuple[int, ...]
    circular_junction_way_ids: Tuple[int, ...]
    build_duration_sec: float


@dataclass(frozen=True)
class RoadTopology:
    nodes: Mapping[int, TopologyNode]
    segments: Mapping[str, RoadSegment]
    drivable_way_ids: Tuple[int, ...]
    junction_candidates: Tuple[JunctionCandidate, ...]
    statistics: TopologyBuildStatistics


def circular_angular_difference(first: float, second: float) -> float:
    """Return the smallest unsigned difference between two bearings."""
    return abs((first - second + 180.0) % 360.0 - 180.0)


def merge_branch_bearings(
        bearings: Sequence[float], merge_angle_deg: float,
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """Group bearings using circular, transitive angular similarity."""
    if not 0.0 < merge_angle_deg < 90.0:
        raise ValueError('merge_angle_deg must be greater than 0 and less than 90')
    normalized = tuple(float(value) % 360.0 for value in bearings)
    remaining = set(range(len(normalized)))
    groups = []
    while remaining:
        pending = [remaining.pop()]
        indices = []
        while pending:
            current = pending.pop()
            indices.append(current)
            neighbours = {
                index for index in remaining
                if circular_angular_difference(
                    normalized[current], normalized[index]) <= merge_angle_deg
            }
            remaining.difference_update(neighbours)
            pending.extend(neighbours)
        groups.append(tuple(sorted(normalized[index] for index in indices)))
    groups.sort(key=lambda group: group[0])
    representatives = tuple(
        math.degrees(math.atan2(
            sum(math.sin(math.radians(value)) for value in group),
            sum(math.cos(math.radians(value)) for value in group))) % 360.0
        for group in groups)
    return tuple(groups), representatives


def _segment_geometry(start, end):
    mean_latitude = math.radians((start.latitude + end.latitude) / 2.0)
    east = math.radians(end.longitude - start.longitude) * (
        EARTH_RADIUS_M * math.cos(mean_latitude))
    north = math.radians(end.latitude - start.latitude) * EARTH_RADIUS_M
    return math.hypot(east, north), math.degrees(math.atan2(east, north)) % 360.0


def build_road_topology(
        map_data: OsmMapData, drivable_ways: Sequence[OsmHighwayWay],
        branch_merge_angle_deg: float = 20.0,
) -> RoadTopology:
    """Build shared-node topology without modifying map data or its ways."""
    if not 0.0 < branch_merge_angle_deg < 90.0:
        raise ValueError(
            'branch_merge_angle_deg must be greater than 0 and less than 90')
    started = time.perf_counter()
    segments: Dict[str, RoadSegment] = {}
    incidents = defaultdict(list)
    missing = zero_length = invalid = 0
    ways_by_id = {way.way_id: way for way in drivable_ways}
    roundabouts = tuple(sorted(
        way.way_id for way in drivable_ways if way.junction == 'roundabout'))
    circular = tuple(sorted(
        way.way_id for way in drivable_ways if way.junction == 'circular'))

    for way in drivable_ways:
        missing += sum(
            reference not in map_data.nodes
            for reference in way.node_references)
        for index, (start_id, end_id) in enumerate(zip(
                way.node_references, way.node_references[1:])):
            absent = int(start_id not in map_data.nodes) + int(
                end_id not in map_data.nodes)
            if absent:
                invalid += 1
                continue
            start, end = map_data.nodes[start_id], map_data.nodes[end_id]
            length, bearing = _segment_geometry(start, end)
            if not math.isfinite(length) or not math.isfinite(bearing):
                invalid += 1
                continue
            if length <= 1e-9:
                zero_length += 1
                invalid += 1
                continue
            segment_id = '{}:{}'.format(way.way_id, index)
            segment = RoadSegment(
                segment_id, way.way_id, start_id, end_id,
                start.latitude, start.longitude, end.latitude, end.longitude,
                bearing, length, way.highway, way.name, way.ref, way.maxspeed,
                parse_maxspeed_kmh(way.maxspeed), way.lanes, way.oneway)
            segments[segment_id] = segment
            incidents[start_id].append((segment_id, way.way_id, bearing))
            incidents[end_id].append(
                (segment_id, way.way_id, (bearing + 180.0) % 360.0))

    nodes = {}
    candidates = []
    for node_id, entries in incidents.items():
        source = map_data.nodes[node_id]
        raw = tuple(sorted(entry[2] for entry in entries))
        groups, physical = merge_branch_bearings(raw, branch_merge_angle_deg)
        node = TopologyNode(
            node_id, source.latitude, source.longitude,
            tuple(entry[0] for entry in entries),
            tuple(sorted(set(entry[1] for entry in entries))), raw,
            physical, groups, dict(source.tags))
        nodes[node_id] = node
        if len(physical) < 3:
            continue
        incident_ways = [ways_by_id[way_id] for way_id in node.connected_way_ids]
        candidates.append(JunctionCandidate(
            node_id, node.latitude, node.longitude, node.connected_way_ids,
            node.incident_segment_ids, len(raw), len(physical), raw, physical,
            tuple(sorted(set(way.highway for way in incident_ways))),
            tuple(sorted(set(way.name for way in incident_ways if way.name))),
            tuple(sorted(set(way.ref for way in incident_ways if way.ref))),
            source.tags.get('highway') == 'traffic_signals',
            {key: value for key, value in source.tags.items()
             if key in ('junction', 'crossing', 'traffic_signals')}))

    branch_counts = Counter(item.physical_branch_count for item in candidates)
    stats = TopologyBuildStatistics(
        len(drivable_ways), len(nodes), len(segments), invalid, missing,
        zero_length, len(candidates), dict(branch_counts),
        sum(item.has_traffic_signals for item in candidates),
        sum(node.tags.get('highway') == 'stop' for node in map_data.nodes.values()),
        sum(node.tags.get('highway') == 'give_way' for node in map_data.nodes.values()),
        roundabouts, circular, time.perf_counter() - started)
    return RoadTopology(nodes, segments,
                        tuple(way.way_id for way in drivable_ways),
                        tuple(candidates), stats)
