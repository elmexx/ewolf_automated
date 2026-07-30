"""ROS-independent loading of road geometry from offline OSM XML files."""

import os
import time
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple


@dataclass(frozen=True)
class OsmNode:
    """A geographic node from an OSM document."""

    node_id: int
    latitude: float
    longitude: float


@dataclass(frozen=True)
class OsmHighwayWay:
    """An ordered OSM way carrying a highway tag."""

    way_id: int
    highway: str
    name: Optional[str]
    maxspeed: Optional[str]
    lanes: Optional[str]
    oneway: Optional[str]
    node_references: Tuple[int, ...]
    coordinates: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class OsmBoundingBox:
    """Geographic extent of all parsed OSM nodes."""

    min_latitude: float
    max_latitude: float
    min_longitude: float
    max_longitude: float


@dataclass(frozen=True)
class OsmMapData:
    """Reusable road geometry and parsing statistics for an OSM map."""

    path: str
    nodes: Mapping[int, OsmNode]
    highway_ways: Tuple[OsmHighwayWay, ...]
    bounding_box: Optional[OsmBoundingBox]
    total_way_count: int
    skipped_highway_way_count: int
    ways_with_missing_node_references: int
    missing_node_reference_count: int
    highway_type_counts: Mapping[str, int]
    loading_duration_sec: float

    @property
    def total_node_count(self) -> int:
        return len(self.nodes)

    @property
    def retained_highway_way_count(self) -> int:
        return len(self.highway_ways)


def resolve_osm_path(map_file: str) -> str:
    """Expand and validate an offline OSM file path."""
    path = os.path.abspath(os.path.expanduser(map_file))
    if not os.path.exists(path):
        raise FileNotFoundError('OSM map file does not exist: {}'.format(path))
    if not os.path.isfile(path):
        raise ValueError('OSM map path is not a regular file: {}'.format(path))
    if os.path.splitext(path)[1].lower() != '.osm':
        raise ValueError('OSM map file must have a .osm extension: {}'.format(path))
    return path


def load_osm_map(map_file: str) -> OsmMapData:
    """Load nodes and highway ways from an offline OSM XML document."""
    path = resolve_osm_path(map_file)
    started = time.perf_counter()
    nodes: Dict[int, OsmNode] = {}
    raw_ways = []

    # Keep only the compact data needed after each element has been consumed.
    for _, element in ET.iterparse(path, events=('end',)):
        if element.tag == 'node':
            node = OsmNode(
                node_id=int(element.attrib['id']),
                latitude=float(element.attrib['lat']),
                longitude=float(element.attrib['lon']),
            )
            nodes[node.node_id] = node
            element.clear()
        elif element.tag == 'way':
            references = tuple(
                int(child.attrib['ref']) for child in element if child.tag == 'nd')
            tags = {
                child.attrib['k']: child.attrib['v']
                for child in element if child.tag == 'tag'
            }
            raw_ways.append((int(element.attrib['id']), references, tags))
            element.clear()

    highways = []
    skipped = 0
    ways_with_missing = 0
    missing_references = 0
    highway_counts = Counter()
    for way_id, references, tags in raw_ways:
        highway = tags.get('highway')
        if highway is None:
            continue
        missing = sum(1 for reference in references if reference not in nodes)
        if missing:
            ways_with_missing += 1
            missing_references += missing
        coordinates = tuple(
            (nodes[reference].latitude, nodes[reference].longitude)
            for reference in references if reference in nodes
        )
        if len(coordinates) < 2:
            skipped += 1
            continue
        highways.append(OsmHighwayWay(
            way_id=way_id,
            highway=highway,
            name=tags.get('name'),
            maxspeed=tags.get('maxspeed'),
            lanes=tags.get('lanes'),
            oneway=tags.get('oneway'),
            node_references=references,
            coordinates=coordinates,
        ))
        highway_counts[highway] += 1

    bounding_box = None
    if nodes:
        latitudes = [node.latitude for node in nodes.values()]
        longitudes = [node.longitude for node in nodes.values()]
        bounding_box = OsmBoundingBox(
            min_latitude=min(latitudes),
            max_latitude=max(latitudes),
            min_longitude=min(longitudes),
            max_longitude=max(longitudes),
        )

    return OsmMapData(
        path=path,
        nodes=nodes,
        highway_ways=tuple(highways),
        bounding_box=bounding_box,
        total_way_count=len(raw_ways),
        skipped_highway_way_count=skipped,
        ways_with_missing_node_references=ways_with_missing,
        missing_node_reference_count=missing_references,
        highway_type_counts=dict(highway_counts),
        loading_duration_sec=time.perf_counter() - started,
    )
