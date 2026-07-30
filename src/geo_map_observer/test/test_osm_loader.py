"""Tests for the ROS-independent offline OSM loader."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from geo_map_observer.osm_loader import load_osm_map


TEST_MAP = Path(__file__).parent / 'data' / 'test_map.osm'


def test_successful_parsing_and_highway_filtering():
    data = load_osm_map(str(TEST_MAP))

    assert data.path == str(TEST_MAP.resolve())
    assert data.total_node_count == 4
    assert data.total_way_count == 3
    assert data.retained_highway_way_count == 1
    assert [way.way_id for way in data.highway_ways] == [100]
    assert data.skipped_highway_way_count == 1
    assert data.highway_type_counts == {'primary': 1}
    assert data.loading_duration_sec >= 0.0


def test_ordered_coordinates_and_tags_are_preserved():
    way = load_osm_map(str(TEST_MAP)).highway_ways[0]

    assert way.node_references == (3, 1, 2)
    assert way.coordinates == (
        (48.05, 9.2), (48.1, 9.1), (48.2, 9.3))
    assert way.highway == 'primary'
    assert way.name == 'Test Road'
    assert way.maxspeed == '50'
    assert way.lanes == '2'
    assert way.oneway == 'yes'


def test_bounding_box_uses_all_parsed_nodes():
    bounds = load_osm_map(str(TEST_MAP)).bounding_box

    assert bounds.min_latitude == 48.05
    assert bounds.max_latitude == 48.3
    assert bounds.min_longitude == 9.05
    assert bounds.max_longitude == 9.3


def test_missing_references_are_counted_and_short_way_is_skipped():
    data = load_osm_map(str(TEST_MAP))

    assert data.ways_with_missing_node_references == 1
    assert data.missing_node_reference_count == 1
    assert 300 not in [way.way_id for way in data.highway_ways]


def test_nonexistent_file_raises_clear_error(tmp_path):
    missing = tmp_path / 'missing.osm'
    with pytest.raises(FileNotFoundError, match='does not exist'):
        load_osm_map(str(missing))


def test_invalid_xml_error_is_propagated(tmp_path):
    invalid = tmp_path / 'invalid.osm'
    invalid.write_text('<osm><node></osm>', encoding='utf-8')

    with pytest.raises(ET.ParseError):
        load_osm_map(str(invalid))
