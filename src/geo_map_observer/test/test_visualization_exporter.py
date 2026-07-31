"""ROS-independent tests for browser visualization export."""

import json
from copy import deepcopy
from types import SimpleNamespace

import pytest

from geo_map_observer.junction_classifier import ClassifiedJunction, JunctionType
from geo_map_observer.osm_loader import OsmHighwayWay
from geo_map_observer.road_matcher import RoadMatch, match_nearest_road
from geo_map_observer.visualization_exporter import VisualizationExporter


def way(way_id=1, highway='residential', **values):
    defaults = dict(name='Main', ref='A1', maxspeed='30 mph', lanes='2',
                    oneway='yes', junction=None)
    defaults.update(values)
    return OsmHighwayWay(way_id, highway, defaults['name'], defaults['ref'],
                         defaults['maxspeed'], defaults['lanes'],
                         defaults['oneway'], (1, 2),
                         ((50.0, 8.0), (50.1, 8.2)), defaults['junction'], {})


def gnss(timestamp=1, latitude=50.01, longitude=8.02):
    return SimpleNamespace(timestamp_ns=timestamp, latitude=latitude,
                           longitude=longitude, altitude=123.4, status=0)


def match(timestamp=1, matched=True, way_id=1, highway='residential'):
    return RoadMatch(timestamp, 50.01, 8.02, matched,
                     way_id if matched else None, highway if matched else None,
                     'Main' if matched else None, 'A1' if matched else None,
                     '50' if matched else None, 50.0 if matched else None,
                     '2' if matched else None, 'yes' if matched else None, 3.2,
                     50.01001 if matched else None, 8.02001 if matched else None)


def counters(received=1, matched=1, unmatched=0):
    return {'gnss_received_count': received, 'match_attempt_count': received,
            'matched_count': matched, 'unmatched_count': unmatched}


def exporter(tmp_path, limit=10, clock=lambda: 0):
    value = VisualizationExporter(str(tmp_path), 1.0, limit, clock_ns=clock)
    # Tests do not need to copy assets, but do require initial valid state.
    value._atomic_json('runtime_state.json', value.runtime_state())
    return value


def test_static_export_groups_coordinates_attributes_and_deduplication(tmp_path):
    output = exporter(tmp_path)
    drivable = way(junction='roundabout')
    contextual = way(2, 'footway', name=None, ref=None, maxspeed=None,
                     lanes=None, oneway=None)
    counts = output.export_highways((drivable,), (contextual, drivable))
    document = json.loads((tmp_path / 'osm_highways.geojson').read_text())
    assert counts == {'total': 2, 'drivable': 1, 'contextual': 1}
    assert document['features'][0]['geometry']['coordinates'][0] == [8.0, 50.0]
    assert document['features'][0]['properties'] == {
        'way_id': 1, 'highway': 'residential', 'road_group': 'drivable',
        'name': 'Main', 'ref': 'A1', 'maxspeed_raw': '30 mph',
        'maxspeed_kmh': pytest.approx(48.28032), 'lanes_raw': '2',
        'oneway_raw': 'yes', 'junction': 'roundabout'}
    assert document['features'][1]['properties']['name'] is None


@pytest.mark.parametrize('drivable,contextual,expected', [
    (False, True, ['contextual']), (True, False, ['drivable'])])
def test_static_export_filter_does_not_mutate_input(
        tmp_path, drivable, contextual, expected):
    roads = [way(), way(2, 'footway')]
    before = deepcopy(roads)
    output = exporter(tmp_path)
    output.export_highways(roads[:1], roads[1:], drivable, contextual)
    document = json.loads((tmp_path / 'osm_highways.geojson').read_text())
    assert [f['properties']['road_group'] for f in document['features']] == expected
    assert roads == before


def test_empty_runtime_is_valid_and_atomic(tmp_path):
    output = exporter(tmp_path)
    state = json.loads((tmp_path / 'runtime_state.json').read_text())
    assert state['latest_gnss'] is None
    assert state['latest_match']['matched'] is False
    assert state['gnss_track'] == state['matched_track'] == []
    assert not list(tmp_path.glob('*.tmp'))


def test_empty_junction_export_is_valid_and_runtime_remains_compatible(tmp_path):
    output = exporter(tmp_path)
    before = json.loads((tmp_path / 'runtime_state.json').read_text())
    assert before['nearest_junction'] is None
    assert output.export_junctions(()) == 0
    document = json.loads((tmp_path / 'junctions.geojson').read_text())
    assert document == {'type': 'FeatureCollection', 'features': []}
    assert json.loads((tmp_path / 'runtime_state.json').read_text()) == before


def test_runtime_state_exports_nearest_junction_and_remains_valid_json(tmp_path):
    output = exporter(tmp_path)
    nearest = SimpleNamespace(
        node_id=42, junction_type='T_JUNCTION', distance_m=18.4)

    output.observe(
        gnss(), match(), counters(), now_ns=2_000_000_000,
        nearest_junction=nearest)

    state = json.loads((tmp_path / 'runtime_state.json').read_text())
    assert state['nearest_junction'] == {
        'node_id': 42, 'junction_type': 'T_JUNCTION', 'distance_m': 18.4}
    assert state['latest_match']['matched'] is True


def test_junction_export_uses_lon_lat_and_preserves_properties(tmp_path):
    output = exporter(tmp_path)
    junction = ClassifiedJunction(
        42, 48.1, 9.2, JunctionType.T_JUNCTION, 3,
        (0.0, 90.0, 180.0), (10, 11, 12), True)
    assert output.export_junctions((junction,)) == 1
    feature = json.loads((tmp_path / 'junctions.geojson').read_text())[
        'features'][0]
    assert feature['geometry']['coordinates'] == [9.2, 48.1]
    assert feature['properties'] == {
        'node_id': 42, 'junction_type': 'T_JUNCTION',
        'physical_branch_count': 3,
        'physical_branch_bearings_deg': [0.0, 90.0, 180.0],
        'connected_way_ids': [10, 11, 12],
        'has_traffic_signals': True}


def test_junction_export_handles_missing_optional_properties(tmp_path):
    output = exporter(tmp_path)
    minimal = SimpleNamespace(latitude=48.1, longitude=9.2)
    output.export_junctions((minimal,))
    properties = json.loads((tmp_path / 'junctions.geojson').read_text())[
        'features'][0]['properties']
    assert properties['junction_type'] is None
    assert properties['physical_branch_bearings_deg'] == []


def test_matched_and_unmatched_observations(tmp_path):
    output = exporter(tmp_path)
    output.observe(gnss(), match(), counters(), now_ns=2_000_000_000)
    state = output.runtime_state()
    assert state['latest_gnss']['altitude'] == 123.4
    assert state['latest_match']['distance_m'] == 3.2
    assert state['latest_match']['nearest_longitude'] == 8.02001
    assert state['matched_track'] == [
        {'timestamp_ns': 1, 'latitude': 50.01001,
         'longitude': 8.02001, 'way_id': 1}]
    output.observe(gnss(2), match(2, False), counters(2, 1, 1),
                   now_ns=2_000_000_001)
    state = output.runtime_state()
    assert state['latest_match']['matched'] is False
    assert len(state['gnss_track']) == 2
    assert len(state['matched_track']) == 1


def test_track_limit_and_throttling_preserve_all_bounded_state(tmp_path):
    output = exporter(tmp_path, limit=3)
    for index in range(5):
        output.observe(gnss(index), match(index), counters(index + 1, index + 1),
                       now_ns=100 + index)
    assert [p['timestamp_ns'] for p in output.runtime_state()['gnss_track']] == [2, 3, 4]
    assert [p['timestamp_ns'] for p in output.runtime_state()['matched_track']] == [2, 3, 4]
    assert output.runtime_state()['matched_count'] == 5
    assert output.runtime_write_count == 1  # first write only; later points are throttled


def test_important_match_transitions_write_immediately(tmp_path):
    output = exporter(tmp_path)
    output.observe(gnss(1), match(1), counters(), now_ns=2_000_000_000)
    writes = output.runtime_write_count
    output.observe(gnss(2), match(2, way_id=2), counters(2, 2),
                   now_ns=2_000_000_001)
    assert output.runtime_write_count == writes + 1
    output.observe(gnss(3), match(3, False), counters(3, 2, 1),
                   now_ns=2_000_000_002)
    assert output.runtime_write_count == writes + 2
    output.observe(gnss(4), match(4, False, highway='service'),
                   counters(4, 2, 2), now_ns=2_000_000_003)
    # Unmatched highway values are intentionally null, so identity is unchanged.
    assert output.runtime_write_count == writes + 2
    output.observe(gnss(5), match(5, way_id=2, highway='service'),
                   counters(5, 3, 2), now_ns=2_000_000_004)
    assert output.runtime_write_count == writes + 3


def test_export_does_not_change_matcher_result(tmp_path):
    roads = (way(),)
    before = match_nearest_road(1, 50.01, 8.02, roads, 1000.0)
    exporter(tmp_path).export_highways(roads, ())
    after = match_nearest_road(1, 50.01, 8.02, roads, 1000.0)
    assert after == before


def test_invalid_limits_are_rejected(tmp_path):
    with pytest.raises(ValueError, match='update_interval'):
        VisualizationExporter(str(tmp_path), 0.0, 1)
    with pytest.raises(ValueError, match='max_track_points'):
        VisualizationExporter(str(tmp_path), 1.0, 0)


def test_initialize_copies_assets_and_writes_empty_runtime(tmp_path):
    assets = tmp_path / 'assets'
    assets.mkdir()
    for name in ('index.html', 'viewer.js', 'viewer.css'):
        (assets / name).write_text(name, encoding='utf-8')
    output_dir = tmp_path / 'output'
    output = VisualizationExporter(str(output_dir), 0.5, 5)
    output.initialize(str(assets))
    assert (output_dir / 'viewer.js').read_text() == 'viewer.js'
    assert json.loads((output_dir / 'runtime_state.json').read_text())[
        'gnss_received_count'] == 0
