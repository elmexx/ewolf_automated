"""Tests for ROS-independent drivable-road classification and matching."""

import pytest

from geo_map_observer.osm_loader import OsmHighwayWay, load_osm_map
from geo_map_observer.road_matcher import (
    DEFAULT_DRIVABLE_HIGHWAY_TYPES,
    RoadMatchLogThrottle,
    candidate_statistics,
    classify_highway_ways,
    match_nearest_road,
    parse_maxspeed_kmh,
)


def way(way_id, highway, coordinates, **tags):
    return OsmHighwayWay(
        way_id=way_id,
        highway=highway,
        name=tags.get('name'),
        ref=tags.get('ref'),
        maxspeed=tags.get('maxspeed'),
        lanes=tags.get('lanes'),
        oneway=tags.get('oneway'),
        node_references=tuple(range(len(coordinates))),
        coordinates=tuple(coordinates),
    )


def test_all_highways_remain_available_while_footway_is_contextual(tmp_path):
    osm = tmp_path / 'mixed.osm'
    osm.write_text('''<osm>
      <node id="1" lat="48.0" lon="9.0"/>
      <node id="2" lat="48.0" lon="9.001"/>
      <way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="footway"/></way>
      <way id="20"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
    </osm>''', encoding='utf-8')
    data = load_osm_map(str(osm))
    drivable, contextual = classify_highway_ways(data)

    assert [item.way_id for item in data.highway_ways] == [10, 20]
    assert [item.way_id for item in drivable] == [20]
    assert [item.way_id for item in contextual] == [10]
    stats = candidate_statistics(data)
    assert (stats.all_way_count, stats.drivable_way_count,
            stats.contextual_way_count) == (2, 1, 1)
    assert stats.drivable_type_counts == {'residential': 1}
    assert stats.contextual_type_counts == {'footway': 1}


def test_nearby_footway_does_not_override_residential_candidate():
    footway = way(1, 'footway', ((48.0, 9.0), (48.0, 9.001)))
    residential = way(
        2, 'residential', ((48.00005, 9.0), (48.00005, 9.001)),
        name='Main Street', ref='L 1', maxspeed='50', lanes='2', oneway='yes')
    candidates = tuple(item for item in (footway, residential)
                       if item.highway in DEFAULT_DRIVABLE_HIGHWAY_TYPES)

    result = match_nearest_road(123, 48.00001, 9.0005, candidates, 20.0)

    assert result.matched is True
    assert result.osm_way_id == 2
    assert result.distance_m == pytest.approx(4.45, abs=0.1)
    assert result.nearest_latitude == pytest.approx(48.00005)
    assert result.nearest_longitude == pytest.approx(9.0005)


@pytest.mark.parametrize('highway', ['track', 'path'])
def test_custom_types_can_enable_conservative_highways(highway):
    candidate = way(7, highway, ((48.0, 9.0), (48.0, 9.001)))
    # A map-like object is sufficient because classification is ROS independent.
    map_data = type('Map', (), {'highway_ways': (candidate,)})()

    default, _ = classify_highway_ways(map_data)
    custom, _ = classify_highway_ways(map_data, (highway,))

    assert default == ()
    assert custom == (candidate,)


def test_nearest_drivable_match_preserves_all_road_attributes():
    farther = way(1, 'primary', ((48.001, 9.0), (48.001, 9.001)))
    nearest = way(
        2, 'service', ((48.0, 9.0), (48.0, 9.001)),
        name='Depot Access', ref='A7', maxspeed='30 mph',
        lanes='1;2', oneway='-1')

    result = match_nearest_road(9_000_000_042, 48.00001, 9.0005,
                                (farther, nearest), 20.0)

    assert result.timestamp_ns == 9_000_000_042
    assert result.latitude == 48.00001
    assert result.longitude == 9.0005
    assert result.osm_way_id == 2
    assert result.highway == 'service'
    assert result.name == 'Depot Access'
    assert result.ref == 'A7'
    assert result.maxspeed_raw == '30 mph'
    assert result.maxspeed_kmh == pytest.approx(48.28032)
    assert result.lanes_raw == '1;2'
    assert result.oneway_raw == '-1'


def test_outside_threshold_is_unmatched_but_reports_nearest_distance():
    candidate = way(1, 'residential', ((48.0, 9.0), (48.0, 9.001)))

    result = match_nearest_road(1, 48.001, 9.0005, (candidate,), 20.0)

    assert result.matched is False
    assert result.osm_way_id is None
    assert result.distance_m == pytest.approx(111.2, abs=0.2)
    assert result.nearest_latitude == pytest.approx(48.0)


def test_no_candidates_returns_unmatched_without_fabricated_distance():
    result = match_nearest_road(1, 48.0, 9.0, (), 20.0)

    assert result.matched is False
    assert result.distance_m is None
    assert result.nearest_latitude is None


def test_burst_matching_is_not_suppressed_by_repeated_road_log_throttle():
    candidate = way(1, 'residential', ((48.0, 9.0), (48.0, 9.001)),
                    maxspeed='50')
    throttle = RoadMatchLogThrottle(1.0)

    results = [match_nearest_road(index, 48.00001, 9.0005,
                                  (candidate,), 20.0)
               for index in range(5)]
    log_decisions = [throttle.should_log(result, index) for index, result
                     in enumerate(results)]

    assert len(results) == 5
    assert all(result.matched for result in results)
    assert log_decisions == [True, False, False, False, False]


def test_way_and_maxspeed_changes_bypass_log_interval():
    first = way(1, 'residential', ((48.0, 9.0), (48.0, 9.001)),
                maxspeed='30')
    changed_way = way(2, 'residential', first.coordinates, maxspeed='30')
    changed_speed = way(2, 'residential', first.coordinates, maxspeed='50')
    throttle = RoadMatchLogThrottle(10.0)

    first_result = match_nearest_road(1, 48.00001, 9.0005, (first,), 20.0)
    repeat_result = match_nearest_road(2, 48.00001, 9.0005, (first,), 20.0)
    way_result = match_nearest_road(3, 48.00001, 9.0005,
                                    (changed_way,), 20.0)
    speed_result = match_nearest_road(4, 48.00001, 9.0005,
                                      (changed_speed,), 20.0)

    assert throttle.should_log(first_result, 0) is True
    assert throttle.should_log(repeat_result, 1) is False
    assert throttle.should_log(way_result, 2) is True
    assert throttle.should_log(speed_result, 3) is True


@pytest.mark.parametrize('raw,expected', [
    ('80', 80.0), ('50 km/h', 50.0), ('30 mph', 48.28032),
    ('DE:urban', None), (None, None),
])
def test_maxspeed_parsing_preserves_unknown_semantics(raw, expected):
    parsed = parse_maxspeed_kmh(raw)
    if expected is None:
        assert parsed is None
    else:
        assert parsed == pytest.approx(expected)
