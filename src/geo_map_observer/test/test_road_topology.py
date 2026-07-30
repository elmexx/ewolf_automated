"""Synthetic tests for ROS-independent drivable-road topology."""

from types import SimpleNamespace

import pytest

from geo_map_observer.osm_loader import OsmHighwayWay, OsmNode, load_osm_map
from geo_map_observer.road_matcher import classify_highway_ways, match_nearest_road
from geo_map_observer.road_topology import (
    build_road_topology, merge_branch_bearings)


def way(way_id, references, highway='residential', junction=None, **tags):
    return OsmHighwayWay(
        way_id, highway, tags.get('name'), tags.get('ref'),
        tags.get('maxspeed'), tags.get('lanes'), tags.get('oneway'),
        tuple(references), (), junction,
        {} if junction is None else {'junction': junction})


def topology(coordinates, ways, tags=None, angle=20.0):
    tags = tags or {}
    nodes = {
        node_id: OsmNode(node_id, latitude, longitude, tags.get(node_id, {}))
        for node_id, (latitude, longitude) in coordinates.items()
    }
    data = SimpleNamespace(nodes=nodes, highway_ways=tuple(ways))
    return build_road_topology(data, ways, angle)


def spokes(bearings, tagged=False):
    # At this scale, planar direction is sufficient to construct test bearings.
    coordinates = {0: (48.0, 9.0)}
    ways = []
    for index, bearing in enumerate(bearings, 1):
        radians = __import__('math').radians(bearing)
        coordinates[index] = (
            48.0 + 0.001 * __import__('math').cos(radians),
            9.0 + 0.0015 * __import__('math').sin(radians))
        ways.append(way(index, (0, index), name='Road {}'.format(index)))
    tags = {0: {'highway': 'traffic_signals'}} if tagged else {}
    return topology(coordinates, ways, tags)


def test_straight_continuation_and_split_ways_are_not_junctions():
    coordinates = {1: (48.0, 8.999), 2: (48.0, 9.0), 3: (48.0, 9.001)}
    single = topology(coordinates, [way(10, (1, 2, 3))])
    split = topology(coordinates, [way(10, (1, 2)), way(20, (2, 3))])

    assert single.junction_candidates == ()
    assert split.junction_candidates == ()
    assert split.nodes[2].connected_way_ids == (10, 20)
    assert len(split.nodes[2].physical_branch_bearings_deg) == 2


@pytest.mark.parametrize('bearings,count', [
    ((0, 90, 180), 3), ((0, 90, 180, 270), 4),
])
def test_three_and_four_way_candidates(bearings, count):
    result = spokes(bearings)

    assert len(result.junction_candidates) == 1
    assert result.junction_candidates[0].physical_branch_count == count


def test_duplicate_and_circular_branches_merge_but_opposites_do_not():
    groups, physical = merge_branch_bearings((90, 98), 20.0)
    circular_groups, circular = merge_branch_bearings((2, 358), 20.0)
    opposite_groups, opposite = merge_branch_bearings((0, 180), 20.0)

    assert len(groups) == len(physical) == 1
    assert len(circular_groups) == len(circular) == 1
    assert circular[0] == pytest.approx(0.0, abs=1e-9)
    assert len(opposite_groups) == len(opposite) == 2


def test_curved_way_has_no_false_junction_at_bends():
    result = topology(
        {1: (48.0, 9.0), 2: (48.0005, 9.0003),
         3: (48.0008, 9.001), 4: (48.001, 9.002)},
        [way(1, (1, 2, 3, 4))])

    assert result.junction_candidates == ()
    assert all(len(node.physical_branch_bearings_deg) <= 2
               for node in result.nodes.values())


def test_zero_length_and_missing_references_are_skipped_and_counted():
    result = topology(
        {1: (48.0, 9.0), 2: (48.0, 9.0), 3: (48.0, 9.001)},
        [way(1, (1, 2, 3, 999))])

    assert result.statistics.zero_length_segment_count == 1
    assert result.statistics.missing_node_reference_count == 1
    assert result.statistics.skipped_invalid_segment_count == 2
    assert result.statistics.segment_count == 1


def test_control_tags_and_traffic_signal_candidate_are_retained():
    result = spokes((0, 90, 180), tagged=True)
    candidate = result.junction_candidates[0]

    assert candidate.has_traffic_signals is True
    assert result.statistics.traffic_signal_candidate_count == 1

    controls = topology(
        {1: (48.0, 9.0), 2: (48.0, 9.001), 3: (48.0, 9.002)},
        [way(1, (1, 2, 3))],
        {1: {'highway': 'stop'}, 3: {'highway': 'give_way'}})
    assert controls.nodes[1].tags['highway'] == 'stop'
    assert controls.nodes[3].tags['highway'] == 'give_way'
    assert controls.statistics.stop_node_count == 1
    assert controls.statistics.give_way_node_count == 1


def test_roundabout_and_circular_way_ids_are_retained():
    result = topology(
        {1: (48.0, 9.0), 2: (48.0, 9.001), 3: (48.001, 9.001)},
        [way(5, (1, 2), junction='roundabout'),
         way(6, (2, 3), junction='circular')])

    assert result.statistics.roundabout_way_ids == (5,)
    assert result.statistics.circular_junction_way_ids == (6,)


def test_loader_retains_relevant_node_tags_only(tmp_path):
    osm = tmp_path / 'tags.osm'
    osm.write_text('''<osm>
      <node id="1" lat="48" lon="9"><tag k="highway" v="stop"/><tag k="note" v="omit"/></node>
      <node id="2" lat="48" lon="9.001"><tag k="crossing" v="marked"/></node>
      <way id="1"><nd ref="1"/><nd ref="2"/>
        <tag k="highway" v="residential"/>
        <tag k="junction" v="roundabout"/>
      </way>
    </osm>''', encoding='utf-8')
    data = load_osm_map(str(osm))

    assert data.nodes[1].tags == {'highway': 'stop'}
    assert data.nodes[2].tags == {'crossing': 'marked'}
    assert data.highway_ways[0].junction == 'roundabout'


def test_contextual_ways_and_task_four_matching_survive_topology(tmp_path):
    osm = tmp_path / 'mixed.osm'
    osm.write_text('''<osm>
      <node id="1" lat="48" lon="9"/><node id="2" lat="48" lon="9.001"/>
      <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="footway"/></way>
      <way id="2"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
    </osm>''', encoding='utf-8')
    data = load_osm_map(str(osm))
    before = data.highway_ways
    drivable, contextual = classify_highway_ways(data)
    build_road_topology(data, drivable)
    match = match_nearest_road(1, 48.0, 9.0005, drivable, 20.0)

    assert data.highway_ways is before
    assert contextual[0].highway == 'footway'
    assert match.matched and match.osm_way_id == 2


@pytest.mark.parametrize('angle', [0, 90, -1, 180])
def test_invalid_merge_angle_is_rejected(angle):
    with pytest.raises(ValueError, match='greater than 0 and less than 90'):
        merge_branch_bearings((0, 1), angle)
