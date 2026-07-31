"""ROS-independent tests for static junction classification."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from geo_map_observer.junction_classifier import (
    JunctionType, circular_angle_difference_deg,
    classify_junction_candidate, classify_junction_candidates)
from geo_map_observer.road_matcher import RoadMatch


def candidate(bearings):
    return SimpleNamespace(
        node_id=42, latitude=48.1, longitude=9.2,
        physical_branch_count=len(bearings),
        physical_branch_bearings_deg=tuple(bearings),
        connected_way_ids=(10, 11, 12), has_traffic_signals=True)


def kind(bearings):
    return classify_junction_candidate(candidate(bearings)).junction_type


def test_circular_angle_boundary():
    assert circular_angle_difference_deg(2, 358) == pytest.approx(4)
    assert circular_angle_difference_deg(10, 190) == pytest.approx(180)


@pytest.mark.parametrize('bearings,expected', [
    ((0, 90, 180), JunctionType.T_JUNCTION),
    ((5, 100, 185), JunctionType.T_JUNCTION),
    ((30, 150, 270), JunctionType.Y_JUNCTION),
    ((0, 90, 180, 270), JunctionType.CROSS_INTERSECTION),
    ((10, 75, 190, 255), JunctionType.CROSS_INTERSECTION),
    ((0, 50, 100, 150), JunctionType.FOUR_WAY_UNKNOWN),
    ((0, 60, 120, 200, 280), JunctionType.MULTI_WAY),
])
def test_classification_rules(bearings, expected):
    assert kind(bearings) is expected


def test_order_independent_and_does_not_mutate_candidate():
    original = candidate((5, 100, 185))
    before = deepcopy(original)
    forward = classify_junction_candidate(original)
    reverse = classify_junction_candidate(candidate((185, 5, 100)))
    assert forward.junction_type is reverse.junction_type
    assert original == before


def test_invalid_three_way_is_unknown_and_fewer_than_three_is_skipped():
    assert kind((0, 0, 180)) is JunctionType.THREE_WAY_UNKNOWN
    assert classify_junction_candidate(candidate((0, 180))) is None
    assert classify_junction_candidates((candidate((0, 180)),)) == ()


@pytest.mark.parametrize('tolerance', [0, 90, -1])
def test_invalid_tolerance(tolerance):
    with pytest.raises(ValueError, match='greater than 0 and less than 90'):
        classify_junction_candidate(candidate((0, 90, 180)), tolerance)


def test_classification_does_not_change_road_match_value():
    road_match = RoadMatch(1, 48.0, 9.0, False, None, None, None, None,
                           None, None, None, None, None, None, None)
    before = deepcopy(road_match)
    classify_junction_candidate(candidate((0, 90, 180)))
    assert road_match == before
