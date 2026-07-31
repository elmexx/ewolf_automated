"""ROS-independent tests for nearest classified junction lookup."""

from types import SimpleNamespace

import pytest

from geo_map_observer.junction_lookup import JunctionLookup


def junction(node_id, latitude, longitude, junction_type='T_JUNCTION'):
    return SimpleNamespace(
        node_id=node_id, latitude=latitude, longitude=longitude,
        junction_type=junction_type)


def test_empty_junction_list_returns_none():
    assert JunctionLookup(()).nearest(50.0, 8.0) is None


def test_single_junction_returns_correct_distance():
    result = JunctionLookup((junction(1, 50.001, 8.0),)).nearest(50.0, 8.0)

    assert result.node_id == 1
    assert result.junction_type == 'T_JUNCTION'
    assert result.distance_m == pytest.approx(111.2, abs=0.2)


def test_multiple_junctions_select_nearest():
    lookup = JunctionLookup((
        junction(10, 50.01, 8.0, 'CROSS_INTERSECTION'),
        junction(20, 50.0001, 8.0, 'Y_JUNCTION'),
        junction(30, 49.99, 8.0, 'MULTI_WAY')))

    result = lookup.nearest(50.0, 8.0)

    assert result.node_id == 20
    assert result.junction_type == 'Y_JUNCTION'


def test_equal_distance_uses_lowest_node_id_independent_of_input_order():
    high = junction(200, 50.001, 8.0)
    low = junction(100, 49.999, 8.0)

    assert JunctionLookup((high, low)).nearest(50.0, 8.0).node_id == 100
    assert JunctionLookup((low, high)).nearest(50.0, 8.0).node_id == 100


@pytest.mark.parametrize('latitude,longitude', [
    (float('nan'), 8.0), (50.0, float('inf')),
])
def test_invalid_position_is_rejected_before_search(latitude, longitude):
    with pytest.raises(ValueError, match='must be finite'):
        JunctionLookup((junction(1, 50.0, 8.0),)).nearest(
            latitude, longitude)
