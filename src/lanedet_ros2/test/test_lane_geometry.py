from lanedet_ros2.lane_geometry import calculate_ego_geometry, lane_width
from lanedet_ros2.lane_observation import LaneObservation, make_lanes


class FakeArray(list):
    @property
    def size(self):
        return len(self)


def test_width_and_straight_centerline():
    left = FakeArray([[0.0, 2.0], [5.0, 2.0], [10.0, 2.0]])
    right = FakeArray([[0.0, -2.0], [5.0, -2.0], [10.0, -2.0]])
    geometry = calculate_ego_geometry(left, right, True, True)
    assert lane_width(left, right) == 4.0
    assert geometry.lane_width == 4.0
    assert abs(geometry.curvature) < 1e-10
    assert geometry.radius is None
    assert geometry.direction == "straight"


def test_missing_real_boundary_makes_geometry_invalid():
    points = FakeArray([[0.0, 2.0], [5.0, 2.0], [10.0, 2.0]])
    geometry = calculate_ego_geometry(points, points, True, False)
    assert geometry.lane_width is None
    assert geometry.curvature is None
    assert geometry.direction == "unknown"


def test_observation_preserves_projected_points_and_no_detection_status():
    lane = FakeArray([[1.0, 0.5], [2.0, 0.75]])
    lanes = make_lanes([lane], lane, FakeArray())
    assert lanes[0].role == "ego_left"
    assert lanes[0].confidence is None
    assert lanes[0].points[1].longitudinal == 2.0
    assert lanes[0].points[1].lateral == 0.75

    observation = LaneObservation(
        1_000_000_002, "base_link", "not_detected", 0, False, False,
        [], None, None, None, "unknown", 4.2)
    payload = observation.to_dict()
    assert payload["header"]["stamp"] == {"sec": 1, "nanosec": 2}
    assert payload["lane_marking_count"] == 0
    assert 'NaN' not in observation.to_json()
