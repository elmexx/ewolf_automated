"""ROS-independent geometry calculations for LaneDet observations."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class LaneGeometry:
    lane_width: Optional[float]
    curvature: Optional[float]
    radius: Optional[float]
    direction: str


def _as_points(points: Sequence[Sequence[float]]) -> List[Tuple[float, float]]:
    result = []
    for point in points:
        if len(point) >= 2 and math.isfinite(point[0]) and math.isfinite(point[1]):
            result.append((float(point[0]), float(point[1])))
    return sorted(result)


def _interpolate(points, x):
    for first, second in zip(points, points[1:]):
        if first[0] <= x <= second[0]:
            if second[0] == first[0]:
                return (first[1] + second[1]) / 2.0
            ratio = (x - first[0]) / (second[0] - first[0])
            return first[1] + ratio * (second[1] - first[1])
    return points[-1][1]


def _shared_samples(left, right, count=20):
    start, end = max(left[0][0], right[0][0]), min(left[-1][0], right[-1][0])
    if end <= start:
        return []
    return [start + (end - start) * index / (count - 1) for index in range(count)]


def lane_width(left_points: Sequence[Sequence[float]],
               right_points: Sequence[Sequence[float]]) -> Optional[float]:
    """Return median lateral separation over the shared longitudinal range."""
    left, right = _as_points(left_points), _as_points(right_points)
    if len(left) < 2 or len(right) < 2:
        return None
    widths = [_interpolate(left, x) - _interpolate(right, x)
              for x in _shared_samples(left, right)]
    if not widths:
        return None
    widths.sort()
    middle = len(widths) // 2
    width = ((widths[middle - 1] + widths[middle]) / 2.0
             if len(widths) % 2 == 0 else widths[middle])
    return width if width > 0.0 else None


def _quadratic_fit(xs, ys):
    # Solve the 3x3 normal equation for y=a*x^2+b*x+c.
    sums = [sum(x ** power for x in xs) for power in range(5)]
    matrix = [[sums[4], sums[3], sums[2], sum(y * x * x for x, y in zip(xs, ys))],
              [sums[3], sums[2], sums[1], sum(y * x for x, y in zip(xs, ys))],
              [sums[2], sums[1], sums[0], sum(ys)]]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(matrix[row][column]))
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        if abs(matrix[column][column]) < 1e-12:
            return None
        divisor = matrix[column][column]
        matrix[column] = [value / divisor for value in matrix[column]]
        for row in range(3):
            if row != column:
                factor = matrix[row][column]
                matrix[row] = [value - factor * base
                               for value, base in zip(matrix[row], matrix[column])]
    return matrix[0][3], matrix[1][3], matrix[2][3]


def centerline_curvature(left_points: Sequence[Sequence[float]],
                         right_points: Sequence[Sequence[float]],
                         straight_threshold: float = 1e-4
                         ) -> Tuple[Optional[float], Optional[float], str]:
    left, right = _as_points(left_points), _as_points(right_points)
    if len(left) < 3 or len(right) < 3:
        return None, None, "unknown"
    xs = _shared_samples(left, right)
    if not xs:
        return None, None, "unknown"
    ys = [(_interpolate(left, x) + _interpolate(right, x)) / 2.0 for x in xs]
    coefficients = _quadratic_fit(xs, ys)
    if coefficients is None:
        return None, None, "unknown"
    a, b, _ = coefficients
    evaluation_x = (xs[0] + xs[-1]) / 2.0
    curvature = (2.0 * a) / ((1.0 + (2.0 * a * evaluation_x + b) ** 2) ** 1.5)
    if abs(curvature) < straight_threshold:
        return curvature, None, "straight"
    return curvature, abs(1.0 / curvature), "left" if curvature > 0 else "right"


def calculate_ego_geometry(left_points, right_points,
                           left_available: bool,
                           right_available: bool) -> LaneGeometry:
    if not (left_available and right_available):
        return LaneGeometry(None, None, None, "unknown")
    width = lane_width(left_points, right_points)
    curvature, radius, direction = centerline_curvature(left_points, right_points)
    return LaneGeometry(width, curvature, radius, direction)
