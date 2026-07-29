"""Tests for ROS-independent GNSS fix validation."""

from types import SimpleNamespace

import pytest

from geo_map_observer.validation import extract_nav_sat_fix, validate_nav_sat_fix


def nav_sat_fix(status=0, latitude=48.123, longitude=9.123,
                altitude=420.3):
    """Construct a synthetic NavSatFix-shaped message."""
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=345),
            frame_id='gnss_link',
        ),
        status=SimpleNamespace(status=status),
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        position_covariance=[float(value) for value in range(9)],
        position_covariance_type=2,
    )


@pytest.mark.parametrize('altitude', [420.3, float('nan')])
def test_valid_fix_accepts_known_or_unknown_altitude(altitude):
    assert validate_nav_sat_fix(nav_sat_fix(altitude=altitude)) == (True, 'valid')


def test_status_no_fix_is_rejected():
    assert validate_nav_sat_fix(nav_sat_fix(status=-1)) == (False, 'no_fix')


def test_required_nav_sat_fix_fields_are_extracted():
    position = extract_nav_sat_fix(nav_sat_fix())

    assert position.timestamp_ns == 12_000_000_345
    assert position.frame_id == 'gnss_link'
    assert position.status == 0
    assert position.latitude == 48.123
    assert position.longitude == 9.123
    assert position.altitude == 420.3
    assert position.position_covariance == tuple(float(value) for value in range(9))
    assert position.position_covariance_type == 2


@pytest.mark.parametrize(
    'latitude,longitude,reason',
    [
        (float('nan'), 9.0, 'non_finite_coordinates'),
        (48.0, float('inf'), 'non_finite_coordinates'),
        (-float('inf'), 9.0, 'non_finite_coordinates'),
        (90.000001, 9.0, 'latitude_out_of_range'),
        (-90.000001, 9.0, 'latitude_out_of_range'),
        (48.0, 180.000001, 'longitude_out_of_range'),
        (48.0, -180.000001, 'longitude_out_of_range'),
    ],
)
def test_invalid_coordinates_are_rejected(latitude, longitude, reason):
    assert validate_nav_sat_fix(nav_sat_fix(
        latitude=latitude, longitude=longitude)) == (False, reason)


@pytest.mark.parametrize(
    'latitude,longitude',
    [(-90.0, -180.0), (90.0, 180.0), (0.0, 0.0)],
)
def test_coordinate_boundaries_are_valid(latitude, longitude):
    assert validate_nav_sat_fix(nav_sat_fix(
        latitude=latitude, longitude=longitude)) == (True, 'valid')
