"""ROS-independent validation for NavSatFix-like messages."""

import math
from dataclasses import dataclass
from typing import Tuple


STATUS_NO_FIX = -1


@dataclass(frozen=True)
class GnssPosition:
    """Fields extracted from a NavSatFix message."""

    timestamp_ns: int
    frame_id: str
    status: int
    latitude: float
    longitude: float
    altitude: float
    position_covariance: Tuple[float, ...]
    position_covariance_type: int


def extract_nav_sat_fix(message: object) -> GnssPosition:
    """Extract the required fields from a NavSatFix-like message."""
    return GnssPosition(
        timestamp_ns=(message.header.stamp.sec * 1_000_000_000 +
                      message.header.stamp.nanosec),
        frame_id=message.header.frame_id,
        status=message.status.status,
        latitude=message.latitude,
        longitude=message.longitude,
        altitude=message.altitude,
        position_covariance=tuple(message.position_covariance),
        position_covariance_type=message.position_covariance_type,
    )


def validate_nav_sat_fix(message: object) -> Tuple[bool, str]:
    """Validate the status and coordinates of a NavSatFix-like message.

    Altitude is deliberately not validated because NavSatFix permits an
    unknown altitude to be represented by NaN.
    """
    if message.status.status == STATUS_NO_FIX:
        return False, 'no_fix'

    latitude = message.latitude
    longitude = message.longitude
    if not math.isfinite(latitude) or not math.isfinite(longitude):
        return False, 'non_finite_coordinates'
    if not -90.0 <= latitude <= 90.0:
        return False, 'latitude_out_of_range'
    if not -180.0 <= longitude <= 180.0:
        return False, 'longitude_out_of_range'
    return True, 'valid'
