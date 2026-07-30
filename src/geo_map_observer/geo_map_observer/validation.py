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
    navsat_status: int
    status_valid: bool
    latitude: float
    longitude: float
    altitude: float
    position_covariance: Tuple[float, ...]
    position_covariance_type: int

    @property
    def status(self) -> int:
        """Return the raw status under the record's original attribute name."""
        return self.navsat_status


def extract_nav_sat_fix(message: object) -> GnssPosition:
    """Extract the required fields from a NavSatFix-like message."""
    return GnssPosition(
        timestamp_ns=(message.header.stamp.sec * 1_000_000_000 +
                      message.header.stamp.nanosec),
        frame_id=message.header.frame_id,
        navsat_status=message.status.status,
        status_valid=message.status.status != STATUS_NO_FIX,
        latitude=message.latitude,
        longitude=message.longitude,
        altitude=message.altitude,
        position_covariance=tuple(message.position_covariance),
        position_covariance_type=message.position_covariance_type,
    )


def validate_nav_sat_fix(message: object,
                         require_fix_status: bool = False) -> Tuple[bool, str]:
    """Validate the coordinates and, when requested, the fix status.

    Altitude is deliberately not validated because NavSatFix permits an
    unknown altitude to be represented by NaN. Coordinate validity always
    takes precedence over status validation.
    """
    latitude = message.latitude
    longitude = message.longitude
    if not math.isfinite(latitude) or not math.isfinite(longitude):
        return False, 'non_finite_coordinates'
    if not -90.0 <= latitude <= 90.0:
        return False, 'latitude_out_of_range'
    if not -180.0 <= longitude <= 180.0:
        return False, 'longitude_out_of_range'
    if require_fix_status and message.status.status == STATUS_NO_FIX:
        return False, 'no_fix'
    if message.status.status == STATUS_NO_FIX:
        return True, 'no_fix_accepted'
    return True, 'valid'
