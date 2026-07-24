"""Duck-typed adapters from ROS-like messages to internal records.

This module intentionally does not import generated ROS message classes so it can
be tested without ROS 2 installed. Only required attributes are accessed.
"""

from odd_extraction.models.common import MessageMetadata, Timestamp, Vector3
from odd_extraction.models.gnss import (
    GnssEcefRecord,
    GnssNedRecord,
    GnssQualityRecord,
    GnssStatusRecord,
)
from odd_extraction.models.imu import ImuStatusRecord

_TIMESTAMP_SCALE = 1_000_000_000


class MissingRosFieldError(AttributeError):
    """Raised when a ROS-like message lacks a required field."""


def _get_attr(message_type: str, adapter: str, obj: object, path: str) -> object:
    current = obj
    for part in path.split("."):
        try:
            current = getattr(current, part)
        except AttributeError:
            raise MissingRosFieldError(
                f"{adapter} expected {message_type} field '{path}' but missing '{part}'"
            ) from None
    return current


def _metadata(message_type: str, adapter: str, message: object) -> MessageMetadata:
    sec = _get_attr(message_type, adapter, message, "header.stamp.sec")
    nanosec = _get_attr(message_type, adapter, message, "header.stamp.nanosec")
    frame_id = _get_attr(message_type, adapter, message, "header.frame_id")
    return MessageMetadata(
        timestamp=Timestamp(int(sec) * _TIMESTAMP_SCALE + int(nanosec)),
        frame_id=str(frame_id),
    )


def _vector3(message_type: str, adapter: str, message: object, path: str) -> Vector3:
    return Vector3(
        x=float(_get_attr(message_type, adapter, message, f"{path}.x")),
        y=float(_get_attr(message_type, adapter, message, f"{path}.y")),
        z=float(_get_attr(message_type, adapter, message, f"{path}.z")),
    )


def gnss_ecef_from_ros(message: object) -> GnssEcefRecord:
    """Convert a sensor_driver_msgs/msg/GnssEcef-like object."""

    message_type = "sensor_driver_msgs/msg/GnssEcef"
    adapter = "gnss_ecef_from_ros"
    return GnssEcefRecord(
        metadata=_metadata(message_type, adapter, message),
        station=int(_get_attr(message_type, adapter, message, "station")),
        pos=_vector3(message_type, adapter, message, "pos"),
        velocity=_vector3(message_type, adapter, message, "velocity"),
        pos_error=float(_get_attr(message_type, adapter, message, "pos_error")),
        speed_error=float(_get_attr(message_type, adapter, message, "speed_error")),
    )


def gnss_ned_from_ros(message: object) -> GnssNedRecord:
    """Convert a sensor_driver_msgs/msg/GnssNed-like object."""

    message_type = "sensor_driver_msgs/msg/GnssNed"
    adapter = "gnss_ned_from_ros"
    return GnssNedRecord(
        metadata=_metadata(message_type, adapter, message),
        rel_pos_length=float(_get_attr(message_type, adapter, message, "rel_pos_length")),
        rel_pos_heading=float(_get_attr(message_type, adapter, message, "rel_pos_heading")),
        rel_pos=_vector3(message_type, adapter, message, "rel_pos"),
        rel_speed=_vector3(message_type, adapter, message, "rel_speed"),
    )


def gnss_quality_from_ros(message: object) -> GnssQualityRecord:
    """Convert a sensor_driver_msgs/msg/GnssQuality-like object."""

    message_type = "sensor_driver_msgs/msg/GnssQuality"
    adapter = "gnss_quality_from_ros"
    return GnssQualityRecord(
        metadata=_metadata(message_type, adapter, message),
        time_error=float(_get_attr(message_type, adapter, message, "time_error")),
        latitude_error=float(_get_attr(message_type, adapter, message, "latitude_error")),
        longitude_error=float(_get_attr(message_type, adapter, message, "longitude_error")),
        altitude_error=float(_get_attr(message_type, adapter, message, "altitude_error")),
        speed_error=float(_get_attr(message_type, adapter, message, "speed_error")),
        climb_error=float(_get_attr(message_type, adapter, message, "climb_error")),
        pos2d_error=float(_get_attr(message_type, adapter, message, "pos2d_error")),
        pos3d_error=float(_get_attr(message_type, adapter, message, "pos3d_error")),
        xdop=float(_get_attr(message_type, adapter, message, "xdop")),
        ydop=float(_get_attr(message_type, adapter, message, "ydop")),
        pdop=float(_get_attr(message_type, adapter, message, "pdop")),
        hdop=float(_get_attr(message_type, adapter, message, "hdop")),
        vdop=float(_get_attr(message_type, adapter, message, "vdop")),
        tdop=float(_get_attr(message_type, adapter, message, "tdop")),
        gdop=float(_get_attr(message_type, adapter, message, "gdop")),
    )


def gnss_status_from_ros(message: object) -> GnssStatusRecord:
    """Convert a sensor_driver_msgs/msg/GnssStatus-like object."""

    message_type = "sensor_driver_msgs/msg/GnssStatus"
    adapter = "gnss_status_from_ros"
    return GnssStatusRecord(
        metadata=_metadata(message_type, adapter, message),
        sensor_time=float(_get_attr(message_type, adapter, message, "sensor_time")),
        online=float(_get_attr(message_type, adapter, message, "online")),
        mask=int(_get_attr(message_type, adapter, message, "mask")),
        satellites_used=int(_get_attr(message_type, adapter, message, "satellites_used")),
        satellites_visible=int(_get_attr(message_type, adapter, message, "satellites_visible")),
        dgps_station=int(_get_attr(message_type, adapter, message, "dgps_station")),
        dgps_age=float(_get_attr(message_type, adapter, message, "dgps_age")),
    )


def imu_status_from_ros(message: object) -> ImuStatusRecord:
    """Convert a sensor_driver_msgs/msg/ImuStatus-like object."""

    message_type = "sensor_driver_msgs/msg/ImuStatus"
    adapter = "imu_status_from_ros"
    return ImuStatusRecord(
        metadata=_metadata(message_type, adapter, message),
        mask=int(_get_attr(message_type, adapter, message, "mask")),
        accel_valid=bool(_get_attr(message_type, adapter, message, "accel_valid")),
        ypr_valid=bool(_get_attr(message_type, adapter, message, "ypr_valid")),
        mag_valid=bool(_get_attr(message_type, adapter, message, "mag_valid")),
        gyro_valid=bool(_get_attr(message_type, adapter, message, "gyro_valid")),
    )
