"""ROS-independent records for custom GNSS messages."""

from dataclasses import dataclass

from odd_extraction.models.common import MessageMetadata, Vector3


@dataclass(frozen=True)
class GnssEcefRecord:
    """Internal record preserving sensor_driver_msgs/msg/GnssEcef fields."""

    metadata: MessageMetadata
    station: int
    pos: Vector3
    velocity: Vector3
    pos_error: float
    speed_error: float


@dataclass(frozen=True)
class GnssNedRecord:
    """Internal record preserving sensor_driver_msgs/msg/GnssNed fields."""

    metadata: MessageMetadata
    rel_pos_length: float
    rel_pos_heading: float
    rel_pos: Vector3
    rel_speed: Vector3


@dataclass(frozen=True)
class GnssQualityRecord:
    """Internal record preserving sensor_driver_msgs/msg/GnssQuality fields."""

    metadata: MessageMetadata
    time_error: float
    latitude_error: float
    longitude_error: float
    altitude_error: float
    speed_error: float
    climb_error: float
    pos2d_error: float
    pos3d_error: float
    xdop: float
    ydop: float
    pdop: float
    hdop: float
    vdop: float
    tdop: float
    gdop: float


@dataclass(frozen=True)
class GnssStatusRecord:
    """Internal record preserving sensor_driver_msgs/msg/GnssStatus fields."""

    metadata: MessageMetadata
    sensor_time: float
    online: float
    mask: int
    satellites_used: int
    satellites_visible: int
    dgps_station: int
    dgps_age: float
