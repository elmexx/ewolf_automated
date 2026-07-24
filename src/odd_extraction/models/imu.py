"""ROS-independent records for custom IMU messages."""

from dataclasses import dataclass

from odd_extraction.models.common import MessageMetadata


@dataclass(frozen=True)
class ImuStatusRecord:
    """Internal record preserving sensor_driver_msgs/msg/ImuStatus fields."""

    metadata: MessageMetadata
    mask: int
    accel_valid: bool
    ypr_valid: bool
    mag_valid: bool
    gyro_valid: bool
