import sys
from types import SimpleNamespace

import pytest

from odd_extraction.adapters import (
    MissingRosFieldError,
    gnss_ecef_from_ros,
    gnss_ned_from_ros,
    gnss_quality_from_ros,
    gnss_status_from_ros,
    imu_status_from_ros,
)
from odd_extraction.models import Vector3


def _header(sec=7, nanosec=89, frame_id="frame"):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )


def _vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def test_timestamp_conversion_from_ros_header():
    message = SimpleNamespace(
        header=_header(sec=12, nanosec=345),
        mask=1,
        accel_valid=True,
        ypr_valid=True,
        mag_valid=False,
        gyro_valid=True,
    )

    record = imu_status_from_ros(message)

    assert record.metadata.timestamp.nanoseconds == 12_000_000_345
    assert record.metadata.frame_id == "frame"


def test_gnss_ecef_adapter_preserves_every_field():
    message = SimpleNamespace(
        header=_header(),
        station=5,
        pos=_vector(1.1, 2.2, 3.3),
        velocity=_vector(4.4, 5.5, 6.6),
        pos_error=7.7,
        speed_error=8.8,
    )

    record = gnss_ecef_from_ros(message)

    assert record.station == 5
    assert record.pos == Vector3(1.1, 2.2, 3.3)
    assert record.velocity == Vector3(4.4, 5.5, 6.6)
    assert record.pos_error == 7.7
    assert record.speed_error == 8.8


def test_gnss_ned_adapter_preserves_every_field_and_vectors():
    message = SimpleNamespace(
        header=_header(),
        rel_pos_length=1.2,
        rel_pos_heading=3.4,
        rel_pos=_vector(5.6, 7.8, 9.0),
        rel_speed=_vector(1.1, 2.2, 3.3),
    )

    record = gnss_ned_from_ros(message)

    assert record.rel_pos_length == 1.2
    assert record.rel_pos_heading == 3.4
    assert record.rel_pos == Vector3(5.6, 7.8, 9.0)
    assert record.rel_speed == Vector3(1.1, 2.2, 3.3)


def test_gnss_quality_adapter_preserves_every_field():
    message = SimpleNamespace(
        header=_header(),
        time_error=1,
        latitude_error=2,
        longitude_error=3,
        altitude_error=4,
        speed_error=5,
        climb_error=6,
        pos2d_error=7,
        pos3d_error=8,
        xdop=9,
        ydop=10,
        pdop=11,
        hdop=12,
        vdop=13,
        tdop=14,
        gdop=15,
    )

    record = gnss_quality_from_ros(message)

    assert record.time_error == 1.0
    assert record.latitude_error == 2.0
    assert record.longitude_error == 3.0
    assert record.altitude_error == 4.0
    assert record.speed_error == 5.0
    assert record.climb_error == 6.0
    assert record.pos2d_error == 7.0
    assert record.pos3d_error == 8.0
    assert record.xdop == 9.0
    assert record.ydop == 10.0
    assert record.pdop == 11.0
    assert record.hdop == 12.0
    assert record.vdop == 13.0
    assert record.tdop == 14.0
    assert record.gdop == 15.0


def test_gnss_status_adapter_preserves_raw_mask_and_status_values():
    message = SimpleNamespace(
        header=_header(),
        sensor_time=99.5,
        online=0.0,
        mask=65535,
        satellites_used=10,
        satellites_visible=20,
        dgps_station=30,
        dgps_age=40.5,
    )

    record = gnss_status_from_ros(message)

    assert record.sensor_time == 99.5
    assert record.online == 0.0
    assert record.mask == 65535
    assert record.satellites_used == 10
    assert record.satellites_visible == 20
    assert record.dgps_station == 30
    assert record.dgps_age == 40.5


def test_imu_status_adapter_preserves_boolean_validity_flags_and_mask():
    message = SimpleNamespace(
        header=_header(),
        mask=3,
        accel_valid=True,
        ypr_valid=False,
        mag_valid=False,
        gyro_valid=True,
    )

    record = imu_status_from_ros(message)

    assert record.mask == 3
    assert record.accel_valid is True
    assert record.ypr_valid is False
    assert record.mag_valid is False
    assert record.gyro_valid is True


def test_missing_required_field_error_names_type_field_and_adapter():
    message = SimpleNamespace(
        header=_header(),
        station=5,
        velocity=_vector(1, 2, 3),
        pos_error=1.0,
        speed_error=2.0,
    )

    with pytest.raises(MissingRosFieldError) as exc_info:
        gnss_ecef_from_ros(message)

    error = str(exc_info.value)
    assert "sensor_driver_msgs/msg/GnssEcef" in error
    assert "pos.x" in error
    assert "gnss_ecef_from_ros" in error


def test_internal_models_import_without_ros_packages(monkeypatch):
    for name in ["rclpy", "rosbag2_py", "sensor_driver_msgs"]:
        monkeypatch.setitem(sys.modules, name, None)

    import odd_extraction.models as models

    assert models.GnssEcefRecord.__name__ == "GnssEcefRecord"
