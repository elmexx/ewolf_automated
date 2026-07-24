from odd_extraction.models import (
    GnssEcefRecord,
    GnssNedRecord,
    GnssQualityRecord,
    GnssStatusRecord,
    MessageMetadata,
    Timestamp,
    Vector3,
)


def _metadata():
    return MessageMetadata(timestamp=Timestamp(123), frame_id="gnss")


def test_gnss_ecef_model_preserves_fields():
    record = GnssEcefRecord(
        metadata=_metadata(),
        station=42,
        pos=Vector3(1.0, 2.0, 3.0),
        velocity=Vector3(4.0, 5.0, 6.0),
        pos_error=0.7,
        speed_error=0.8,
    )

    assert record.metadata.timestamp.nanoseconds == 123
    assert record.metadata.frame_id == "gnss"
    assert record.station == 42
    assert record.pos == Vector3(1.0, 2.0, 3.0)
    assert record.velocity == Vector3(4.0, 5.0, 6.0)
    assert record.pos_error == 0.7
    assert record.speed_error == 0.8


def test_gnss_ned_model_preserves_fields():
    record = GnssNedRecord(
        metadata=_metadata(),
        rel_pos_length=9.0,
        rel_pos_heading=10.0,
        rel_pos=Vector3(11.0, 12.0, 13.0),
        rel_speed=Vector3(14.0, 15.0, 16.0),
    )

    assert record.rel_pos_length == 9.0
    assert record.rel_pos_heading == 10.0
    assert record.rel_pos == Vector3(11.0, 12.0, 13.0)
    assert record.rel_speed == Vector3(14.0, 15.0, 16.0)


def test_gnss_quality_model_preserves_fields():
    record = GnssQualityRecord(
        metadata=_metadata(),
        time_error=1.0,
        latitude_error=2.0,
        longitude_error=3.0,
        altitude_error=4.0,
        speed_error=5.0,
        climb_error=6.0,
        pos2d_error=7.0,
        pos3d_error=8.0,
        xdop=9.0,
        ydop=10.0,
        pdop=11.0,
        hdop=12.0,
        vdop=13.0,
        tdop=14.0,
        gdop=15.0,
    )

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


def test_gnss_status_model_preserves_raw_status_fields():
    record = GnssStatusRecord(
        metadata=_metadata(),
        sensor_time=1234.5,
        online=1.0,
        mask=0xA5,
        satellites_used=8,
        satellites_visible=12,
        dgps_station=3,
        dgps_age=0.4,
    )

    assert record.sensor_time == 1234.5
    assert record.online == 1.0
    assert record.mask == 0xA5
    assert record.satellites_used == 8
    assert record.satellites_visible == 12
    assert record.dgps_station == 3
    assert record.dgps_age == 0.4
