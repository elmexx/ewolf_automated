from odd_extraction.models import ImuStatusRecord, MessageMetadata, Timestamp


def test_imu_status_model_preserves_mask_and_validity_flags():
    record = ImuStatusRecord(
        metadata=MessageMetadata(timestamp=Timestamp(987), frame_id="imu"),
        mask=0x0F,
        accel_valid=True,
        ypr_valid=False,
        mag_valid=True,
        gyro_valid=False,
    )

    assert record.metadata.timestamp.nanoseconds == 987
    assert record.metadata.frame_id == "imu"
    assert record.mask == 0x0F
    assert record.accel_valid is True
    assert record.ypr_valid is False
    assert record.mag_valid is True
    assert record.gyro_valid is False
