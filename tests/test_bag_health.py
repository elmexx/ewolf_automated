import csv
import sqlite3
from pathlib import Path

import pytest
import json

from odd_extraction.cli.bag_health import main
from odd_extraction.validation.metadata import load_metadata
from odd_extraction.validation.sqlite_reader import SqliteBagError, inspect_sqlite_timestamps, resolve_db_paths
from odd_extraction.validation.topic_health import evaluate_topic_stats, metadata_only_health
from odd_extraction.validation.domain_health import evaluate_analysis, evaluate_domains

from odd_extraction.validation import yaml_compat as yaml
CFG = yaml.safe_load(Path("config/bag_health.yaml").read_text())
NS = 1_000_000_000


def write_metadata(path, topics, duration=10*NS, files=None, start=100*NS):
    path.mkdir(exist_ok=True)
    data = {"rosbag2_bagfile_information": {"version": 4, "storage_identifier": "sqlite3", "relative_file_paths": files or ["bag_0.db3"], "duration": {"nanoseconds": duration}, "starting_time": {"nanoseconds_since_epoch": start}, "message_count": sum(c for _, _, c in topics), "topics_with_message_count": []}}
    for name, typ, count in topics:
        data["rosbag2_bagfile_information"]["topics_with_message_count"].append({"topic_metadata": {"name": name, "type": typ, "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": count})
    (path / "metadata.yaml").write_text(json.dumps(data))


def write_db(path, topic_defs, messages):
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)")
    conn.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)")
    for tid, name, typ in topic_defs:
        conn.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (tid, name, typ, "cdr", ""))
    for i, (tid, ts) in enumerate(messages, 1):
        conn.execute("INSERT INTO messages VALUES(?,?,?,?)", (i, tid, ts, b"payload-not-read"))
    conn.commit(); conn.close()


def inspect(tmp_path, topics, messages, duration=10*NS, files=None):
    write_metadata(tmp_path, topics, duration=duration, files=files)
    if files is None:
        write_db(tmp_path / "bag_0.db3", [(i+1, t[0], t[1]) for i, t in enumerate(topics)], messages)
    meta = load_metadata(tmp_path)
    dbs = resolve_db_paths(tmp_path, meta.relative_file_paths)
    return evaluate_topic_stats(meta, inspect_sqlite_timestamps(dbs), CFG["thresholds"])


def test_metadata_only_inspection(tmp_path):
    write_metadata(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 10)])
    meta = load_metadata(tmp_path)
    health = metadata_only_health(meta, CFG["thresholds"])["/imu/data"]
    assert health.health == "metadata_only"
    assert health.estimated_frequency_hz == 1.0


def test_healthy_continuous_topic(tmp_path):
    topics, _ = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 10)], [(1, (100+i)*NS) for i in range(10)])
    h = topics["/imu/data"]
    assert h.health == "usable"
    assert h.median_period_ms == 1000.0
    assert h.long_gap_count == 0


def test_topic_starting_late_and_stopping_early(tmp_path):
    topics, _ = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 3), ("/odometry/vehicle", "nav_msgs/msg/Odometry", 3)], [(1, 106*NS), (1, 107*NS), (1, 108*NS), (2, 100*NS), (2, 101*NS), (2, 102*NS)])
    assert "starts_late" in topics["/imu/data"].reasons
    assert "stops_early" in topics["/odometry/vehicle"].reasons


def test_internal_long_gap(tmp_path):
    topics, gaps = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 4)], [(1, 100*NS), (1, 101*NS), (1, 107*NS), (1, 108*NS)])
    assert "long_gaps_detected" in topics["/imu/data"].reasons
    assert len(gaps) == 1
    assert gaps[0].gap_duration_s == 6.0


def test_low_message_count_and_missing_topic(tmp_path):
    topics, _ = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 1), ("/gnss/fix", "sensor_msgs/msg/NavSatFix", 0)], [(1, 100*NS)])
    assert topics["/imu/data"].health == "insufficient_data"
    assert topics["/gnss/fix"].health == "missing"


def test_domain_partial_when_lane_degraded_not_rejected(tmp_path):
    topics, _ = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 10), ("/gnss/status", "sensor_driver_msgs/msg/GnssStatus", 10), ("/detection/lane/leftlanedetection", "lane_parameter_msg/msg/LaneParams", 3)], [(1, (100+i)*NS) for i in range(10)] + [(2, (100+i)*NS) for i in range(10)] + [(3, 100*NS), (3, 106*NS), (3, 108*NS)])
    domains = evaluate_domains(topics, CFG)
    status, reasons, _ = evaluate_analysis(domains, CFG)
    assert domains["lane_observation"].health in ("degraded", "unusable")
    assert status == "partial"


def test_missing_lane_and_object_with_usable_localization_is_partial(tmp_path):
    topics, _ = inspect(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 10), ("/gnss/status", "sensor_driver_msgs/msg/GnssStatus", 10)], [(1, (100+i)*NS) for i in range(10)] + [(2, (100+i)*NS) for i in range(10)])
    status, _, _ = evaluate_analysis(evaluate_domains(topics, CFG), CFG)
    assert status == "partial"


def test_rejection_when_no_ego_motion_or_localization(tmp_path):
    topics, _ = inspect(tmp_path, [("/detection/object/yolo2dbboxes", "bboxes_msg/msg/BoundingBoxes", 10)], [(1, (100+i)*NS) for i in range(10)])
    assert evaluate_analysis(evaluate_domains(topics, CFG), CFG)[0] == "rejected"


def test_split_bag_and_metadata_count_mismatch(tmp_path):
    write_metadata(tmp_path, [("/imu/data", "sensor_msgs/msg/Imu", 3)], files=["a.db3", "b.db3"])
    write_db(tmp_path/"a.db3", [(1, "/imu/data", "sensor_msgs/msg/Imu")], [(1, 100*NS), (1, 101*NS)])
    write_db(tmp_path/"b.db3", [(1, "/imu/data", "sensor_msgs/msg/Imu")], [(1, 102*NS), (1, 103*NS)])
    meta = load_metadata(tmp_path)
    topics, _ = evaluate_topic_stats(meta, inspect_sqlite_timestamps(resolve_db_paths(tmp_path, meta.relative_file_paths)), CFG["thresholds"])
    assert topics["/imu/data"].database_message_count == 4
    assert "metadata_count_mismatch" in topics["/imu/data"].reasons


def test_cli_outputs_headers_overwrite_and_no_gaps(tmp_path):
    bag = tmp_path / "bag"
    write_metadata(bag, [("/imu/data", "sensor_msgs/msg/Imu", 10)])
    write_db(bag/"bag_0.db3", [(1, "/imu/data", "sensor_msgs/msg/Imu")], [(1, (100+i)*NS) for i in range(10)])
    out = tmp_path / "out"
    assert main([str(bag), "--config", "config/bag_health.yaml", "--output", str(out)]) == 0
    assert (out / "bag_health_summary.yaml").exists()
    assert next(csv.reader((out / "topic_health.csv").open()))[0] == "topic"
    assert list(csv.reader((out / "data_gaps.csv").open())) == [["topic", "gap_start_timestamp_ns", "gap_end_timestamp_ns", "gap_duration_s", "previous_record_timestamp_ns", "next_record_timestamp_ns", "threshold_s"]]
    assert main([str(bag), "--config", "config/bag_health.yaml", "--output", str(out)]) == 2


def test_missing_metadata_error(tmp_path):
    tmp_path.mkdir(exist_ok=True)
    with pytest.raises(FileNotFoundError):
        load_metadata(tmp_path)


def test_malformed_sqlite_schema_error(tmp_path):
    db = tmp_path / "bad.db3"
    sqlite3.connect(db).execute("CREATE TABLE nope(id INTEGER)").connection.close()
    with pytest.raises(SqliteBagError):
        inspect_sqlite_timestamps([db])


def test_model_imports_without_ros_installed(monkeypatch):
    import sys
    for name in ["rclpy", "rosbag2_py", "sensor_driver_msgs"]:
        monkeypatch.setitem(sys.modules, name, None)
    import odd_extraction.models as models
    assert models.ImuStatusRecord.__name__ == "ImuStatusRecord"
