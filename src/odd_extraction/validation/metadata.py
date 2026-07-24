"""ROS 2 rosbag metadata.yaml parsing."""
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from odd_extraction.validation import yaml_compat as yaml


@dataclass(frozen=True)
class TopicMetadata:
    name: str
    message_type: str
    message_count: int


@dataclass(frozen=True)
class BagMetadata:
    path: Path
    storage_identifier: str
    relative_file_paths: List[str]
    duration_ns: int
    starting_time_ns: Optional[int]
    message_count: int
    topics: Dict[str, TopicMetadata]

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1_000_000_000.0


def load_metadata(bag_path: Path) -> BagMetadata:
    bag_path = Path(bag_path)
    if not bag_path.exists() or not bag_path.is_dir():
        raise FileNotFoundError("bag directory does not exist: %s" % bag_path)
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        raise FileNotFoundError("metadata.yaml is missing in bag directory: %s" % bag_path)
    text = metadata_path.read_text(encoding="utf-8")
    try:
        raw = yaml.safe_load(text) or {}
    except Exception:
        raw = _parse_rosbag_metadata_fallback(text)
    info = raw.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise ValueError("metadata.yaml does not contain rosbag2_bagfile_information")
    duration = info.get("duration") or {}
    starting = info.get("starting_time") or {}
    topics: Dict[str, TopicMetadata] = {}
    for item in info.get("topics_with_message_count") or []:
        topic_meta = item.get("topic_metadata") or {}
        name = topic_meta.get("name")
        if not name:
            continue
        topics[name] = TopicMetadata(
            name=name,
            message_type=str(topic_meta.get("type", "")),
            message_count=int(item.get("message_count", 0)),
        )
    return BagMetadata(
        path=bag_path,
        storage_identifier=str(info.get("storage_identifier", "")),
        relative_file_paths=[str(p) for p in info.get("relative_file_paths") or []],
        duration_ns=int(duration.get("nanoseconds", 0)),
        starting_time_ns=starting.get("nanoseconds_since_epoch"),
        message_count=int(info.get("message_count", 0)),
        topics=topics,
    )



def _parse_rosbag_metadata_fallback(text: str) -> dict:
    """Parse the ROS 2 metadata fields used by this tool when PyYAML is absent."""
    info = {
        "storage_identifier": "",
        "relative_file_paths": [],
        "duration": {"nanoseconds": 0},
        "starting_time": {"nanoseconds_since_epoch": None},
        "message_count": 0,
        "topics_with_message_count": [],
    }
    context = []
    current_topic = None
    expect_file = False
    for raw in text.splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped == "relative_file_paths:":
            expect_file = True
            continue
        if expect_file and stripped.startswith("-"):
            info["relative_file_paths"].append(stripped[1:].strip())
            continue
        if not stripped.startswith("-") and ":" in stripped:
            expect_file = False
        if stripped == "duration:":
            context = ["duration"]
            continue
        if stripped == "starting_time:":
            context = ["starting_time"]
            continue
        if stripped in ("topic_metadata:", "- topic_metadata:"):
            current_topic = {"topic_metadata": {}, "message_count": 0}
            info["topics_with_message_count"].append(current_topic)
            context = ["topic"]
            continue
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        key = key.strip(); value = value.strip().strip('"')
        if key == "storage_identifier":
            info["storage_identifier"] = value
        elif key == "nanoseconds" and context == ["duration"]:
            info["duration"]["nanoseconds"] = int(value)
        elif key == "nanoseconds_since_epoch":
            info["starting_time"]["nanoseconds_since_epoch"] = int(value)
        elif key == "message_count":
            if current_topic is not None and context == ["topic"]:
                current_topic["message_count"] = int(value)
            else:
                info["message_count"] = int(value)
        elif current_topic is not None and key in {"name", "type", "serialization_format", "offered_qos_profiles"}:
            current_topic["topic_metadata"][key] = value
    return {"rosbag2_bagfile_information": info}
