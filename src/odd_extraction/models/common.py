"""Common ROS-independent value objects for ODD extraction records."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Timestamp:
    """Timestamp stored as integer nanoseconds."""

    nanoseconds: int


@dataclass(frozen=True)
class MessageMetadata:
    """Metadata copied from a ROS message header without ROS dependencies."""

    timestamp: Timestamp
    frame_id: Optional[str] = None


@dataclass(frozen=True)
class Vector3:
    """Three-component vector preserving source x/y/z field names."""

    x: float
    y: float
    z: float
