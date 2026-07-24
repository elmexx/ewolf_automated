"""Per-topic rosbag delivery health metrics."""
from dataclasses import dataclass, field
from statistics import median
from typing import Dict, Iterable, List, Optional, Tuple

from odd_extraction.validation.metadata import BagMetadata


@dataclass
class DataGap:
    topic: str
    gap_start_timestamp_ns: int
    gap_end_timestamp_ns: int
    gap_duration_s: float
    previous_record_timestamp_ns: int
    next_record_timestamp_ns: int
    threshold_s: float


@dataclass
class TopicHealth:
    topic: str
    message_type: str = ""
    metadata_message_count: Optional[int] = None
    database_message_count: Optional[int] = None
    health: str = "missing"
    reasons: List[str] = field(default_factory=list)
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None
    active_duration_s: Optional[float] = None
    estimated_frequency_hz: Optional[float] = None
    mean_frequency_hz: Optional[float] = None
    median_period_ms: Optional[float] = None
    p95_period_ms: Optional[float] = None
    maximum_gap_s: Optional[float] = None
    long_gap_count: Optional[int] = None
    long_gap_total_duration_s: Optional[float] = None
    coverage_ratio: Optional[float] = None
    start_delay_s: Optional[float] = None
    end_shortfall_s: Optional[float] = None
    duplicate_timestamp_count: Optional[int] = None
    timestamp_regression_count: Optional[int] = None
    inspection_mode: str = "metadata"


def percentile(values: List[float], percent: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * percent / 100.0))
    return ordered[index]


def metadata_only_health(metadata: BagMetadata, thresholds: Dict[str, float]) -> Dict[str, TopicHealth]:
    result = {}
    for topic in metadata.topics.values():
        est = None
        if metadata.duration_s > 0 and topic.message_count:
            est = topic.message_count / metadata.duration_s
        health = "metadata_only" if topic.message_count >= int(thresholds["minimum_message_count"]) else "insufficient_data"
        reasons = ["metadata_estimate_only"] if health == "metadata_only" else ["low_message_count", "metadata_estimate_only"]
        result[topic.name] = TopicHealth(
            topic=topic.name,
            message_type=topic.message_type,
            metadata_message_count=topic.message_count,
            health=health,
            reasons=reasons,
            estimated_frequency_hz=est,
            inspection_mode="metadata_only",
        )
    return result


def evaluate_topic_stats(metadata: BagMetadata, stats: Dict[str, dict], thresholds: Dict[str, float]) -> Tuple[Dict[str, TopicHealth], List[DataGap]]:
    topics = set(metadata.topics.keys()) | set(stats.keys())
    output: Dict[str, TopicHealth] = {}
    gaps: List[DataGap] = []
    bag_duration_s = metadata.duration_s
    bag_start = metadata.starting_time_ns
    bag_end = bag_start + metadata.duration_ns if bag_start is not None else None
    for name in sorted(topics):
        meta = metadata.topics.get(name)
        stat = stats.get(name)
        if not stat:
            output[name] = TopicHealth(name, meta.message_type if meta else "", meta.message_count if meta else None, 0, "missing", ["no_database_messages"], inspection_mode="timestamp")
            continue
        count = stat["count"]
        periods = stat["periods_s"]
        active = (stat["last"] - stat["first"]) / 1e9 if count > 1 else 0.0
        med = median(periods) if periods else None
        p95 = percentile(periods, 95) if periods else None
        max_gap = max(periods) if periods else None
        threshold_s = max(float(thresholds["minimum_long_gap_s"]), float(thresholds["long_gap_factor"]) * med) if med is not None else None
        topic_gaps = []
        if threshold_s is not None:
            for previous_ns, next_ns in stat["gap_pairs"]:
                gap_s = (next_ns - previous_ns) / 1e9
                if gap_s > threshold_s:
                    topic_gaps.append(DataGap(name, previous_ns, next_ns, gap_s, previous_ns, next_ns, threshold_s))
        reasons: List[str] = []
        if count < int(thresholds["minimum_message_count"]):
            health = "insufficient_data"
            reasons.append("low_message_count")
        else:
            coverage = active / bag_duration_s if bag_duration_s > 0 else None
            start_delay = (stat["first"] - bag_start) / 1e9 if bag_start is not None else None
            end_shortfall = (bag_end - stat["last"]) / 1e9 if bag_end is not None else None
            if coverage is not None and coverage < float(thresholds["degraded_coverage_ratio"]):
                reasons.append("very_low_coverage")
            elif coverage is not None and coverage < float(thresholds["minimum_coverage_ratio"]):
                reasons.append("low_coverage")
            if start_delay is not None and start_delay > float(thresholds["maximum_start_delay_s"]):
                reasons.append("starts_late")
            if end_shortfall is not None and end_shortfall > float(thresholds["maximum_end_shortfall_s"]):
                reasons.append("stops_early")
            if max_gap is not None and max_gap > float(thresholds["unusable_maximum_gap_s"]):
                reasons.append("excessive_gap")
            elif max_gap is not None and max_gap > float(thresholds["degraded_maximum_gap_s"]):
                reasons.append("large_gap")
            if topic_gaps:
                reasons.append("long_gaps_detected")
            if stat["regressions"]:
                reasons.append("timestamp_regression")
            if meta and meta.message_count != count:
                reasons.append("metadata_count_mismatch")
            health = "unusable" if any(r in reasons for r in ["very_low_coverage", "excessive_gap", "timestamp_regression"]) else ("degraded" if reasons else "usable")
        output[name] = TopicHealth(
            topic=name,
            message_type=meta.message_type if meta else stat.get("type", ""),
            metadata_message_count=meta.message_count if meta else None,
            database_message_count=count,
            health=health,
            reasons=reasons,
            first_timestamp_ns=stat["first"],
            last_timestamp_ns=stat["last"],
            active_duration_s=active,
            estimated_frequency_hz=(meta.message_count / bag_duration_s) if meta and bag_duration_s > 0 else None,
            mean_frequency_hz=(count - 1) / active if active > 0 and count > 1 else None,
            median_period_ms=med * 1000.0 if med is not None else None,
            p95_period_ms=p95 * 1000.0 if p95 is not None else None,
            maximum_gap_s=max_gap,
            long_gap_count=len(topic_gaps),
            long_gap_total_duration_s=sum(g.gap_duration_s for g in topic_gaps),
            coverage_ratio=active / bag_duration_s if bag_duration_s > 0 else None,
            start_delay_s=(stat["first"] - bag_start) / 1e9 if bag_start is not None else None,
            end_shortfall_s=(bag_end - stat["last"]) / 1e9 if bag_end is not None else None,
            duplicate_timestamp_count=stat["duplicates"],
            timestamp_regression_count=stat["regressions"],
            inspection_mode="timestamp",
        )
        gaps.extend(topic_gaps)
    return output, gaps
