"""YAML/CSV report generation for bag health inspection."""
import csv
import logging
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Iterable, List

from odd_extraction.validation import yaml_compat as yaml

from odd_extraction.validation.domain_health import DomainHealth
from odd_extraction.validation.metadata import BagMetadata
from odd_extraction.validation.topic_health import DataGap, TopicHealth

TOPIC_COLUMNS = ["topic","message_type","metadata_message_count","database_message_count","health","reason_codes","first_timestamp_ns","last_timestamp_ns","active_duration_s","estimated_frequency_hz","mean_frequency_hz","median_period_ms","p95_period_ms","maximum_gap_s","long_gap_count","long_gap_total_duration_s","coverage_ratio","start_delay_s","end_shortfall_s","duplicate_timestamp_count","timestamp_regression_count","inspection_mode"]
GAP_COLUMNS = ["topic","gap_start_timestamp_ns","gap_end_timestamp_ns","gap_duration_s","previous_record_timestamp_ns","next_record_timestamp_ns","threshold_s"]


def ensure_output_dir(path: Path, overwrite: bool) -> None:
    existing = ["bag_health_summary.yaml", "topic_health.csv", "data_gaps.csv", "bag_health.log"]
    if path.exists() and not path.is_dir():
        raise FileExistsError("output path exists and is not a directory: %s" % path)
    if path.exists() and not overwrite:
        conflicts = [name for name in existing if (path / name).exists()]
        if conflicts:
            raise FileExistsError("output files already exist; use --overwrite: %s" % ", ".join(conflicts))
    path.mkdir(parents=True, exist_ok=True)


def write_reports(output: Path, metadata: BagMetadata, topics: Dict[str, TopicHealth], gaps: List[DataGap], domains: Dict[str, DomainHealth], analysis_status: str, analysis_reasons: List[str], next_step: dict, inspection_mode: str) -> None:
    summary = {
        "schema_version": 1,
        "bag": {"path": str(metadata.path), "storage_identifier": metadata.storage_identifier, "duration_s": metadata.duration_s, "start_time_ns": metadata.starting_time_ns, "relative_file_paths": metadata.relative_file_paths, "total_message_count": metadata.message_count, "inspection_mode": inspection_mode},
        "analysis": {"status": analysis_status, "reasons": analysis_reasons},
        "domains": {name: asdict(value) for name, value in domains.items()},
        "next_step": next_step,
    }
    with (output / "bag_health_summary.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(summary, stream, sort_keys=True)
    with (output / "topic_health.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TOPIC_COLUMNS)
        writer.writeheader()
        for item in sorted(topics.values(), key=lambda x: x.topic):
            row = {column: _fmt(getattr(item, column)) for column in TOPIC_COLUMNS if column != "reason_codes"}
            row["reason_codes"] = ";".join(item.reasons)
            writer.writerow(row)
    with (output / "data_gaps.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=GAP_COLUMNS)
        writer.writeheader()
        for gap in gaps:
            writer.writerow({column: _fmt(getattr(gap, column)) for column in GAP_COLUMNS})


def configure_file_logging(output: Path, level: str) -> None:
    logging.basicConfig(filename=str(output / "bag_health.log"), level=getattr(logging, level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s", force=True)


def _fmt(value):
    if value is None:
        return ""
    return value
