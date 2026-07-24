"""Command line entry point for rosbag availability and health inspection."""
import argparse
import logging
from pathlib import Path
from typing import List, Optional

from odd_extraction.validation import yaml_compat as yaml

from odd_extraction.validation.domain_health import evaluate_analysis, evaluate_domains
from odd_extraction.validation.metadata import load_metadata
from odd_extraction.validation.report import configure_file_logging, ensure_output_dir, write_reports
from odd_extraction.validation.sqlite_reader import SqliteBagError, inspect_sqlite_timestamps, resolve_db_paths
from odd_extraction.validation.topic_health import evaluate_topic_stats, metadata_only_health


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect ROS 2 rosbag2 metadata and SQLite timestamps for ODD data availability.")
    parser.add_argument("bag_path", metavar="BAG_PATH", help="Path to a rosbag2 directory containing metadata.yaml")
    parser.add_argument("--config", default="config/bag_health.yaml", help="YAML health configuration path")
    parser.add_argument("--output", default="bag_health_output", help="Output directory for YAML, CSV, and log files")
    parser.add_argument("--metadata-only", action="store_true", help="Inspect metadata.yaml only and do not require .db3 files")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing output report files")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"], help="Log verbosity")
    return parser


def run(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    metadata = load_metadata(Path(args.bag_path))
    output = Path(args.output)
    ensure_output_dir(output, args.overwrite)
    configure_file_logging(output, args.log_level)
    logging.info("input path: %s", metadata.path)
    thresholds = config.get("thresholds") or {}
    if args.metadata_only:
        inspection_mode = "metadata_only"
        topic_health = metadata_only_health(metadata, thresholds)
        gaps = []
        logging.info("inspection mode: metadata_only")
        logging.info("database files inspected: none")
    else:
        inspection_mode = "timestamp"
        db_paths = resolve_db_paths(metadata.path, metadata.relative_file_paths)
        logging.info("inspection mode: timestamp")
        logging.info("database files inspected: %s", ", ".join(str(p) for p in db_paths))
        stats = inspect_sqlite_timestamps(db_paths)
        topic_health, gaps = evaluate_topic_stats(metadata, stats, thresholds)
    domains = evaluate_domains(topic_health, config)
    status, reasons, next_step = evaluate_analysis(domains, config)
    for topic in sorted(t for t, h in topic_health.items() if h.health in ("missing", "unusable", "insufficient_data")):
        logging.warning("rejected or unavailable topic %s: %s", topic, topic_health[topic].reasons)
    logging.info("final analysis decision: %s", status)
    write_reports(output, metadata, topic_health, gaps, domains, status, reasons, next_step, inspection_mode)
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (FileNotFoundError, FileExistsError, ValueError, OSError, SqliteBagError, yaml.YAMLError) as exc:
        parser.print_usage()
        print("odd-bag-health: error: %s" % exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
