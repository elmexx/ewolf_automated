"""Read rosbag2 SQLite record timestamps without reading message payloads."""
import sqlite3
from pathlib import Path
from typing import Dict, Iterable, List


class SqliteBagError(RuntimeError):
    """Raised when rosbag2 SQLite data cannot be inspected."""


def resolve_db_paths(bag_path: Path, relative_file_paths: Iterable[str]) -> List[Path]:
    paths = [bag_path / p for p in relative_file_paths if str(p).endswith(".db3")]
    if not paths:
        paths = sorted(bag_path.glob("*.db3"))
    missing = [str(p) for p in paths if not p.exists()]
    if missing:
        raise SqliteBagError("database file(s) listed by metadata are missing: %s" % ", ".join(missing))
    return paths


def inspect_sqlite_timestamps(db_paths: Iterable[Path]) -> Dict[str, dict]:
    stats: Dict[str, dict] = {}
    for path in db_paths:
        uri = "file:%s?mode=ro" % path
        try:
            conn = sqlite3.connect(uri, uri=True)
        except sqlite3.Error as exc:
            raise SqliteBagError("cannot open rosbag database %s: %s" % (path, exc)) from exc
        try:
            _validate_schema(conn, path)
            topic_rows = conn.execute("SELECT id, name, type FROM topics")
            id_to_topic = {int(row[0]): (str(row[1]), str(row[2])) for row in topic_rows}
            for topic_id, timestamp in conn.execute("SELECT topic_id, timestamp FROM messages ORDER BY topic_id, timestamp"):
                if int(topic_id) not in id_to_topic:
                    raise SqliteBagError("database %s contains message with unknown topic_id %s" % (path, topic_id))
                name, msg_type = id_to_topic[int(topic_id)]
                _add_timestamp(stats.setdefault(name, _empty_stat(msg_type)), int(timestamp))
        except sqlite3.Error as exc:
            raise SqliteBagError("malformed rosbag SQLite schema in %s: %s" % (path, exc)) from exc
        finally:
            conn.close()
    return stats


def _validate_schema(conn: sqlite3.Connection, path: Path) -> None:
    tables = {row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    required = {"topics", "messages"}
    if not required.issubset(tables):
        raise SqliteBagError("malformed rosbag SQLite schema in %s: missing %s" % (path, sorted(required - tables)))
    topic_cols = {row[1] for row in conn.execute("PRAGMA table_info(topics)")}
    msg_cols = {row[1] for row in conn.execute("PRAGMA table_info(messages)")}
    if not {"id", "name", "type"}.issubset(topic_cols) or not {"topic_id", "timestamp"}.issubset(msg_cols):
        raise SqliteBagError("malformed rosbag SQLite schema in %s: required columns missing" % path)


def _empty_stat(msg_type: str) -> dict:
    return {"type": msg_type, "count": 0, "first": None, "last": None, "periods_s": [], "gap_pairs": [], "duplicates": 0, "regressions": 0}


def _add_timestamp(stat: dict, timestamp: int) -> None:
    previous = stat["last"]
    if stat["first"] is None:
        stat["first"] = timestamp
    if previous is not None:
        if timestamp == previous:
            stat["duplicates"] += 1
        elif timestamp < previous:
            stat["regressions"] += 1
        else:
            stat["periods_s"].append((timestamp - previous) / 1e9)
            stat["gap_pairs"].append((previous, timestamp))
    stat["last"] = timestamp if previous is None or timestamp >= previous else previous
    stat["count"] += 1
