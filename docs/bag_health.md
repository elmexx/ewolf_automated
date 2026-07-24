# Rosbag data availability and health inspection

`odd-bag-health` is a lightweight preflight check for ROS 2 Foxy rosbag2
recordings. It decides whether a bag has enough recorded topic availability and
record-timestamp continuity for later ODD analysis. It reads `metadata.yaml` and,
when requested, rosbag2 SQLite3 `topics` and `messages` table columns only:
`topics.id`, `topics.name`, `topics.type`, `messages.topic_id`, and
`messages.timestamp`.

It does **not** deserialize payloads, decode custom messages, inspect lane or
object results, check TF transform validity, replay simulated time, evaluate
source-message timestamp latency, synchronize topics, interpolate, extract ODD
features, or visualize data. Record timestamp continuity does not prove sensor
result correctness; continuous lane-detection messages do not mean lane
detections are stable.

## Modes

Metadata-only mode (`--metadata-only`) works with only `metadata.yaml`. It
reports bag duration, start time, storage identifier, relative bag files, total
message count, topic names, topic types, per-topic metadata counts, and estimated
average frequency (`message_count / bag_duration`). Estimated values are marked
with `metadata_only` health and the `metadata_estimate_only` reason.

Timestamp mode opens `.db3` files read-only and streams timestamps ordered by
`topic_id, timestamp`. It does not select `messages.data`, so payload blobs are
not loaded. Split bags are supported by inspecting every `.db3` file listed in
metadata and merging per-topic statistics.

## Metrics

Timestamps are rosbag record timestamps from the SQLite `messages.timestamp`
column, not sensor header timestamps.

Per topic, timestamp mode computes first and last record timestamp, active
duration, database message count, mean frequency over active duration, median
period, 95th-percentile period, maximum gap, duplicate timestamps, timestamp
regressions, metadata/database count match, start delay from bag start, end
shortfall from bag end, and coverage ratio.

Coverage ratio is:

```text
topic active duration / bag duration
```

This captures start/stop coverage but not all internal gaps. Internal gaps are
reported separately.

The long-gap threshold is derived per topic from the median observed period:

```text
max(minimum_long_gap_s, long_gap_factor * median_period_s)
```

If too few messages exist to calculate a period, the topic is marked
`insufficient_data` rather than assigning a fabricated rate.

## Topic states

Topics are classified as `usable`, `degraded`, `unusable`, `missing`,
`insufficient_data`, or `metadata_only`. Machine-readable reason codes include
`low_message_count`, `low_coverage`, `very_low_coverage`, `starts_late`,
`stops_early`, `large_gap`, `excessive_gap`, `long_gaps_detected`,
`timestamp_regression`, `metadata_count_mismatch`, `no_database_messages`, and
`metadata_estimate_only`.

Thresholds live in `config/bag_health.yaml`, not in downstream business logic.

## Domains and decisions

Configured information domains are based on the repository topic catalog and
sample bag metadata:

- `ego_motion`: `/vehicle_dynamic_data`, `/odometry/vehicle`,
  `/odometry/filtered`, `/gnss/velocity`, `/imu/data`
- `localization`: `/gnss/fix`, `/gnss/ecef`, `/gnss/ned`, `/gnss/quality`,
  `/gnss/status`, `/odometry/filtered`
- `lane_observation`: `/detection/lane/lane_markings_projected`,
  `/detection/lane/rightlanedetection`, `/detection/lane/leftlanedetection`,
  `/detection/lane/image_raw`
- `object_observation`: `/detection/object/yolo2ddetection`,
  `/detection/object/yolo2dbboxes`, `/detection/object/image_raw`
- `camera`: `/camera/color/image_raw`, `/camera/imu`, `/camera/gyro/sample`,
  `/camera/accel/sample`
- `tf`: `/tf`, `/tf_static`

`policy: any` means at least the configured `minimum_usable_topics` suitable
sources can support the domain. `policy: all` means every listed topic is
required; degraded topics make the domain degraded, and missing/unusable topics
make it unusable.

Overall status is `full`, `partial`, or `rejected`. A bag is rejected only when
no configured core domain (`ego_motion` or `localization`) is usable/degraded, or
when fatal input errors prevent inspection. Missing or degraded lane/object
observation topics alone produce `partial`, not `rejected`.

## CLI examples

```bash
odd-bag-health /path/to/bag --config config/bag_health.yaml --output /tmp/health
odd-bag-health /path/to/bag --metadata-only --output /tmp/health --overwrite
odd-bag-health --help
```

The command does not overwrite existing report files unless `--overwrite` is
provided. Data-quality `partial` or `rejected` decisions still return exit code
0; missing metadata, missing/corrupt databases, malformed schema, and overwrite
conflicts return nonzero.

## Outputs

`bag_health_summary.yaml` contains bag metadata, analysis status/reasons,
domain evidence, and next-step supported outputs derived from domain health.

`topic_health.csv` includes topic, message type, metadata/database counts,
health, reason codes, timestamp bounds, active duration, estimated and measured
frequencies, period/gap metrics, coverage, start delay, end shortfall, duplicate
count, regression count, and inspection mode. Unknown values are blank.

`data_gaps.csv` contains one row per long gap with topic, previous/next record
timestamps, gap duration, and applied threshold. If no gaps are detected, it is
still created with headers.

`bag_health.log` records input path, inspection mode, inspected database files,
warnings, unavailable/rejected topics, and final analysis decision.

## Known limitations

The tool evaluates recording continuity only. It does not prove semantic sensor
latency, message contents, object/lane detection stability, TF availability at
query times, or suitability for any specific ODD attribute. Those checks belong
to later processing steps.
