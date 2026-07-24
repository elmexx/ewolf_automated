# ODD Extraction Internal Data Models

This document describes the first ODD extraction development step: a plain
Python model layer and duck-typed ROS adapter layer for the custom GNSS and IMU
messages already present in this repository.

## Repository interfaces inspected

The five custom messages are defined by the `sensor_driver_msgs` ROS 2 interface
package under `src/sensor_driver_msgs/msg`:

| Topic from `docs/sample_bag/metadata.yaml` | ROS type | Internal record | Adapter |
| --- | --- | --- | --- |
| `/gnss/ecef` | `sensor_driver_msgs/msg/GnssEcef` | `GnssEcefRecord` | `gnss_ecef_from_ros` |
| `/gnss/ned` | `sensor_driver_msgs/msg/GnssNed` | `GnssNedRecord` | `gnss_ned_from_ros` |
| `/gnss/quality` | `sensor_driver_msgs/msg/GnssQuality` | `GnssQualityRecord` | `gnss_quality_from_ros` |
| `/gnss/status` | `sensor_driver_msgs/msg/GnssStatus` | `GnssStatusRecord` | `gnss_status_from_ros` |
| `/imu/status` | `sensor_driver_msgs/msg/ImuStatus` | `ImuStatusRecord` | `imu_status_from_ros` |

The sample bag metadata reports rosbag2 `sqlite3` storage, duration
`678625386` ns, and total message count `290`. Message counts for the five
custom topics are: `/gnss/ecef` 0, `/gnss/ned` 0, `/gnss/quality` 0,
`/gnss/status` 0, and `/imu/status` 10.

Lane-detection topics discovered in the metadata use `lane_parameter_msg`:

- `/detection/lane/lane_markings_projected` →
  `lane_parameter_msg/msg/LaneMarkingProjectedArrayBoth`
- `/detection/lane/rightlanedetection` → `lane_parameter_msg/msg/LaneParams`
- `/detection/lane/leftlanedetection` → `lane_parameter_msg/msg/LaneParams`
- `/detection/lane/image_raw` → `sensor_msgs/msg/Image`

Object-detection topics discovered in the metadata use `bboxes_msg` and
`vision_msgs`:

- `/detection/object/yolo2dbboxes` → `bboxes_msg/msg/BoundingBoxes`
- `/detection/object/yolo2ddetection` → `vision_msgs/msg/Detection2DArray`
- `/detection/object/image_raw` → `sensor_msgs/msg/Image`

Recording-launch/package ownership is not explicitly documented in the sample
metadata. The repository contains top-level camera and lidar launch files and
sensor packages, but no checked-in rosbag record launch file was identified by
this step. The workspace is ROS 2 Foxy-oriented; Ubuntu 20.04 and Python 3.8 are
therefore treated as compatibility targets.

## Message field mapping

All records carry `metadata: MessageMetadata`, copied from `std_msgs/Header`:

| ROS field | Internal field |
| --- | --- |
| `header.stamp.sec`, `header.stamp.nanosec` | `metadata.timestamp.nanoseconds` |
| `header.frame_id` | `metadata.frame_id` |

Timestamp conversion is always:

```text
timestamp_ns = sec * 1_000_000_000 + nanosec
```

### `sensor_driver_msgs/msg/GnssEcef`

| ROS field | Internal field |
| --- | --- |
| `station` | `station` |
| `pos.x`, `pos.y`, `pos.z` | `pos: Vector3` |
| `velocity.x`, `velocity.y`, `velocity.z` | `velocity: Vector3` |
| `pos_error` | `pos_error` |
| `speed_error` | `speed_error` |

`topic_catalog.md` documents `pos` and `velocity` as ECEF coordinates with
expected units of metres and metres per second respectively.

### `sensor_driver_msgs/msg/GnssNed`

| ROS field | Internal field |
| --- | --- |
| `rel_pos_length` | `rel_pos_length` |
| `rel_pos_heading` | `rel_pos_heading` |
| `rel_pos.x`, `rel_pos.y`, `rel_pos.z` | `rel_pos: Vector3` |
| `rel_speed.x`, `rel_speed.y`, `rel_speed.z` | `rel_speed: Vector3` |

`topic_catalog.md` names these as relative position and speed in NED
coordinates, but also warns that the mapping of `geometry_msgs/Vector3` x/y/z to
north/east/down still requires confirmation from driver documentation. The
internal record therefore preserves x/y/z without renaming them.

### `sensor_driver_msgs/msg/GnssQuality`

The internal `GnssQualityRecord` preserves: `time_error`, `latitude_error`,
`longitude_error`, `altitude_error`, `speed_error`, `climb_error`,
`pos2d_error`, `pos3d_error`, `xdop`, `ydop`, `pdop`, `hdop`, `vdop`, `tdop`,
and `gdop`.

### `sensor_driver_msgs/msg/GnssStatus`

The internal `GnssStatusRecord` preserves: `sensor_time`, `online`, raw `mask`,
`satellites_used`, `satellites_visible`, `dgps_station`, and `dgps_age`.
Individual `mask` bit meanings are undocumented in the repository and are not
decoded.

### `sensor_driver_msgs/msg/ImuStatus`

The internal `ImuStatusRecord` preserves: raw `mask`, `accel_valid`,
`ypr_valid`, `mag_valid`, and `gyro_valid`. Individual `mask` bit meanings are
undocumented in the repository and are not decoded.

## Adapter boundary

The dependency direction is:

```text
ROS-like message object → odd_extraction.adapters.ros_messages → dataclass record
```

Internal modules under `odd_extraction.models` do not import `rclpy`,
`rosbag2_py`, generated custom message classes, or other ROS-specific packages.
The adapter layer uses duck typing and raises `MissingRosFieldError` when a
required attribute path is absent.

## Assumptions and unresolved semantics

- The five custom message types are owned by `sensor_driver_msgs` because their
  `.msg` files and package manifest are located under `src/sensor_driver_msgs`.
- `online` is preserved as `float` because `GnssStatus.msg` declares it as
  `float64`, even though the topic catalog describes it as an online status.
- Units are documented only where `topic_catalog.md` states them. Other numeric
  fields are copied without unit interpretation.
- GNSS and IMU masks are raw status values. Their bit layouts remain unresolved.
- `GnssNed` vector x/y/z axis-to-NED mapping remains unresolved and is not
  renamed in the model.

## Explicit non-goals

This step does not implement rosbag reading or deserialization, rosbag playback,
time synchronization, interpolation, fixed-rate frame generation, quality
scoring, lane or object adapters, feature extraction, segmentation, ODD
classification, map matching, file export, visualization, or ROS nodes.
