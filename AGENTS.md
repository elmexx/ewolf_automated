Read the repository-level `AGENTS.md` and `topic_catalog.md` before making any changes.

Inspect the complete ROS 2 Foxy workspace in this repository. The repository contains the packages used to record the vehicle rosbag, including custom GNSS and IMU messages and existing lane-detection and object-detection packages.

Also inspect the sample rosbag metadata file if present, for example:

```text
docs/sample_bag/metadata.yaml
```

Use the metadata only to verify actual topic names, message types, message counts, bag duration, and storage format. Do not implement rosbag deserialization in this task.

# Goal

Complete the first development step of the ODD extraction framework:

1. understand the existing workspace and interfaces;
2. create a new, isolated ROS 2 Python package named `odd_extraction`;
3. implement ROS-independent internal data models for the custom GNSS and IMU messages;
4. implement a ROS adapter layer;
5. add unit tests and documentation.

# Repository inspection

Before editing, identify and report:

* the ROS 2 package that defines `GnssEcef`;
* the ROS 2 package that defines `GnssNed`;
* the ROS 2 package that defines `GnssQuality`;
* the ROS 2 package that defines `GnssStatus`;
* the ROS 2 package that defines `ImuStatus`;
* their exact field definitions;
* the corresponding topic names found in the bag metadata;
* the packages and message types used for lane detection;
* the packages and message types used for object detection;
* the package or launch configuration used to record the rosbag;
* the ROS 2 distribution and supported Python version inferred from the repository.

Do not change existing sensor, interface, localization, lane-detection, object-detection, launch, or recording packages during this task.

If an existing package appears broken, document the problem instead of modifying unrelated code.

# New package

Create a new ROS 2 Python package named `odd_extraction` in the appropriate workspace source directory.

Use a structure similar to:

```text
odd_extraction/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── odd_extraction
├── config/
├── docs/
│   └── message_normalization.md
├── odd_extraction/
│   ├── __init__.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── common.py
│   │   ├── gnss.py
│   │   └── imu.py
│   └── adapters/
│       ├── __init__.py
│       └── ros_messages.py
└── test/
    ├── test_models.py
    ├── test_ros_adapters.py
    └── test_import_without_ros.py
```

Adapt the exact location to the repository's actual ROS 2 workspace layout.

The package must remain compatible with Ubuntu 20.04, ROS 2 Foxy, and Python 3.8 unless the repository clearly establishes different requirements.

# Architecture boundary

Use this dependency direction:

```text
ROS message
    ↓
ROS adapter
    ↓
plain Python internal record
    ↓
future synchronization and feature extraction
```

The internal model modules must not import:

* `rclpy`;
* `rosbag2_py`;
* generated ROS message classes;
* ROS-specific Python packages.

Only the adapter layer may contain ROS-specific message access or imports.

Do not implement a ROS node in this task.

# Internal models

Create typed Python dataclasses for the five actual custom message definitions:

* `GnssEcefRecord`;
* `GnssNedRecord`;
* `GnssQualityRecord`;
* `GnssStatusRecord`;
* `ImuStatusRecord`.

Create reusable value objects where appropriate, such as:

* `MessageMetadata`;
* `Vector3Record`.

Preserve all fields found in the actual `.msg` definitions.

At minimum, preserve:

* timestamp as integer nanoseconds;
* `frame_id`, when the source message has one;
* all numerical fields;
* all boolean validity fields;
* all raw mask, status, or flag values;
* all nested vector components.

Convert ROS timestamps using:

```python
timestamp_ns = sec * 1_000_000_000 + nanosec
```

Do not guess undocumented units.

Do not decode undocumented mask bits.

Do not rename coordinates based only on assumptions. For example, do not automatically interpret `x`, `y`, and `z` as north, east, and down unless the message definition, driver source code, or repository documentation confirms this mapping.

# ROS adapters

Implement adapter functions for all five custom message types.

Use clear names such as:

```python
gnss_ecef_from_ros(message: object) -> GnssEcefRecord
gnss_ned_from_ros(message: object) -> GnssNedRecord
gnss_quality_from_ros(message: object) -> GnssQualityRecord
gnss_status_from_ros(message: object) -> GnssStatusRecord
imu_status_from_ros(message: object) -> ImuStatusRecord
```

The exact implementation may use duck typing so that tests do not require ROS 2 to be installed.

If generated ROS message classes are imported, keep those imports optional and entirely inside the adapter layer.

When a required field is missing, raise a descriptive exception containing:

* the expected message type;
* the missing field path;
* the adapter function involved.

Do not silently substitute zero, `False`, an empty string, or another fabricated value for a missing required field.

# Tests

Tests must run without ROS 2 installed.

Use `types.SimpleNamespace`, small test dataclasses, or fixtures to imitate ROS messages.

Test at least:

1. conversion of `sec` and `nanosec` to integer nanoseconds;
2. preservation of every field in `GnssEcef`;
3. preservation of every field in `GnssNed`;
4. preservation of every field in `GnssQuality`;
5. preservation of every field in `GnssStatus`;
6. preservation of every field in `ImuStatus`;
7. conversion of nested vector fields;
8. preservation of boolean validity flags;
9. preservation of raw masks and status values;
10. descriptive errors for missing required fields;
11. importing `odd_extraction.models` without ROS installed.

Avoid tests that merely recreate the implementation logic without checking meaningful behavior.

# Documentation

Create:

```text
odd_extraction/docs/message_normalization.md
```

Document:

* the discovered source package for each custom message;
* the exact ROS message type names;
* the topic names confirmed by `metadata.yaml`;
* the field-to-record mapping;
* timestamp conversion;
* coordinate conventions that are confirmed by source code;
* coordinate conventions that remain unresolved;
* undocumented masks or status fields;
* assumptions made;
* explicit non-goals of this task.

Include a small table showing:

```text
topic → ROS type → internal record → adapter
```

Also note the lane-detection and object-detection types discovered during inspection, but do not implement their adapters yet.

# Validation

Run the ROS-independent Python test suite.

If the execution environment already supports ROS 2 Foxy, also run the appropriate package build and test commands, such as the repository's documented `colcon build` and `colcon test` commands.

Do not spend this task attempting to install or repair a full ROS 2 Foxy environment in Codex Cloud.

If ROS 2 is unavailable:

* run all ROS-independent tests;
* verify package metadata and Python syntax;
* state clearly that the ROS build was not executed.

# Non-goals

Do not implement:

* rosbag reading or deserialization;
* rosbag playback;
* time synchronization;
* interpolation;
* fixed-rate frame generation;
* GNSS quality scoring;
* lane feature extraction;
* object feature extraction;
* segmentation;
* ODD classification;
* map matching;
* GeoJSON, Parquet, or CSV export;
* visualization;
* online ROS nodes.

Do not add unnecessary dependencies.

# Completion report

At the end, provide:

1. repository structure discovered;
2. custom message packages and exact type names;
3. topic names verified from metadata;
4. lane and object message types discovered;
5. files created or modified;
6. tests and commands executed;
7. test results;
8. assumptions made;
9. unresolved message semantics;
10. anything that could not be validated in the current environment.

Before editing, briefly state the implementation plan. After implementation, inspect the final diff and remove unrelated or unnecessary changes.
