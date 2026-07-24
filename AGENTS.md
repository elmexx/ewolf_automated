# Project Instructions

## Project goal

Build a modular ROS 2 Foxy pipeline for extracting Operational Design
Domain information from recorded rosbag2 data.

The project should not directly classify the complete ODD from raw messages.
It must use the following processing hierarchy:

1. L0 data validation
2. L1 message standardization
3. L2 temporal synchronization
4. L3 continuous feature extraction
5. L4 segment aggregation
6. L5 ODD attribute classification

## Runtime environment

- Ubuntu 20.04
- ROS 2 Foxy
- Python 3.8
- rosbag2 SQLite3 storage
- Custom ROS message packages may not be installed in the Codex environment

The core analysis logic must therefore be separated from ROS-specific code.

## Architecture rules

Use these layers:

- bag_reader: reading rosbag2 messages
- adapters: converting ROS messages into internal models
- synchronization: creating a fixed-rate timeline
- features: calculating continuous features
- segmentation: aggregating frames into route segments
- classification: assigning ODD attributes
- exporters: writing Parquet, CSV and JSON outputs

Do not put all logic into one script.

## Internal data models

ROS messages must be converted immediately into plain Python dataclasses or
Pydantic models.

Core logic must not depend directly on generated ROS message classes.

## Initial scope

Implement only GNSS and IMU status processing first.

Input topics:

- /gnss/ecef
- /gnss/ned
- /gnss/quality
- /gnss/status
- /imu/status
- /gnss/fix
- /imu/data
- /odometry/filtered

Initial outputs:

- topic_statistics.csv
- data_gaps.csv
- ego_state.parquet
- localization_quality.parquet
- frame_table.parquet
- segment_table.parquet

## Coding requirements

- Python 3.8 compatible
- Type annotations required
- Use pathlib
- Add logging
- Add unit tests
- Avoid hard-coded topic names
- Configuration must use YAML
- Each module should have a clear responsibility
- Handle missing topics and invalid values without crashing
- Never silently interpolate across long data gaps

## Testing

Tests must not require ROS 2 or a real rosbag.

Create synthetic internal message objects for unit tests.

ROS integration tests may be placed in a separate optional test directory.

## Deliverable workflow

Before implementing a large feature:

1. Inspect existing files.
2. Write or update the architecture documentation.
3. Propose a short implementation plan.
4. Implement one layer at a time.
5. Run tests.
6. Summarize changed files and remaining limitations.
