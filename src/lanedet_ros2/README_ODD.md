# Optional lane observations

The existing LaneDet node can emit ODD-oriented observations without changing
its default behavior. All optional features default to disabled.

## Parameters

| Parameter | Default | Effect |
|---|---:|---|
| `publish_lane_observation` | `false` | Publish JSON in `std_msgs/msg/String` on `/lane_observation` |
| `publish_debug_overlay` | `false` | Publish an additional annotated image on `/lane_observation/debug_overlay` |
| `save_lane_observation` | `false` | Append each observation to JSON Lines on a background thread |
| `lane_observation_path` | `lane_observations.jsonl` | Destination used by the JSONL writer |

The established `/detection/lane/*` publishers, image payload, inference,
projection, and launch defaults are unchanged. Geometry is evaluated only when
at least one optional output is enabled.

## Launch/CLI example

No existing launch file was changed. Enable outputs through ROS arguments:

```bash
ros2 run lanedet_ros2 lanedet_node --ros-args \
  -p publish_lane_observation:=true \
  -p publish_debug_overlay:=true \
  -p save_lane_observation:=true \
  -p lane_observation_path:=/tmp/lane_observations.jsonl
```

Example topic inspection:

```bash
ros2 topic echo /lane_observation std_msgs/msg/String
ros2 topic echo /lane_observation/debug_overlay sensor_msgs/msg/Image
```

The JSON object includes `header`, `timestamp_ns`, `frame_id`, detection status,
marking count, genuine ego-side availability, every projected marking and its
role, nullable confidence, nullable lane width/curvature/radius, curve direction,
and callback processing time. A no-detection frame is still published with
`status: "not_detected"`, zero markings, and invalid geometry represented by
JSON `null`.

Coordinates reuse `BirdsEyeView.imagetovehicle()` output: `longitudinal` is the
first component, `lateral` the second, left is positive, and the implementation's
camera/world configuration establishes metres. See
`docs/lane_geometry_analysis.md` for evidence and unresolved semantics.
