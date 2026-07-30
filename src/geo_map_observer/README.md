# geo_map_observer

`geo_map_observer` is a ROS 2 Foxy Python package that subscribes to
`sensor_msgs/msg/NavSatFix`, validates geographic coordinates, and periodically
logs valid latitude/longitude observations. It can also load road geometry from
an offline OSM XML map at startup. Map loading remains independent of GNSS
callbacks; nearest-road matching is not implemented.

By default the node listens on `/gnss/fix`. It can optionally cache the latest
`sensor_driver_msgs/msg/GnssStatus` and `GnssQuality` messages without
interpreting their fields.

By default, finite in-range coordinates are accepted even when
`NavSatFix.status.status` is `STATUS_NO_FIX` (`-1`). Such positions retain the
raw status, are marked `status_valid=false`, and produce a throttled warning.
Set `require_fix_status: true` to restore strict rejection of `STATUS_NO_FIX`.

## Build and source

From the workspace root:

```bash
colcon build --packages-select geo_map_observer
source install/setup.bash
```

## Run

Run with parameter defaults:

```bash
ros2 run geo_map_observer gnss_position_node
```

Or use the installed YAML configuration:

```bash
ros2 launch geo_map_observer geo_map_observer.launch.py
```

Parameters can be overridden on the command line, for example:

```bash
ros2 run geo_map_observer gnss_position_node --ros-args \
  -p gnss_fix_topic:=/gnss/fix -p log_interval_sec:=2.0 \
  -p require_fix_status:=true \
  -p subscribe_gnss_status:=true -p subscribe_gnss_quality:=true
```

Load a map stored outside the package (for example under the workspace's
`maps/` directory):

```bash
ros2 run geo_map_observer gnss_position_node --ros-args \
  -p map_file:=/absolute/path/to/workspace/maps/map.osm \
  -p require_fix_status:=false \
  -p log_interval_sec:=0.0
```

The loader expands `~`, resolves the path, and requires an existing regular
file with a `.osm` extension. A requested map that cannot be parsed stops node
startup with an error. The source map is never copied into the installed ROS
package. The launch file accepts the same setting as
`map_file:=/absolute/path/to/workspace/maps/map.osm`.

## Use with a rosbag

Start the observer in one sourced terminal. In another sourced terminal, play a
bag that contains `/gnss/fix`:

```bash
ros2 bag play /path/to/bag
```

The sample metadata in this repository lists `/gnss/fix` as
`sensor_msgs/msg/NavSatFix`; its message count is zero, so that sample cannot
demonstrate live position logging. Use a bag with actual fix messages or the
running GNSS driver.

## Inspect `/gnss/fix` manually

```bash
ros2 topic info /gnss/fix --verbose
ros2 topic echo /gnss/fix
```

The node always rejects non-finite latitude/longitude values and values outside
`[-90, 90]` latitude or `[-180, 180]` longitude, including when status is
`STATUS_NO_FIX`. In permissive mode (the default), it accepts valid coordinates
with `STATUS_NO_FIX`; strict mode rejects them. Altitude is not used for
validation, so both NaN and `0.0` are accepted.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `gnss_fix_topic` | `/gnss/fix` | `NavSatFix` input topic. |
| `require_fix_status` | `false` | Reject `STATUS_NO_FIX` when enabled. |
| `log_interval_sec` | `1.0` | Minimum interval between logs of each throttled category. |
| `subscribe_gnss_status` | `false` | Cache `/gnss/status` messages. |
| `subscribe_gnss_quality` | `false` | Cache `/gnss/quality` messages. |
| `map_file` | `""` | Offline `.osm` XML file to load; empty disables map loading. |

## Offline OSM loading

`geo_map_observer.osm_loader` uses the Python standard library's streaming XML
parser. It retains every OSM node and every way tagged `highway` that resolves
to at least two coordinates, including ordered node references, coordinates,
and the optional `name`, `maxspeed`, `lanes`, and `oneway` tags. Non-highway
ways are ignored. Missing references and skipped highway ways are counted and
reported in a single startup summary together with the map bounds and highway
type counts.

## Test

The validation tests use synthetic NavSatFix-shaped Python objects and do not
require ROS 2:

```bash
python3 -m pytest src/geo_map_observer/test
```
