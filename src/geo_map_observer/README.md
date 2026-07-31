# geo_map_observer

`geo_map_observer` is a ROS 2 Foxy Python package that subscribes to
`sensor_msgs/msg/NavSatFix`, validates geographic coordinates, and periodically
optionally logs valid latitude/longitude observations. It loads road geometry
from an offline OSM XML map and matches every accepted fix to the nearest
configured drivable highway candidate.

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
  -p gnss_fix_topic:=/gnss/fix -p road_match_log_interval_sec:=2.0 \
  -p require_fix_status:=true \
  -p subscribe_gnss_status:=true -p subscribe_gnss_quality:=true
```

Load a map stored outside the package (for example under the workspace's
`maps/` directory):

```bash
ros2 run geo_map_observer gnss_position_node --ros-args \
  -p map_file:=/absolute/path/to/workspace/maps/map.osm \
  -p require_fix_status:=false \
  -p road_match_log_interval_sec:=0.0
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
| `enable_gnss_position_log` | `false` | Enable raw position logs. |
| `gnss_position_log_interval_sec` | `1.0` | Minimum interval between raw position logs. |
| `enable_road_match_log` | `true` | Enable compact road-match logs. |
| `road_match_log_interval_sec` | `1.0` | Repeated-road log interval; way, highway, or speed changes log immediately. |
| `enable_status_warning_log` | `true` | Enable status/invalid-fix warnings. |
| `status_warning_log_interval_sec` | `10.0` | Warning throttle interval. |
| `enable_road_matching` | `true` | Match every accepted fix when a map is loaded. |
| `enable_road_topology` | `true` | Build static drivable-road topology once during map startup. |
| `junction_branch_merge_angle_deg` | `20.0` | Maximum circular bearing difference for merging physical branches; must be greater than 0 and less than 90. |
| `enable_topology_summary_log` | `true` | Print one compact topology summary after construction. |
| `enable_junction_candidate_log` | `false` | Print one debug line per candidate junction. |
| `enable_junction_classification` | `true` | Classify topology candidates once at startup; requires topology. |
| `junction_opposite_tolerance_deg` | `25.0` | Maximum deviation from 180 degrees for opposite branches; must be greater than 0 and less than 90. |
| `enable_junction_classification_summary_log` | `true` | Print one compact startup classification summary. |
| `max_match_distance_m` | `20.0` | Maximum distance for a successful match. |
| `drivable_highway_types` | major road types, `residential`, `living_street`, `service` | Highway values eligible for vehicle matching. |
| `subscribe_gnss_status` | `false` | Cache `/gnss/status` messages. |
| `subscribe_gnss_quality` | `false` | Cache `/gnss/quality` messages. |
| `map_file` | `""` | Offline `.osm` XML file to load; empty disables map loading. |

## Offline OSM loading

`geo_map_observer.osm_loader` uses the Python standard library's streaming XML
parser. It retains every OSM node and every way tagged `highway` that resolves
to at least two coordinates, including ordered node references, coordinates,
and the optional `name`, `ref`, `maxspeed`, `lanes`, and `oneway` tags. Non-highway
ways are ignored. Missing references and skipped highway ways are counted and
reported in a single startup summary together with the map bounds and highway
type counts.

All retained highways stay in `OsmMapData`. Candidate classification only
selects which ways the vehicle matcher searches. Footways, cycleways, paths,
tracks, steps, pedestrian ways, platforms, and corridors are contextual by
default; users may explicitly add values such as `track` or `path`. Startup
logs report totals and per-type counts for both groups. Matching uses a local
metric projection and point-to-segment distance. It does not infer access from
other OSM tags, vehicle heading, or route continuity.

## Static road topology and junction candidates

When a map is loaded and `enable_road_topology` is true, the node builds an
in-memory topology once at startup from the same configured drivable ways used
by road matching. It does not wait for `/gnss/fix`, and accepted fixes continue
to use the Task 4 matcher after topology construction. Setting the parameter to
false skips this work without disabling matching.

An OSM way is an ordered geometry and tagging unit, not necessarily one
physical road. Mappers commonly split a continuous road into multiple ways
where `maxspeed`, `lanes`, `name`, `ref`, `surface`, or `oneway` changes.
Consequently, connected-way count alone cannot identify a junction. The
topology creates a segment for each valid consecutive node pair and examines
the direction leaving each shared node. Bearings within
`junction_branch_merge_angle_deg` (including across north's 0/360-degree
boundary) are grouped into one physical branch. A node becomes a static
candidate only when at least three physical outgoing branches remain.
Underlying ways and segments are retained unchanged.

The startup summary reports topology size, invalid/missing/zero-length segment
counts, candidates by branch count, retained traffic controls, and explicit
roundabout/circular way counts. Per-candidate output is intentionally disabled
by default. Relevant OSM node tags (`highway=traffic_signals|stop|give_way`,
`junction`, `crossing`, and `traffic_signals`) are retained; unrelated node
tags are not.

Topology tests need neither ROS nor a rosbag:

```bash
python3 -m pytest src/geo_map_observer/test/test_road_topology.py -q
```

### Static junction classification

When both topology and junction classification are enabled, every extracted
candidate is classified once at startup using only its physical branch
bearings. Three branches are a `T_JUNCTION` when any pair is approximately
opposite and a `Y_JUNCTION` otherwise. Four branches are a
`CROSS_INTERSECTION` when one of the three possible pairings contains two
approximately opposite pairs; other valid four-way shapes remain
`FOUR_WAY_UNKNOWN`. Five or more branches are `MULTI_WAY`. Invalid three-way
inputs remain `THREE_WAY_UNKNOWN`. Opposite means within
`junction_opposite_tolerance_deg` of 180 degrees; a cross need not be
perpendicular. Disabling classification leaves topology and road matching
enabled, while disabling topology necessarily skips classification.

Current limitations are intentional: connectivity exists only where ways
share an OSM node ID. The package does not find geometric crossings, score
confidence, explain classifications, classify roundabouts, determine whether
the vehicle passes a candidate, infer maneuvers, use heading/route continuity,
or model a whole roundabout as one node.

## Test

The validation tests use synthetic NavSatFix-shaped Python objects and do not
require ROS 2:

```bash
python3 -m pytest src/geo_map_observer/test
```

## Local GNSS and road-match visualization

The optional initial browser viewer makes nearest-road behavior observable
without changing matching or starting a server. It exports the classified OSM
vectors once and atomically refreshes a bounded runtime track. Visualization is
disabled by default.

Set these YAML parameters (the remaining visualization settings are documented
in the parameter table below):

```yaml
enable_visualization: true
visualization_output_dir: "/tmp/geo_map_observer_visualization"
```

Then:

1. Start the ROS node with the YAML configuration and a valid `map_file`.
2. Run `python3 -m http.server 8080 --directory /tmp/geo_map_observer_visualization`.
3. Open <http://localhost:8080/>.
4. Publish valid `/gnss/fix` messages or play a rosbag containing them.

The Leaflet layer control switches drivable/contextual vectors, static
`Junctions` markers, dashed raw GNSS
track, heavier matched track, latest markers, the GNSS-to-road error line, and
the strongly highlighted current way. The information panel shows fix values,
road tags, nearest point, distance, and counters; click a road for all exported
attributes. **Follow vehicle** pans only when the marker approaches the visible
edge and preserves zoom. Disable it to pan freely.

The standard OpenStreetMap raster background requires internet access. Exported
OSM vector roads and tracking data remain visible if raster tiles fail to load
(the Leaflet library itself is also loaded from a public CDN in this initial
version). `runtime_state.json` is polled every 500 ms without overlapping
requests; stale data remains displayed during a failed request.

Runtime writes are limited by `visualization_update_interval_sec`, but every
accepted fix is still matched, counted, and accumulated. A matched/unmatched,
way-ID, or highway change is written immediately. Each raw and matched track is
independently capped at `visualization_max_track_points`. Shutdown attempts one
final atomic write and retains the output directory for inspection.

| Visualization parameter | Default | Description |
| --- | --- | --- |
| `enable_visualization` | `false` | Generate local viewer files. |
| `visualization_output_dir` | `/tmp/geo_map_observer_visualization` | Viewer and data output directory. |
| `visualization_update_interval_sec` | `0.5` | Regular runtime write interval; must be greater than zero. |
| `visualization_max_track_points` | `10000` | Per-track bounded history; must be greater than zero. |
| `visualization_export_contextual_highways` | `true` | Include contextual roads in static GeoJSON. |
| `visualization_export_drivable_highways` | `true` | Include drivable roads in static GeoJSON. |
| `visualization_export_topology_candidates` | `false` | Reserved for a future topology overlay; no candidates are exported in Task 6. |
| `visualization_follow_vehicle_default` | `true` | Initial browser follow-control state. |

`osm_highways.geojson` is a GeoJSON `FeatureCollection`; each unique LineString
uses `[longitude, latitude]` coordinates and has `way_id`, `highway`,
`road_group`, `name`, `ref`, `maxspeed_raw`, `maxspeed_kmh`, `lanes_raw`,
`oneway_raw`, and `junction` properties. `runtime_state.json` contains wall-clock
`updated_at_ns`, four counters, `follow_vehicle_default`, nullable `latest_gnss`,
a `latest_match` object, and bounded `gnss_track`/`matched_track` arrays.

When visualization is enabled, startup also writes `junctions.geojson` once,
even when classification is disabled or produces no results. Each classified
junction is a GeoJSON Point with longitude before latitude and includes its
type, node ID, physical branch count and bearings, connected way IDs, and
traffic-signal flag. The browser `Junctions` layer uses compact `T`, `Y`, `+`,
unknown, or branch-count markers with the same details in a popup. Runtime GNSS
writes never rewrite this static file.

Current limitations: there is no heading/continuity matching, trajectory
simplification, offline Leaflet/tile bundle, roundabout visualization,
branch-direction ray overlay, WebSocket transport, or remote hosting. The
viewer is intended for a trusted local HTTP server; it has no authentication.
