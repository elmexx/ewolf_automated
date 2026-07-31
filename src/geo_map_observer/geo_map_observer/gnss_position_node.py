"""ROS 2 node that observes and validates geographic GNSS fixes."""

import math
import time
from collections import Counter
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_driver_msgs.msg import GnssQuality, GnssStatus
from sensor_msgs.msg import NavSatFix

from geo_map_observer.osm_loader import load_osm_map
from geo_map_observer.junction_classifier import (
    JunctionType, classify_junction_candidates)
from geo_map_observer.junction_lookup import JunctionLookup
from geo_map_observer.road_matcher import (
    DEFAULT_DRIVABLE_HIGHWAY_TYPES,
    RoadMatchLogThrottle,
    candidate_statistics,
    classify_highway_ways,
    match_nearest_road,
)
from geo_map_observer.road_topology import build_road_topology
from geo_map_observer.validation import extract_nav_sat_fix, validate_nav_sat_fix
from geo_map_observer.visualization_exporter import VisualizationExporter


class GnssPositionNode(Node):
    """Receive geographic fixes and report valid positions."""

    def __init__(self) -> None:
        super().__init__('gnss_position_node')

        self.declare_parameter('gnss_fix_topic', '/gnss/fix')
        self.declare_parameter('require_fix_status', False)
        self.declare_parameter('enable_gnss_position_log', False)
        self.declare_parameter('gnss_position_log_interval_sec', 1.0)
        self.declare_parameter('enable_road_match_log', True)
        self.declare_parameter('road_match_log_interval_sec', 1.0)
        self.declare_parameter('enable_status_warning_log', True)
        self.declare_parameter('status_warning_log_interval_sec', 10.0)
        self.declare_parameter('enable_road_matching', True)
        self.declare_parameter('enable_road_topology', True)
        self.declare_parameter('junction_branch_merge_angle_deg', 20.0)
        self.declare_parameter('enable_topology_summary_log', True)
        self.declare_parameter('enable_junction_candidate_log', False)
        self.declare_parameter('enable_junction_classification', True)
        self.declare_parameter('junction_opposite_tolerance_deg', 25.0)
        self.declare_parameter(
            'enable_junction_classification_summary_log', True)
        self.declare_parameter('enable_nearest_junction_lookup', True)
        self.declare_parameter('max_match_distance_m', 20.0)
        self.declare_parameter(
            'drivable_highway_types', list(DEFAULT_DRIVABLE_HIGHWAY_TYPES))
        self.declare_parameter('subscribe_gnss_status', False)
        self.declare_parameter('subscribe_gnss_quality', False)
        self.declare_parameter('map_file', '')
        self.declare_parameter('enable_visualization', False)
        self.declare_parameter(
            'visualization_output_dir', '/tmp/geo_map_observer_visualization')
        self.declare_parameter('visualization_update_interval_sec', 0.5)
        self.declare_parameter('visualization_max_track_points', 10000)
        self.declare_parameter(
            'visualization_export_contextual_highways', True)
        self.declare_parameter('visualization_export_drivable_highways', True)
        self.declare_parameter('visualization_export_topology_candidates', False)
        self.declare_parameter('visualization_follow_vehicle_default', True)

        fix_topic = str(self.get_parameter('gnss_fix_topic').value)
        self._require_fix_status = bool(
            self.get_parameter('require_fix_status').value)
        self._enable_gnss_position_log = bool(
            self.get_parameter('enable_gnss_position_log').value)
        self._gnss_log_interval_sec = float(
            self.get_parameter('gnss_position_log_interval_sec').value)
        self._enable_road_match_log = bool(
            self.get_parameter('enable_road_match_log').value)
        self._road_log_interval_sec = float(
            self.get_parameter('road_match_log_interval_sec').value)
        self._enable_status_warning_log = bool(
            self.get_parameter('enable_status_warning_log').value)
        self._status_log_interval_sec = float(
            self.get_parameter('status_warning_log_interval_sec').value)
        self._enable_road_matching = bool(
            self.get_parameter('enable_road_matching').value)
        self._enable_road_topology = bool(
            self.get_parameter('enable_road_topology').value)
        self._branch_merge_angle_deg = float(
            self.get_parameter('junction_branch_merge_angle_deg').value)
        self._enable_topology_summary_log = bool(
            self.get_parameter('enable_topology_summary_log').value)
        self._enable_junction_candidate_log = bool(
            self.get_parameter('enable_junction_candidate_log').value)
        self._enable_junction_classification = bool(
            self.get_parameter('enable_junction_classification').value)
        self._junction_opposite_tolerance_deg = float(
            self.get_parameter('junction_opposite_tolerance_deg').value)
        self._enable_junction_classification_summary_log = bool(
            self.get_parameter(
                'enable_junction_classification_summary_log').value)
        self._enable_nearest_junction_lookup = bool(
            self.get_parameter('enable_nearest_junction_lookup').value)
        self._max_match_distance_m = float(
            self.get_parameter('max_match_distance_m').value)
        self._drivable_highway_types = tuple(
            str(value) for value in
            self.get_parameter('drivable_highway_types').value)
        self._enable_visualization = bool(
            self.get_parameter('enable_visualization').value)
        self._visualization_output_dir = str(
            self.get_parameter('visualization_output_dir').value)
        self._visualization_interval_sec = float(
            self.get_parameter('visualization_update_interval_sec').value)
        self._visualization_max_track_points = int(
            self.get_parameter('visualization_max_track_points').value)
        for name, value in (
                ('gnss_position_log_interval_sec', self._gnss_log_interval_sec),
                ('road_match_log_interval_sec', self._road_log_interval_sec),
                ('status_warning_log_interval_sec', self._status_log_interval_sec),
                ('max_match_distance_m', self._max_match_distance_m)):
            if value < 0.0:
                raise ValueError('{} must be greater than or equal to zero'.format(name))
        if not 0.0 < self._branch_merge_angle_deg < 90.0:
            message = ('junction_branch_merge_angle_deg must be greater than '
                       '0 and less than 90; received {}'.format(
                           self._branch_merge_angle_deg))
            self.get_logger().error(message)
            raise ValueError(message)
        if not 0.0 < self._junction_opposite_tolerance_deg < 90.0:
            message = ('junction_opposite_tolerance_deg must be greater than '
                       '0 and less than 90; received {}'.format(
                           self._junction_opposite_tolerance_deg))
            self.get_logger().error(message)
            raise ValueError(message)
        if self._visualization_interval_sec <= 0.0:
            message = ('visualization_update_interval_sec must be greater than '
                       'zero; received {}'.format(
                           self._visualization_interval_sec))
            self.get_logger().error(message)
            raise ValueError(message)
        if self._visualization_max_track_points <= 0:
            message = ('visualization_max_track_points must be greater than '
                       'zero; received {}'.format(
                           self._visualization_max_track_points))
            self.get_logger().error(message)
            raise ValueError(message)

        self.messages_received = 0
        self.valid_fixes = 0
        self.invalid_fixes = 0
        self.no_fix_messages = 0
        self.road_match_attempts = 0
        self.road_matches = 0
        self.road_unmatched = 0
        self.road_match_errors = 0
        self.latest_status: Optional[GnssStatus] = None
        self.latest_quality: Optional[GnssQuality] = None
        self.latest_fix = None
        self._last_log_time_ns: Optional[int] = None
        self._last_diagnostic_log_time_ns: Optional[int] = None
        self._road_log_throttle = RoadMatchLogThrottle(
            self._road_log_interval_sec)
        self._counters_logged = False
        self.osm_map = None
        self.drivable_highway_ways = ()
        self.contextual_highway_ways = ()
        self.road_topology = None
        self.classified_junctions = ()
        self.junction_lookup = None
        self.visualization_exporter = None

        map_file = str(self.get_parameter('map_file').value)
        if map_file:
            try:
                self.osm_map = load_osm_map(map_file)
            except Exception as error:
                self.get_logger().error(
                    'Failed to load requested OSM map {!r}: {}'.format(
                        map_file, error))
                raise
            self._log_map_summary()
            (self.drivable_highway_ways,
             self.contextual_highway_ways) = classify_highway_ways(
                 self.osm_map, self._drivable_highway_types)
            if self._enable_road_topology:
                self.road_topology = build_road_topology(
                    self.osm_map, self.drivable_highway_ways,
                    self._branch_merge_angle_deg)
                self._log_topology()
                if self._enable_junction_classification:
                    self._classify_junctions()

        if self._enable_nearest_junction_lookup:
            self.junction_lookup = JunctionLookup(self.classified_junctions)
            self.get_logger().info(
                'Nearest junction lookup initialized:\njunction_count={}'.format(
                    self.junction_lookup.junction_count))

        if self._enable_visualization:
            self._initialize_visualization()

        self._fix_subscription = self.create_subscription(
            NavSatFix, fix_topic, self._fix_callback, 10)
        self._status_subscription = None
        self._quality_subscription = None

        if bool(self.get_parameter('subscribe_gnss_status').value):
            self._status_subscription = self.create_subscription(
                GnssStatus, '/gnss/status', self._status_callback, 10)
        if bool(self.get_parameter('subscribe_gnss_quality').value):
            self._quality_subscription = self.create_subscription(
                GnssQuality, '/gnss/quality', self._quality_callback, 10)

        self.get_logger().info('gnss_fix_topic={}'.format(fix_topic))
        self.get_logger().info(
            'require_fix_status={}'.format(self._require_fix_status))
        self.get_logger().info(
            'road_matching={}, drivable_highway_types={}'.format(
                self._enable_road_matching, self._drivable_highway_types))

    def _log_map_summary(self) -> None:
        """Log one concise summary for the map loaded during startup."""
        data = self.osm_map
        bounds = data.bounding_box
        bounding_box = ('none' if bounds is None else
                        'lat=[{}, {}], lon=[{}, {}]'.format(
                            bounds.min_latitude, bounds.max_latitude,
                            bounds.min_longitude, bounds.max_longitude))
        self.get_logger().info(
            'OSM map loaded: path={}, nodes={}, ways={}, retained_highways={}, '
            'skipped_highways={}, ways_with_missing_refs={}, missing_refs={}, '
            'bounding_box={}, highway_types={}, duration_sec={:.3f}'.format(
                data.path, data.total_node_count, data.total_way_count,
                data.retained_highway_way_count,
                data.skipped_highway_way_count,
                data.ways_with_missing_node_references,
                data.missing_node_reference_count, bounding_box,
                dict(data.highway_type_counts), data.loading_duration_sec))
        statistics = candidate_statistics(data, self._drivable_highway_types)
        self.get_logger().info(
            'OSM highway candidates: all={}, drivable={}, contextual={}, '
            'drivable_types={}, contextual_types={}'.format(
                statistics.all_way_count, statistics.drivable_way_count,
                statistics.contextual_way_count,
                dict(statistics.drivable_type_counts),
                dict(statistics.contextual_type_counts)))

    def _log_topology(self) -> None:
        """Optionally log the startup topology summary and candidates."""
        topology = self.road_topology
        stats = topology.statistics
        if self._enable_topology_summary_log:
            self.get_logger().info(
                'Road topology built: drivable_ways={}, nodes={}, segments={}, '
                'skipped_invalid_segments={}, missing_node_references={}, '
                'zero_length_segments={}, junction_candidates={}, '
                'branch_counts={}, traffic_signal_candidates={}, '
                'stop_nodes={}, give_way_nodes={}, roundabout_ways={}, '
                'circular_junction_ways={}, duration_sec={:.3f}'.format(
                    stats.drivable_way_count, stats.topology_node_count,
                    stats.segment_count, stats.skipped_invalid_segment_count,
                    stats.missing_node_reference_count,
                    stats.zero_length_segment_count,
                    stats.junction_candidate_count,
                    dict(stats.candidate_counts_by_physical_branch_count),
                    stats.traffic_signal_candidate_count,
                    stats.stop_node_count, stats.give_way_node_count,
                    len(stats.roundabout_way_ids),
                    len(stats.circular_junction_way_ids),
                    stats.build_duration_sec))
        if self._enable_junction_candidate_log:
            for candidate in topology.junction_candidates:
                self.get_logger().info(
                    'JunctionCandidate: node_id={} lat={:.8f} lon={:.8f} '
                    'physical_branches={} bearings={} ways={} signals={}'.format(
                        candidate.node_id, candidate.latitude,
                        candidate.longitude, candidate.physical_branch_count,
                        [round(value, 1) for value in
                         candidate.physical_branch_bearings_deg],
                        list(candidate.connected_way_ids),
                        str(candidate.has_traffic_signals).lower()))

    def _classify_junctions(self) -> None:
        """Classify the startup topology exactly once."""
        started = time.monotonic()
        self.classified_junctions = classify_junction_candidates(
            self.road_topology.junction_candidates,
            self._junction_opposite_tolerance_deg)
        duration = time.monotonic() - started
        if self._enable_junction_classification_summary_log:
            counts = Counter(
                junction.junction_type for junction in self.classified_junctions)
            self.get_logger().info(
                'Junction classification completed: candidates={} {} '
                'duration_sec={:.3f}'.format(
                    len(self.classified_junctions), ' '.join(
                        '{}={}'.format(kind.value, counts[kind])
                        for kind in JunctionType), duration))

    def _fix_callback(self, message: NavSatFix) -> None:
        self.messages_received += 1
        position = extract_nav_sat_fix(message)
        self.latest_fix = position
        if not position.status_valid:
            self.no_fix_messages += 1
        valid, reason = validate_nav_sat_fix(
            message, require_fix_status=self._require_fix_status)
        if not valid:
            self.invalid_fixes += 1
            self._log_diagnostic(position, reason, accepted=False)
            return

        self.valid_fixes += 1
        if not position.status_valid:
            self._log_diagnostic(position, reason, accepted=True)
        road_match = None
        if self._enable_road_matching and self.osm_map is not None:
            road_match = self._match_road(position)
        nearest_junction = (None if self.junction_lookup is None else
                            self.junction_lookup.nearest(
                                position.latitude, position.longitude))
        if self.visualization_exporter is not None:
            self.visualization_exporter.observe(
                position, road_match, self._visualization_counters(),
                nearest_junction=nearest_junction)
        if not self._enable_gnss_position_log:
            return
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(self._gnss_log_interval_sec * 1_000_000_000)
        if (self._last_log_time_ns is not None and
                now_ns - self._last_log_time_ns < interval_ns):
            return

        self._last_log_time_ns = now_ns
        altitude = ('nan' if math.isnan(position.altitude)
                    else str(position.altitude))
        self.get_logger().info(
            'GNSS position: lat={:.8f}, lon={:.8f}, altitude={}, frame_id={}'.format(
                position.latitude, position.longitude, altitude,
                position.frame_id))

    def _log_diagnostic(self, position, reason: str, accepted: bool) -> None:
        """Emit a throttled warning for rejected or status-invalid fixes."""
        if not self._enable_status_warning_log:
            return
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(self._status_log_interval_sec * 1_000_000_000)
        if (self._last_diagnostic_log_time_ns is not None and
                now_ns - self._last_diagnostic_log_time_ns < interval_ns):
            return
        self._last_diagnostic_log_time_ns = now_ns
        outcome = 'accepted despite STATUS_NO_FIX' if accepted else 'rejected'
        self.get_logger().warning(
            'GNSS fix {}: status={}, latitude={}, longitude={}, reason={}'.format(
                outcome, position.navsat_status, position.latitude,
                position.longitude, reason))

    def _match_road(self, position):
        """Match and count a valid fix; output throttling happens afterwards."""
        self.road_match_attempts += 1
        try:
            result = match_nearest_road(
                position.timestamp_ns, position.latitude, position.longitude,
                self.drivable_highway_ways, self._max_match_distance_m)
        except Exception as error:
            self.road_match_errors += 1
            self.get_logger().error('Road matching failed: {}'.format(error))
            return None
        if result.matched:
            self.road_matches += 1
        else:
            self.road_unmatched += 1
        if not self._enable_road_match_log:
            return result

        now_ns = self.get_clock().now().nanoseconds
        if not self._road_log_throttle.should_log(result, now_ns):
            return result
        if not result.matched:
            distance = ('unknown' if result.distance_m is None else
                        '{:.1f}m'.format(result.distance_m))
            self.get_logger().info(
                'RoadMatch unmatched distance={} lat={:.8f} lon={:.8f}'.format(
                    distance, result.latitude, result.longitude))
            return result
        road = result.name if result.name is not None else ''
        self.get_logger().info(
            'RoadMatch t={} way={} road="{}" ref={} type={} maxspeed={} '
            'distance={:.1f}m'.format(
                result.timestamp_ns, result.osm_way_id, road,
                result.ref or '', result.highway or '',
                result.maxspeed_raw or '', result.distance_m))
        return result

    def _visualization_counters(self):
        return {
            'gnss_received_count': self.messages_received,
            'match_attempt_count': self.road_match_attempts,
            'matched_count': self.road_matches,
            'unmatched_count': self.road_unmatched,
        }

    def _initialize_visualization(self) -> None:
        """Create browser assets and static data without starting a server."""
        self.visualization_exporter = VisualizationExporter(
            self._visualization_output_dir, self._visualization_interval_sec,
            self._visualization_max_track_points,
            bool(self.get_parameter(
                'visualization_follow_vehicle_default').value))
        share_web = Path(get_package_share_directory('geo_map_observer')) / 'web'
        self.visualization_exporter.initialize(str(share_web))
        counts = self.visualization_exporter.export_highways(
            self.drivable_highway_ways, self.contextual_highway_ways,
            bool(self.get_parameter(
                'visualization_export_drivable_highways').value),
            bool(self.get_parameter(
                'visualization_export_contextual_highways').value))
        junction_count = self.visualization_exporter.export_junctions(
            self.classified_junctions)
        output_dir = str(self.visualization_exporter.output_dir)
        self.get_logger().info(
            'Visualization initialized:\noutput_dir={}\nstatic_osm_features={}\n'
            'drivable_features={}\ncontextual_features={}\n'
            'junction_features={}\nruntime_file=runtime_state.json\n'
            'serve_command="python3 -m '
            'http.server 8080 --directory {}"\nurl=http://localhost:8080/'.format(
                output_dir, counts['total'], counts['drivable'],
                counts['contextual'], junction_count, output_dir))

    def _status_callback(self, message: GnssStatus) -> None:
        self.latest_status = message

    def _quality_callback(self, message: GnssQuality) -> None:
        self.latest_quality = message

    def log_counters(self) -> None:
        """Log final counters once."""
        if self._counters_logged:
            return
        self._counters_logged = True
        self.get_logger().info(
            'GNSS counters: messages_received={}, valid_fixes={}, '
            'invalid_fixes={}, no_fix_messages={}, road_match_attempts={}, '
            'road_matches={}, road_unmatched={}, road_match_errors={}'.format(
                self.messages_received, self.valid_fixes,
                self.invalid_fixes, self.no_fix_messages,
                self.road_match_attempts, self.road_matches,
                self.road_unmatched, self.road_match_errors))

    def destroy_node(self) -> bool:
        if self.visualization_exporter is not None:
            try:
                self.visualization_exporter.write_runtime(
                    counters=self._visualization_counters())
            except Exception as error:
                self.get_logger().warning(
                    'Final visualization write failed: {}'.format(error))
        self.log_counters()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
