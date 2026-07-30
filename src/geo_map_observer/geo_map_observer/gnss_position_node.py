"""ROS 2 node that observes and validates geographic GNSS fixes."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_driver_msgs.msg import GnssQuality, GnssStatus
from sensor_msgs.msg import NavSatFix

from geo_map_observer.osm_loader import load_osm_map
from geo_map_observer.road_matcher import (
    DEFAULT_DRIVABLE_HIGHWAY_TYPES,
    RoadMatchLogThrottle,
    candidate_statistics,
    classify_highway_ways,
    match_nearest_road,
)
from geo_map_observer.validation import extract_nav_sat_fix, validate_nav_sat_fix


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
        self.declare_parameter('max_match_distance_m', 20.0)
        self.declare_parameter(
            'drivable_highway_types', list(DEFAULT_DRIVABLE_HIGHWAY_TYPES))
        self.declare_parameter('subscribe_gnss_status', False)
        self.declare_parameter('subscribe_gnss_quality', False)
        self.declare_parameter('map_file', '')

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
        self._max_match_distance_m = float(
            self.get_parameter('max_match_distance_m').value)
        self._drivable_highway_types = tuple(
            str(value) for value in
            self.get_parameter('drivable_highway_types').value)
        for name, value in (
                ('gnss_position_log_interval_sec', self._gnss_log_interval_sec),
                ('road_match_log_interval_sec', self._road_log_interval_sec),
                ('status_warning_log_interval_sec', self._status_log_interval_sec),
                ('max_match_distance_m', self._max_match_distance_m)):
            if value < 0.0:
                raise ValueError('{} must be greater than or equal to zero'.format(name))

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
            self.drivable_highway_ways, _ = classify_highway_ways(
                self.osm_map, self._drivable_highway_types)

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
        if self._enable_road_matching and self.osm_map is not None:
            self._match_road(position)
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

    def _match_road(self, position) -> None:
        """Match and count a valid fix; output throttling happens afterwards."""
        self.road_match_attempts += 1
        try:
            result = match_nearest_road(
                position.timestamp_ns, position.latitude, position.longitude,
                self.drivable_highway_ways, self._max_match_distance_m)
        except Exception as error:
            self.road_match_errors += 1
            self.get_logger().error('Road matching failed: {}'.format(error))
            return
        if result.matched:
            self.road_matches += 1
        else:
            self.road_unmatched += 1
        if not self._enable_road_match_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if not self._road_log_throttle.should_log(result, now_ns):
            return
        if not result.matched:
            distance = ('unknown' if result.distance_m is None else
                        '{:.1f}m'.format(result.distance_m))
            self.get_logger().info(
                'RoadMatch unmatched distance={} lat={:.8f} lon={:.8f}'.format(
                    distance, result.latitude, result.longitude))
            return
        road = result.name if result.name is not None else ''
        self.get_logger().info(
            'RoadMatch t={} way={} road="{}" ref={} type={} maxspeed={} '
            'distance={:.1f}m'.format(
                result.timestamp_ns, result.osm_way_id, road,
                result.ref or '', result.highway or '',
                result.maxspeed_raw or '', result.distance_m))

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
