"""ROS 2 node that observes and validates geographic GNSS fixes."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_driver_msgs.msg import GnssQuality, GnssStatus
from sensor_msgs.msg import NavSatFix

from geo_map_observer.validation import extract_nav_sat_fix, validate_nav_sat_fix


class GnssPositionNode(Node):
    """Receive geographic fixes and report valid positions."""

    def __init__(self) -> None:
        super().__init__('gnss_position_node')

        self.declare_parameter('gnss_fix_topic', '/gnss/fix')
        self.declare_parameter('require_fix_status', False)
        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('subscribe_gnss_status', False)
        self.declare_parameter('subscribe_gnss_quality', False)

        fix_topic = str(self.get_parameter('gnss_fix_topic').value)
        self._require_fix_status = bool(
            self.get_parameter('require_fix_status').value)
        self._log_interval_sec = float(
            self.get_parameter('log_interval_sec').value)
        if self._log_interval_sec < 0.0:
            raise ValueError('log_interval_sec must be greater than or equal to zero')

        self.messages_received = 0
        self.valid_fixes = 0
        self.invalid_fixes = 0
        self.no_fix_messages = 0
        self.latest_status: Optional[GnssStatus] = None
        self.latest_quality: Optional[GnssQuality] = None
        self.latest_fix = None
        self._last_log_time_ns: Optional[int] = None
        self._last_diagnostic_log_time_ns: Optional[int] = None
        self._counters_logged = False

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
            'log_interval_sec={}'.format(self._log_interval_sec))

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
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(self._log_interval_sec * 1_000_000_000)
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
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(self._log_interval_sec * 1_000_000_000)
        if (self._last_diagnostic_log_time_ns is not None and
                now_ns - self._last_diagnostic_log_time_ns < interval_ns):
            return
        self._last_diagnostic_log_time_ns = now_ns
        outcome = 'accepted despite STATUS_NO_FIX' if accepted else 'rejected'
        self.get_logger().warning(
            'GNSS fix {}: status={}, latitude={}, longitude={}, reason={}'.format(
                outcome, position.navsat_status, position.latitude,
                position.longitude, reason))

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
            'invalid_fixes={}, no_fix_messages={}'.format(
                self.messages_received, self.valid_fixes,
                self.invalid_fixes, self.no_fix_messages))

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
