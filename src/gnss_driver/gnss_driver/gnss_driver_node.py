import socket
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from sensor_driver_msgs.msg import GnssStatus, GnssQuality, GnssEcef, GnssNed


GPS_STRUCT = struct.Struct("<ddH16di8d8dii7ddi")


class GnssDriverNode(Node):
    def __init__(self):
        super().__init__("gnss_driver_node")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("port", 10110)
        self.declare_parameter("frame_id", "gnss_link")

        self.bind_ip = self.get_parameter("bind_ip").value
        self.port = int(self.get_parameter("port").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.pub_fix = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.pub_vel = self.create_publisher(TwistStamped, "/gnss/velocity", 10)
        self.pub_status = self.create_publisher(GnssStatus, "/gnss/status", 10)
        self.pub_quality = self.create_publisher(GnssQuality, "/gnss/quality", 10)
        self.pub_ecef = self.create_publisher(GnssEcef, "/gnss/ecef", 10)
        self.pub_ned = self.create_publisher(GnssNed, "/gnss/ned", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.receive)

        self.get_logger().info(
            f"GNSS driver started: {self.bind_ip}:{self.port}, expected packet size={GPS_STRUCT.size}"
        )

    def receive(self):
        try:
            data, addr = self.sock.recvfrom(2048)
        except BlockingIOError:
            return

        if len(data) != GPS_STRUCT.size:
            self.get_logger().warn(
                f"GNSS packet size mismatch: got {len(data)}, expected {GPS_STRUCT.size}"
            )
            return

        v = GPS_STRUCT.unpack(data)

        sensor_time = v[0]
        online = v[1]
        mask = v[2]

        latitude = v[3]
        longitude = v[4]
        altitude = v[5]
        speed = v[6]
        climb = v[7]

        time_error = v[8]
        latitude_error = v[9]
        longitude_error = v[10]
        altitude_error = v[11]
        speed_error = v[12]
        climb_error = v[13]
        pos2d_error = v[14]
        pos3d_error = v[15]

        mag_var = v[16]
        mag_track = v[17]
        geoid_sep = v[18]

        ecef_station = v[19]
        ecef_pos_x = v[20]
        ecef_pos_y = v[21]
        ecef_pos_z = v[22]
        ecef_speed_x = v[23]
        ecef_speed_y = v[24]
        ecef_speed_z = v[25]
        ecef_pos_error = v[26]
        ecef_speed_error = v[27]

        ned_rel_pos_length = v[28]
        ned_rel_pos_heading = v[29]
        ned_pos_n = v[30]
        ned_pos_e = v[31]
        ned_pos_d = v[32]
        ned_speed_n = v[33]
        ned_speed_e = v[34]
        ned_speed_d = v[35]

        satellites_used = v[36]
        satellites_visible = v[37]

        xdop = v[38]
        ydop = v[39]
        pdop = v[40]
        hdop = v[41]
        vdop = v[42]
        tdop = v[43]
        gdop = v[44]

        dgps_age = v[45]
        dgps_station = v[46]

        stamp = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self.frame_id
        fix.status.status = NavSatStatus.STATUS_FIX if online > 0.5 else NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude
        fix.position_covariance[0] = latitude_error ** 2
        fix.position_covariance[4] = longitude_error ** 2
        fix.position_covariance[8] = altitude_error ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = self.frame_id
        vel.twist.linear.x = speed
        vel.twist.linear.z = climb
        self.pub_vel.publish(vel)

        status = GnssStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.frame_id
        status.sensor_time = sensor_time
        status.online = online
        status.mask = mask
        status.satellites_used = satellites_used
        status.satellites_visible = satellites_visible
        status.dgps_station = dgps_station
        status.dgps_age = dgps_age
        self.pub_status.publish(status)

        quality = GnssQuality()
        quality.header.stamp = stamp
        quality.header.frame_id = self.frame_id
        quality.time_error = time_error
        quality.latitude_error = latitude_error
        quality.longitude_error = longitude_error
        quality.altitude_error = altitude_error
        quality.speed_error = speed_error
        quality.climb_error = climb_error
        quality.pos2d_error = pos2d_error
        quality.pos3d_error = pos3d_error
        quality.xdop = xdop
        quality.ydop = ydop
        quality.pdop = pdop
        quality.hdop = hdop
        quality.vdop = vdop
        quality.tdop = tdop
        quality.gdop = gdop
        self.pub_quality.publish(quality)

        ecef = GnssEcef()
        ecef.header.stamp = stamp
        ecef.header.frame_id = self.frame_id
        ecef.station = ecef_station
        ecef.pos.x = ecef_pos_x
        ecef.pos.y = ecef_pos_y
        ecef.pos.z = ecef_pos_z
        ecef.velocity.x = ecef_speed_x
        ecef.velocity.y = ecef_speed_y
        ecef.velocity.z = ecef_speed_z
        ecef.pos_error = ecef_pos_error
        ecef.speed_error = ecef_speed_error
        self.pub_ecef.publish(ecef)

        ned = GnssNed()
        ned.header.stamp = stamp
        ned.header.frame_id = self.frame_id
        ned.rel_pos_length = ned_rel_pos_length
        ned.rel_pos_heading = ned_rel_pos_heading
        ned.rel_pos.x = ned_pos_n
        ned.rel_pos.y = ned_pos_e
        ned.rel_pos.z = ned_pos_d
        ned.rel_speed.x = ned_speed_n
        ned.rel_speed.y = ned_speed_e
        ned.rel_speed.z = ned_speed_d
        self.pub_ned.publish(ned)


def main(args=None):
    rclpy.init(args=args)
    node = GnssDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
