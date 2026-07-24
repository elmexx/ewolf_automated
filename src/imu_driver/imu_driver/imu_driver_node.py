import math
import socket
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from sensor_driver_msgs.msg import ImuStatus


IMU_STRUCT = struct.Struct("<H2x12f")


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver_node")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("port", 10113)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("angles_in_degrees", True)
        self.declare_parameter("gyro_in_degrees", False)

        self.bind_ip = self.get_parameter("bind_ip").value
        self.port = int(self.get_parameter("port").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.angles_in_degrees = bool(self.get_parameter("angles_in_degrees").value)
        self.gyro_in_degrees = bool(self.get_parameter("gyro_in_degrees").value)

        self.pub_imu = self.create_publisher(Imu, "/imu/data", 50)
        self.pub_mag = self.create_publisher(MagneticField, "/imu/mag", 50)
        self.pub_euler = self.create_publisher(Vector3Stamped, "/imu/euler", 50)
        self.pub_status = self.create_publisher(ImuStatus, "/imu/status", 50)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.receive)

        self.get_logger().info(
            f"IMU driver started: {self.bind_ip}:{self.port}, expected packet size={IMU_STRUCT.size}"
        )

    def receive(self):
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return

        if len(data) != IMU_STRUCT.size:
            self.get_logger().warn(
                f"IMU packet size mismatch: got {len(data)}, expected {IMU_STRUCT.size}"
            )
            return

        v = IMU_STRUCT.unpack(data)

        mask = v[0]

        head = v[1]
        pitch = v[2]
        roll = v[3]

        mag_x = v[4]
        mag_y = v[5]
        mag_z = v[6]

        acc_x = v[7]
        acc_y = v[8]
        acc_z = v[9]

        gyro_x = v[10]
        gyro_y = v[11]
        gyro_z = v[12]

        accel_valid = (mask & 0x000F) != 0
        ypr_valid = (mask & 0x00F0) != 0
        mag_valid = (mask & 0x0F00) != 0
        gyro_valid = (mask & 0xF000) != 0

        if self.angles_in_degrees:
            yaw = math.radians(head)
            pitch_rad = math.radians(pitch)
            roll_rad = math.radians(roll)
        else:
            yaw = head
            pitch_rad = pitch
            roll_rad = roll

        if self.gyro_in_degrees:
            gyro_x = math.radians(gyro_x)
            gyro_y = math.radians(gyro_y)
            gyro_z = math.radians(gyro_z)

        stamp = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id

        if ypr_valid:
            qx, qy, qz, qw = quaternion_from_euler(roll_rad, pitch_rad, yaw)
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
        else:
            imu.orientation_covariance[0] = -1.0

        if accel_valid:
            imu.linear_acceleration.x = acc_x
            imu.linear_acceleration.y = acc_y
            imu.linear_acceleration.z = acc_z
        else:
            imu.linear_acceleration_covariance[0] = -1.0

        if gyro_valid:
            imu.angular_velocity.x = gyro_x
            imu.angular_velocity.y = gyro_y
            imu.angular_velocity.z = gyro_z
        else:
            imu.angular_velocity_covariance[0] = -1.0

        self.pub_imu.publish(imu)

        euler = Vector3Stamped()
        euler.header.stamp = stamp
        euler.header.frame_id = self.frame_id
        euler.vector.x = head
        euler.vector.y = pitch
        euler.vector.z = roll
        self.pub_euler.publish(euler)

        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x = mag_x
        mag.magnetic_field.y = mag_y
        mag.magnetic_field.z = mag_z
        self.pub_mag.publish(mag)

        status = ImuStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.frame_id
        status.mask = mask
        status.accel_valid = accel_valid
        status.ypr_valid = ypr_valid
        status.mag_valid = mag_valid
        status.gyro_valid = gyro_valid
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
