import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import math
import time

class SimpleSimulatorNode(Node):
    def __init__(self):
        super().__init__('simple_simulator')

        # Stato veicolo iniziale
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0

        # Parametri veicolo
        self.wheelbase = 2.5  # metri

        # Comandi attuali
        self.current_steering = 0.0
        self.current_accel = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.steer_sub = self.create_subscription(Float64, '/steering_angle', self.steer_cb, 10)
        self.accel_sub = self.create_subscription(Float64, '/acceleration', self.accel_cb, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.last_time = self.get_clock().now()

    def steer_cb(self, msg):
        self.current_steering = math.radians(msg.data)
        #self.get_logger().info(f'Sterzo ricevuto: {msg.data}° -> {self.current_steering:.2f} rad')

    def accel_cb(self, msg):
        self.current_accel = msg.data
        #self.get_logger().info(f'Accelerazione ricevuta: {self.current_accel} m/s²')

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = max((current_time - self.last_time).nanoseconds / 1e9, 1e-3)  # impone un dt minimo
        self.last_time = current_time

        # Semplice integrazione cinematic bicycle model
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.wheelbase * math.tan(self.current_steering) * dt
        self.v += self.current_accel * dt

        # Costruiamo messaggio Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'oodm'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Converti yaw in quaternion semplice
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Velocità lineare lungo x
        odom_msg.twist.twist.linear.x = self.v

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
