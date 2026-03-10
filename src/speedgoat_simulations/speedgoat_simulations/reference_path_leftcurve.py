# path_straight_leftcurve.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__('reference_path')
        self.publisher_ = self.create_publisher(Path, 'reference_path', 20)
        self.timer = self.create_timer(5.0, self.publish_path)
        self.path_msg = self.create_path()

    def create_path(self):
        path = Path()
        path.header.frame_id = 'map'

        # Rettilineo
        num_points_straight = 80
        length_straight = 10.0
        for i in range(num_points_straight):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(length_straight / num_points_straight * i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        # Curva 90° a sinistra
        num_points_curve = 150
        radius = 15.0
        start_x = length_straight
        for i in range(num_points_curve):
            angle = math.pi / 2 * i / num_points_curve
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(start_x + radius * math.sin(angle))
            pose.pose.position.y = float(radius * (1 - math.cos(angle)))
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path_msg)
        self.get_logger().info(f"Percorso pubblicato con {len(self.path_msg.poses)} punti.")

def main(args=None):
    rclpy.init(args=args)
    node = ReferencePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
