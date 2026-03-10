# path_t_junction_right.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__('reference_path')
        self.publisher_ = self.create_publisher(Path, 'reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.path_msg = self.create_path()

    def create_path(self):
        path = Path()
        path.header.frame_id = 'map'

        # Parametri parabola
        a = -0.1
        h = 15
        k = 40

        # Genera punti
        x_vals = np.linspace(0, 30, 500)
        y_vals = a * (x_vals - h) ** 2 + k

        for x, y in zip(x_vals, y_vals):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # orientamento neutro
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
