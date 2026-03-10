import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__('reference_path')
        self.publisher_ = self.create_publisher(Path, 'reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.path_msg = self.create_path()

    def generate_combined_path(self):
        path_points = []

        # --- Tratto lento: rettilineo orizzontale ---
        x_slow = np.linspace(0, 20, 100)
        y_slow = np.zeros_like(x_slow)
        path_points += list(zip(x_slow, y_slow))

        # --- Tratto veloce: curve dolci sinusoidali ---
        x_fast = np.linspace(0, 40, 200)
        y_fast = 3.0 * np.sin(0.15 * x_fast)
        x_fast_shifted = x_fast + x_slow[-1] + 5.0  # separazione da tratto lento
        path_points += list(zip(x_fast_shifted, y_fast))

        return path_points

    def create_path(self):
        path = Path()
        path.header.frame_id = 'map'

        points = self.generate_combined_path()
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # Orientamento (yaw → quaternion)
            if i < len(points) - 1:
                dx = points[i + 1][0] - x
                dy = points[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path_msg)
        self.get_logger().info(f"Traiettoria pubblicata con {len(self.path_msg.poses)} punti")

def main(args=None):
    rclpy.init(args=args)
    node = ReferencePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
