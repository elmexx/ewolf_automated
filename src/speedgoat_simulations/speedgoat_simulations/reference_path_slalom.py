# path_slalom.py
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__('reference_path')
        self.publisher_ = self.create_publisher(Path, 'reference_path', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 20)

        self.timer = self.create_timer(1.0, self.publish_path_once)
        self.has_published = False
        self.current_pose_x = 0
        self.current_pose_y = 0

    def odom_callback(self, odom_msg):
        current_pose = odom_msg.pose.pose
        self.current_pose_x = current_pose.position.x
        self.current_pose_y = current_pose.position.y
        return current_pose

    def publish_path_once(self):
        if self.has_published:
            return
        
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        length = 80.0
        num_points = 800

        # current_pose = self.odom_callback(odom_msg)

        for i in range(num_points):
            x = self.current_pose_x + length * i / num_points
            y = self.current_pose_y + 3.0 * math.sin(0.15 * x)
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher_.publish(path)
        self.get_logger().info(f"Path pubblicato con {len(path.poses)} punti.")
        self.has_published = True

def main(args=None):
    rclpy.init(args=args)
    node = ReferencePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
