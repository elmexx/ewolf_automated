# system time publish
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import TimeReference

class TimePublisher(Node):
    def __init__(self):
        super().__init__('time_publisher_node')
        self.publisher_ = self.create_publisher(TimeReference, 'system_time', 10)
        self.timer = self.create_timer(0.01, self.publish_time) 

    def publish_time(self):
        now = self.get_clock().now().to_msg()
        #msg_stamp = self.get_clock().now().to_msg()
        msg = TimeReference()
        msg.header.stamp = now
        msg.time_ref = now
        #sec = int(now.nanoseconds // 1e9 )
        #nanosec = int(now.nanoseconds % 1e9)  
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published time: {sec}.{nanosec:09d}')

def main(args=None):
    rclpy.init(args=args)
    node = TimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
