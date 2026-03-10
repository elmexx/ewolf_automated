# multi topic delay
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from message_filters import Subscriber, ApproximateTimeSynchronizer
from lane_parameter_msg.msg import LaneParams
from sensor_msgs.msg import TimeReference

class MultiTopicDelayCalculator(Node):
    def __init__(self):
        super().__init__('multi_topic_delay_calculator')

        self.system_time_sub = Subscriber(self, TimeReference, 'system_time')
        self.camera_image_sub = Subscriber(self, SensorImage, '/camera/color/image_raw')
        self.lane_info_sub = Subscriber(self, LaneParams, '/detection/lane/leftlanedetection')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.system_time_sub, self.camera_image_sub, self.lane_info_sub],
            queue_size=100,
            slop=0.2 
        )
        self.sync.registerCallback(self.sync_callback)

    def sync_callback(self, system_time_msg, camera_msg, lane_msg):
        #self.get_logger().info(f'Get All Message')
        system_time = rclpy.time.Time.from_msg(system_time_msg.header.stamp)
        camera_time = rclpy.time.Time.from_msg(camera_msg.header.stamp)
        lane_time = rclpy.time.Time.from_msg(lane_msg.header.stamp)

        delay_camera_to_system = (camera_time - system_time).nanoseconds / 1e6  
        delay_lane_to_system = (lane_time - system_time).nanoseconds / 1e6  
        delay_camera_to_lane = (camera_time - lane_time).nanoseconds / 1e6  

        self.get_logger().info(f'Delay (Camera Image-> System): {delay_camera_to_system:.3f} ms')
        self.get_logger().info(f'Delay (Lane -> System): {delay_lane_to_system:.3f} ms')
        self.get_logger().info(f'Delay (Speedgoat -> Lane): {delay_camera_to_lane:.3f} ms')

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicDelayCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
