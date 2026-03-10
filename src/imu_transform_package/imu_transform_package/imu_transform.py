import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations

class ImuTransform(Node):
    def __init__(self):
        super().__init__('imu_transform')
        self.subscription = self.create_subscription(
            Imu,
            '/camera/imu',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, '/imu_enu', 10)

    def imu_callback(self, msg):
        transformed_imu = Imu()
        transformed_imu.header = msg.header

        # Transform linear acceleration
        transformed_imu.linear_acceleration.x = msg.linear_acceleration.x
        transformed_imu.linear_acceleration.y = msg.linear_acceleration.z
        transformed_imu.linear_acceleration.z = -msg.linear_acceleration.y
        transformed_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance  # Preserve covariance

        # Transform angular velocity
        transformed_imu.angular_velocity.x = msg.angular_velocity.x
        transformed_imu.angular_velocity.y = msg.angular_velocity.z
        transformed_imu.angular_velocity.z = -msg.angular_velocity.y
        transformed_imu.angular_velocity_covariance = msg.angular_velocity_covariance  # Preserve covariance

        # Transform orientation (assuming the orientation is quaternion)
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # Rotate 
        rotation1 = tf_transformations.quaternion_from_euler(3.14159, 0, 0)
        rotation2 = tf_transformations.quaternion_from_euler(0, 0, 1.5708)

        q_tmp = tf_transformations.quaternion_multiply(rotation1, q)
        q_transformed = tf_transformations.quaternion_multiply(rotation2, q_tmp)
        transformed_imu.orientation.x = q_transformed[0]
        transformed_imu.orientation.y = q_transformed[1]
        transformed_imu.orientation.z = q_transformed[2]
        transformed_imu.orientation.w = q_transformed[3]
        transformed_imu.orientation_covariance = msg.orientation_covariance  # Preserve covariance

        self.publisher.publish(transformed_imu)

def main(args=None):
    rclpy.init(args=args)
    imu_transform = ImuTransform()
    rclpy.spin(imu_transform)
    imu_transform.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# import tf_transformations

# class ImuTransform(Node):
#     def __init__(self):
#         super().__init__('imu_transform')
#         self.subscription = self.create_subscription(
#             Imu,
#             '/camera/imu',
#             self.imu_callback,
#             10)
#         self.publisher = self.create_publisher(Imu, '/imu_enu', 10)

#     def imu_callback(self, msg):
#         transformed_imu = Imu()
#         transformed_imu.header = msg.header

#         # Transform linear acceleration
#         transformed_imu.linear_acceleration.x = msg.linear_acceleration.x
#         transformed_imu.linear_acceleration.y = msg.linear_acceleration.z
#         transformed_imu.linear_acceleration.z = -msg.linear_acceleration.y

#         # Transform angular velocity
#         transformed_imu.angular_velocity.x = msg.angular_velocity.x
#         transformed_imu.angular_velocity.y = msg.angular_velocity.z
#         transformed_imu.angular_velocity.z = -msg.angular_velocity.y

#         # Transform orientation (assuming the orientation is quaternion)
#         q = (
#             msg.orientation.x,
#             msg.orientation.y,
#             msg.orientation.z,
#             msg.orientation.w
#         )
#         # Rotate 
#         rotation1 = tf_transformations.quaternion_from_euler(3.14159, 0, 0)
#         rotation2 = tf_transformations.quaternion_from_euler(0, 0, 1.5708)

#         q_tmp = tf_transformations.quaternion_multiply(rotation1, q)
#         q_transformed = tf_transformations.quaternion_multiply(rotation2, q_tmp)
#         transformed_imu.orientation.x = q_transformed[0]
#         transformed_imu.orientation.y = q_transformed[1]
#         transformed_imu.orientation.z = q_transformed[2]
#         transformed_imu.orientation.w = q_transformed[3]

#         self.publisher.publish(transformed_imu)

# def main(args=None):
#     rclpy.init(args=args)
#     imu_transform = ImuTransform()
#     rclpy.spin(imu_transform)
#     imu_transform.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
