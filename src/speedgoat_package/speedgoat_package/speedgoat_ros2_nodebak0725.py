# ROS2 imports
import rclpy
from rclpy.node import Node

# Python imports
import numpy as np
import math
import socket
from scipy.optimize import minimize
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf_transformations import euler_from_quaternion

# ROS2 messages
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

# ========================== UDP Send ==========================
def udp_send(socket, server_address, data):
    try:
        socket.sendto(data, server_address)
    except Exception as e:
        print(f"UDP Send Error: {e}")

# ========================== BaseController ==========================
class BaseController:
    def compute_steering_angle(self, current_pose, reference_path):
        raise NotImplementedError

    def compute_acceleration(self, current_speed, target_speed):
        raise NotImplementedError

# ========================== Pure Pursuit ==========================
class PurePursuitController(BaseController):
    def __init__(self, lookahead_distance=5.0, max_steering_angle=30.0):
        self.lookahead_distance = lookahead_distance
        self.max_steering_angle = math.radians(max_steering_angle)

    def find_lookahead_point(self, car_x, car_y, reference_path):
        min_distance = float('inf')
        target_index = -1
        for i, (x, y) in enumerate(reference_path):
            distance = math.hypot(x - car_x, y - car_y)
            if distance >= self.lookahead_distance and distance < min_distance:
                min_distance = distance
                target_index = i
        return target_index

    def compute_steering_angle(self, current_pose, reference_path):
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        target_index = self.find_lookahead_point(car_x, car_y, reference_path)
        if target_index == -1:
            return 0.0
        
        target_x, target_y = reference_path[target_index]
        dx = target_x - car_x
        dy = target_y - car_y
        target_angle = math.atan2(dy, dx)
        
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        steering_angle = target_angle - yaw
        
        # [-pi, pi]
        steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi
        return np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

    def compute_acceleration(self, current_speed, target_speed):
        return 0.5 * (target_speed - current_speed) 

# ========================== Stanley  ==========================
class StanleyController(BaseController):
    def __init__(self, k=2.0, soft_gain=0.1, max_steering_angle=30.0):
        self.k = k
        self.soft_gain = soft_gain
        self.max_steering_angle = math.radians(max_steering_angle)

    def compute_steering_angle(self, current_pose, reference_path):
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        
       
        nearest_point = min(reference_path, key=lambda p: (p[0]-car_x)**2 + (p[1]-car_y)**2)
        dx = nearest_point[0] - car_x
        dy = nearest_point[1] - car_y
        path_angle = math.atan2(dy, dx)
        
        
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        heading_error = path_angle - yaw
        
        
        cross_track_error = math.atan2(self.k * math.hypot(dx, dy), self.soft_gain)
        
        steering_angle = heading_error + cross_track_error
        return np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

    def compute_acceleration(self, current_speed, target_speed):
        return 0.5 * (target_speed - current_speed)

# ========================== PID  ==========================
class PIDLateralController(BaseController):
    def __init__(self, kp=0.2, ki=0.01, kd=0.05, max_steering_angle=30.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_steering = math.radians(max_steering_angle)
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.1  

    def compute_steering_angle(self, current_pose, reference_path):
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        
       
        nearest_point = min(reference_path, key=lambda p: (p[0]-car_x)**2 + (p[1]-car_y)**2)
        error = nearest_point[0] - car_x 
        
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        steering_angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        return np.clip(steering_angle, -self.max_steering, self.max_steering)

    def compute_acceleration(self, current_speed, target_speed):
        return 0.5 * (target_speed - current_speed)

# ========================== MPC  ==========================
class MPCController(BaseController):
    def __init__(self, N=5, dt=0.2, wheelbase=2.5, max_steer=30.0, max_accel=1.0):
        self.N = N          
        self.dt = dt        
        self.L = wheelbase  
        self.max_steer = math.radians(max_steer)
        self.max_accel = max_accel
        
    def vehicle_model(self, state, u):
        """Bicycle model"""
        x, y, yaw, v = state
        steer, accel = u
        
        new_x = x + v * math.cos(yaw) * self.dt
        new_y = y + v * math.sin(yaw) * self.dt
        new_yaw = yaw + (v / self.L) * math.tan(steer) * self.dt
        new_v = v + accel * self.dt
        return np.array([new_x, new_y, new_yaw, new_v])

    def cost_function(self, u, *args):
        
        state, ref_path = args
        u = u.reshape((self.N, 2))  
        total_cost = 0.0
        
        current_state = np.copy(state)
        for i in range(self.N):
            
            current_state = self.vehicle_model(current_state, u[i])
            x, y, _, _ = current_state
            
            nearest_dist = min((x - rx)**2 + (y - ry)**2 for (rx, ry) in ref_path)
            total_cost += 100 * nearest_dist  
            
            if i > 0:
                total_cost += 10 * (u[i][0] - u[i-1][0])**2  
                total_cost += 10 * (u[i][1] - u[i-1][1])**2  
        
        return total_cost

    def compute_steering_angle(self, current_pose, reference_path, current_speed):

        x = current_pose.position.x
        y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, 
                                         orientation_q.z, orientation_q.w])
        v = current_speed 
        
        initial_state = np.array([x, y, yaw, v])
        
        u0 = np.zeros((self.N, 2))
        
        bounds = [
            (-self.max_steer, self.max_steer),  
            (-self.max_accel, self.max_accel)   
        ] * self.N
        
        result = minimize(self.cost_function, u0.flatten(), args=(initial_state, reference_path),
                          method='SLSQP', bounds=bounds, 
                          options={'maxiter': 20, 'disp': False})
        
        if not result.success:
            print("MPC Error!")
            return 0.0
        
        optimal_u = result.x.reshape((self.N, 2))
        return optimal_u[0][0]  

    def compute_acceleration(self, current_speed, target_speed):
        return np.clip(0.5 * (target_speed - current_speed), -self.max_accel, self.max_accel)

# ========================== Main ==========================
class Speedgoat(Node):
    def __init__(self):
        super().__init__('speedgoat_controller')
        
        self.declare_parameter('control_method', 'pure_pursuit')
        self.declare_parameter('target_speed', 5.0)
        self.declare_parameter('max_steering_angle', 30.0)
        self.declare_parameter('max_acceleration', 1.0)

        method = self.get_parameter('control_method').value
        target_speed = self.get_parameter('target_speed').value
        max_steer = self.get_parameter('max_steering_angle').value
        max_accel = self.get_parameter('max_acceleration').value
        
        if method == 'pure_pursuit':
            self.controller = PurePursuitController(
                lookahead_distance=10.0, 
                max_steering_angle=max_steer
            )
        elif method == 'stanley':
            self.controller = StanleyController(
                k=2.0, 
                soft_gain=0.1, 
                max_steering_angle=max_steer
            )
        elif method == 'pid':
            self.controller = PIDLateralController(
                kp=0.2, 
                ki=0.01, 
                kd=0.05, 
                max_steering_angle=max_steer
            )
        elif method == 'mpc':
            self.controller = MPCController(
                N=5, 
                dt=0.2, 
                max_steer=max_steer,
                max_accel=max_accel
            )
        else:
            self.get_logger().error(f"Unknow Method: {method}")
            raise ValueError

        self.odom_sub = Subscriber(self, Odometry, '/odometry/filtered')
        self.path_sub = Subscriber(self, Path, '/reference_path')
        
        self.sync = ApproximateTimeSynchronizer(
            [self.odom_sub, self.path_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.control_callback)
        
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.accel_pub = self.create_publisher(Float64, '/acceleration', 10)

        self.host = "10.42.0.10"
        self.port = 5500             
        self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.target_speed = target_speed
        self.last_reference_path = []

    def control_callback(self, odom_msg, path_msg):
        if len(path_msg.poses) == 0:
            if len(self.last_reference_path) == 0:
                self.get_logger().warn("No Reference Path!")
                return
            reference_path = self.last_reference_path
        else:
            reference_path = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
            self.last_reference_path = reference_path
        
        current_pose = odom_msg.pose.pose
        current_speed = odom_msg.twist.twist.linear.x  
        
        try:
            if isinstance(self.controller, MPCController):
                # For MPC need current_speed
                steer = self.controller.compute_steering_angle(current_pose, reference_path, current_speed)
            else:
                steer = self.controller.compute_steering_angle(current_pose, reference_path)
                
            accel = self.controller.compute_acceleration(current_speed, self.target_speed)
        except Exception as e:
            self.get_logger().error(f"Controller Error: {str(e)}")
            return
        
        self.publish_control(steer, accel)
        
        udp_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, steer, accel]).astype(np.float64)
        # udp_data = np.array([steer, accel, current_speed], dtype=np.float64)
        udp_send(self.socket_udp, (self.host, self.port), udp_data.tobytes())

    def publish_control(self, steering_angle, acceleration):

        steer_msg = Float64()
        steer_msg.data = float(steering_angle)
        self.steer_pub.publish(steer_msg)
        
        accel_msg = Float64()
        accel_msg.data = float(acceleration)
        self.accel_pub.publish(accel_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = Speedgoat()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown(Speedgoat)

# if __name__ == '__main__':
#     main()


# # ROS2 imports 
# import rclpy
# from rclpy.node import Node

# # python imports
# import numpy as np
# import socket
# import cv2
# import os, sys, time

# # message imports
# from sensor_msgs.msg import Image as SensorImage
# from std_msgs.msg import String
# from lane_parameter_msg.msg import LaneParams
# from sensor_msgs.msg import CompressedImage
# from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer

# from nav_msgs.msg import Odometry, Path
# from std_msgs.msg import Float64
# from tf_transformations import euler_from_quaternion
# import math

# class PurePursuit:
#     def __init__(self, lookahead_distance=5.0, max_steering_angle=30.0):

#         self.lookahead_distance = lookahead_distance
#         self.max_steering_angle = math.radians(max_steering_angle)

#     def find_lookahead_point(self, car_x, car_y, reference_path):

#         min_distance = float('inf')
#         target_index = -1

#         for i, (path_x, path_y) in enumerate(reference_path):
#             dx = path_x - car_x
#             dy = path_y - car_y
#             distance = math.sqrt(dx ** 2 + dy ** 2)

#             if distance >= self.lookahead_distance and distance < min_distance:
#                 min_distance = distance
#                 target_index = i

#         return target_index

#     def compute_steering_angle(self, current_pose, target_point):

#         car_x = current_pose.position.x
#         car_y = current_pose.position.y

#         dx = target_point[0] - car_x
#         dy = target_point[1] - car_y

#         target_angle = math.atan2(dy, dx)

#         orientation_q = current_pose.orientation
#         _, _, car_yaw = euler_from_quaternion([
#             orientation_q.x,
#             orientation_q.y,
#             orientation_q.z,
#             orientation_q.w
#         ])

#         steering_angle = target_angle - car_yaw

#         if steering_angle > math.pi:
#             steering_angle -= 2 * math.pi
#         elif steering_angle < -math.pi:
#             steering_angle += 2 * math.pi

#         return steering_angle

# def UDP_send(socket_UDP, server_address, msg):
#     socket_UDP.sendto(msg,server_address)
#     return None
    
# class PurePursuitController:
#     def __init__(self, lookahead_distance, max_steering_angle):
#         self.lookahead_distance = lookahead_distance
#         self.max_steering_angle = max_steering_angle

#     def calculate_control(self, mid_lane):
#         current_pos = (0,0)
#         lookahead_point = None

#         for x, y in mid_lane:
#             distance = np.sqrt(x**2 + y**2)
#             if distance > self.lookahead_distance:
#                 lookahead_point = (x,y)
#                 break

#         if lookahead_point is None:
#             lookahead_point = mid_lane[-1]
        
#         dx = lookahead_point[0] - current_pos[0]
#         dy = lookahead_point[1] - current_pos[1]

#         angle_to_lookahead = np.arctan2(dy, dx)

#         angle_to_lookahead = max(min(angle_to_lookahead, self.max_steering_angle), -self.max_steering_angle)    
#         return angle_to_lookahead

# from scipy.optimize import minimize

# class MPCController:
#     def __init__(self, N, dt, L, max_steering_angle, max_acceleration, ref_v):
#         self.N = N  # Prediction horizon
#         self.dt = dt  # Time step
#         self.L = L  # Wheelbase
#         self.max_steering_angle = max_steering_angle
#         self.max_acceleration = max_acceleration
#         self.ref_v = ref_v  # Reference velocity

#     def vehicle_model(self, state, control, dt):
#         x, y, psi, v = state
#         delta, a = control
#         x_next = x + v * np.cos(psi) * dt
#         y_next = y + v * np.sin(psi) * dt
#         psi_next = psi + (v / self.L) * np.tan(delta) * dt
#         v_next = v + a * dt
#         return np.array([x_next, y_next, psi_next, v_next])

#     def optimize(self, waypoints, current_state):
#         # Objective function to minimize
#         def objective(U):
#             U = U.reshape((self.N, 2))
#             state = np.array([0, 0, current_state[2], current_state[3]])  # x, y are always 0 in vehicle coordinates
#             cost = 0
#             prev_u = np.zeros(2)
#             for t in range(self.N):
#                 state = self.vehicle_model(state, U[t], self.dt)
#                 x, y, psi, v = state
#                 cost += 10*((x - waypoints[t, 0])**2 + (y - waypoints[t, 1])**2)  # Minimize path error
#                 cost += (v - self.ref_v)**2  # Minimize velocity error
#                 cost += (U[t, 0]**2 + U[t, 1]**2)  # Minimize control effort
#                 cost += 10*np.sum((U[t]-prev_u)**2)
#                 prev_u = U[t]
#             return cost

#         # Initial guess for control inputs
#         U0 = np.zeros((self.N, 2)).flatten()

#         # Bounds for control inputs
#         bounds = []
#         for t in range(self.N):
#             bounds.append((-self.max_steering_angle, self.max_steering_angle))  # Bounds for delta
#             bounds.append((-self.max_acceleration, self.max_acceleration))      # Bounds for a

#         # Solve the optimization problem
#         result = minimize(objective, U0, bounds=bounds, method='SLSQP', options={'ftol': 1e-6})

#         if result.success:
#             optimal_U = result.x.reshape((self.N, 2))
#             steering_angle, acceleration = optimal_U[0]
#             return steering_angle, acceleration
#         else:
#             raise ValueError(f"Optimization failed: {result.message}")


# class Speedgoat(Node):
#     def __init__(self):
#         super().__init__('speedgoat_node')
#         self.declare_parameter('method', 'pure_pursuit')
#         self.declare_parameter('target_speed', 5.0)

#         self.method = self.get_parameter('method').get_parameter_value().string_value
#         self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value

#         self.pure_pursuit = PurePursuit(lookahead_distance=5.0, max_steering_angle=30.0)

#         self.kp = 1.0 
#         self.ki = 0.1  
#         self.kd = 0.01  
#         self.integral_error = 0.0
#         self.previous_error = 0.0

#         self.odom_sub = Subscriber(self, Odometry, '/odometry/filtered')
#         self.path_sub = Subscriber(self, Path, '/reference_path')
#         self.sync = ApproximateTimeSynchronizer([self.odom_sub, self.path_sub], queue_size=10, slop=0.1)
#         self.sync.registerCallback(self.synchronized_callback)
        
#         self.host = "10.42.0.10"
#         #self.host = "169.254.18.189"
#         self.port = 5500
#         self.send_port = 8080
#         self.buffersize = 1024
#         self.server_address = (self.host, self.port) 
       
#         # create Socket UDP
#         self.socket_UDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.socket_UDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
#         # self.socket_UDP.bind((self.host, 0))
#         self.broadcast_address = ('10.42.0.255', self.port)

#         self.socket_UDP_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.receive_addr = ("", self.send_port)
#         self.socket_UDP_receive.bind(self.receive_addr)
#         self.socket_UDP_receive.settimeout(1)
        
#         max_steering_angle = 0.26
#         lookahead_distance = 5
#         self.x_vehicle = np.linspace(0,20,50)

#         N = 10  # Prediction horizon
#         dt = 0.1  # Time step
#         L = 2.5  # Wheelbase of the vehicle in meters
#         max_acceleration = 1.0  # Maximum acceleration in m/s^2
#         #ref_v = 7.0  # Reference velocity in m/s

#         # if self.method == 'pure_pursuit':
#         #     self.pure_pursuit = PurePursuitController(lookahead_distance, max_steering_angle)
#         # elif self.method == 'mpc':
#         #     self.mpc = MPCController(N, dt, L, max_steering_angle, max_acceleration, self.ref_v)

#         self.max_acceleration = 1.0  # Maximum acceleration in m/s^2
#         #ref_v = 7.0  # Reference velocity in m/s

#         # if self.method == 'pure_pursuit':
#         #     self.pure_pursuit = PurePursuitController(lookahead_distance, max_steering_angle)
#         # elif self.method == 'mpc':
#         #     self.mpc = MPCController(N, dt, L, max_steering_angle, max_acceleration, self.ref_v)

#         self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
#         self.acceleration_pub = self.create_publisher(Float64, '/acceleration', 10)

#         self.reference_path = []
#         self.last_valid_reference_path = [] 

#     def synchronized_callback(self, odom_msg, path_msg):

#         if len(path_msg.poses) == 0:
#             if len(self.last_valid_reference_path) == 0:
#                 self.get_logger().warn("Reference path is empty and no valid previous path available.")
#                 return
#             else:
#                 self.get_logger().warn("Reference path is empty. Using the last valid reference path.")
#                 self.reference_path = self.last_valid_reference_path
#         else:
#             self.reference_path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
#             self.last_valid_reference_path = self.reference_path 

#         if not self.reference_path:
#             self.get_logger().warn("Reference path is empty, cannot perform path tracking.")
#             return

#         car_pose = odom_msg.pose.pose
#         target_index = self.pure_pursuit.find_lookahead_point(
#             car_pose.position.x, car_pose.position.y, self.reference_path
#         )

#         if target_index == -1:
#             self.get_logger().warn("No valid lookahead point found.")
#             return

#         target_point = self.reference_path[target_index]
#         steering_angle = self.pure_pursuit.compute_steering_angle(car_pose, target_point)

#         current_speed = odom_msg.twist.twist.linear.x
#         acceleration = self.compute_pid_acceleration(current_speed)

#         steering_angle = max(min(steering_angle, self.pure_pursuit.max_steering_angle),
#                              -self.pure_pursuit.max_steering_angle)
#         acceleration = max(min(acceleration, self.max_acceleration), -self.max_acceleration)

#         self.publish_control_commands(steering_angle, acceleration)

#         current_time = time.time()
#         # udp_data = np.array([steering_angle, acceleration]).astype(np.float64)
#         udp_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, steering_angle]).astype(np.float64)
        
#         udp_data = np.concatenate((udp_data, [current_time]), axis=0).astype(np.float64)
#         UDP_msg = udp_data.tobytes()
#         # print(UDP_msg)
#         UDP_send(socket_UDP=self.socket_UDP, server_address=self.server_address, msg=UDP_msg)

#         return True 

#     def compute_pid_acceleration(self, current_speed):

#         error = self.target_speed - current_speed
#         self.integral_error += error
#         derivative_error = error - self.previous_error

#         acceleration = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error

#         self.previous_error = error

#         return acceleration

#     def publish_control_commands(self, steering_angle, acceleration):

#         steering_msg = Float64()
#         steering_msg.data = steering_angle
#         self.steering_pub.publish(steering_msg)

#         acceleration_msg = Float64()
#         acceleration_msg.data = acceleration
#         self.acceleration_pub.publish(acceleration_msg)


