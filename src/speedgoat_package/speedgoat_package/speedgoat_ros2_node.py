import math
import socket
from typing import List, Sequence, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scipy.optimize import minimize
from std_msgs.msg import Float64, Float64MultiArray
from tf_transformations import euler_from_quaternion

from speedgoat_package.control_utils import (
    DelayedPathBuffer,
    SteeringProtectionConfig,
    SteeringProtector,
    mean_abs_path_delta,
)


Point = Tuple[float, float]


def udp_send(sock: socket.socket, server_address, data: bytes) -> None:
    try:
        sock.sendto(data, server_address)
    except Exception as exc:
        print(f"UDP Send Error: {exc}")


class BaseController:
    def compute_steering_angle(self, current_pose, reference_path: Sequence[Point], current_speed: float) -> float:
        raise NotImplementedError

    def compute_acceleration(self, current_speed: float, target_speed: float) -> float:
        raise NotImplementedError


class PurePursuitController(BaseController):
    def __init__(self, min_lookahead_distance, lookahead_gain, wheelbase, max_steering_angle):
        self.min_lookahead_distance = min_lookahead_distance
        self.lookahead_gain = lookahead_gain
        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_angle)
        self.goal_reached = False

    def find_lookahead_point(self, car_x, car_y, yaw, reference_path, lookahead_distance):
        best_index = -1
        best_distance = float('inf')
        heading_vec = np.array([math.cos(yaw), math.sin(yaw)])

        for i, (x, y) in enumerate(reference_path):
            dx = x - car_x
            dy = y - car_y
            point_vec = np.array([dx, dy])
            distance = np.hypot(dx, dy)
            forward = np.dot(heading_vec, point_vec) > 0

            if forward and distance >= lookahead_distance and distance < best_distance:
                best_distance = distance
                best_index = i

        return best_index

    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        if not reference_path:
            print("[PurePursuit] Reference path vuoto")
            return 0.0

        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        lookahead_distance = max(
            self.min_lookahead_distance,
            self.lookahead_gain * current_speed ** (1 / 2.5),
        )
        idx = self.find_lookahead_point(car_x, car_y, yaw, reference_path, lookahead_distance)

        if idx == -1:
            distances = [np.hypot(x - car_x, y - car_y) for x, y in reference_path]
            idx = int(np.argmin(distances))
            if np.hypot(reference_path[-1][0] - car_x, reference_path[-1][1] - car_y) < 1.0:
                self.goal_reached = True
                return 0.0

        target_x, target_y = reference_path[idx]
        dx = target_x - car_x
        dy = target_y - car_y
        alpha = math.atan2(dy, dx) - yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        if lookahead_distance < 1e-3:
            return 0.0

        steer_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), lookahead_distance)
        return float(np.clip(steer_angle, -self.max_steering_angle, self.max_steering_angle))

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            return -4.0
        return float(np.clip(0.5 * (target_speed - current_speed), -4.0, 2.0))


class StanleyController(BaseController):
    def __init__(self, k, wheelbase, max_steering_angle):
        self.k = k
        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_angle)
        self.goal_reached = False

    def find_nearest_point(self, front_car_x, front_car_y, reference_path):
        nearest_index = 0
        nearest_distance = float('inf')
        for i, (path_x, path_y) in enumerate(reference_path):
            distance = math.hypot(path_x - front_car_x, path_y - front_car_y)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i
        return nearest_index

    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        if not reference_path or len(reference_path) < 2:
            return 0.0

        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        front_car_x = car_x + (self.wheelbase / 1.7) * math.cos(car_yaw)
        front_car_y = car_y + (self.wheelbase / 1.7) * math.sin(car_yaw)
        nearest_index = self.find_nearest_point(front_car_x, front_car_y, reference_path)

        if nearest_index >= len(reference_path) - 1:
            self.goal_reached = True
            return 0.0

        nearest_point = reference_path[nearest_index]
        next_point = reference_path[min(nearest_index + 1, len(reference_path) - 1)]
        path_dx = next_point[0] - nearest_point[0]
        path_dy = next_point[1] - nearest_point[1]
        path_yaw = math.atan2(path_dy, path_dx)

        heading_error = path_yaw - car_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        dx = nearest_point[0] - front_car_x
        dy = nearest_point[1] - front_car_y
        error_vector = np.array([dx, dy])
        normal_vector = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        cross_track_error = float(np.dot(error_vector, normal_vector))

        steering_angle = heading_error + math.atan2(self.k * cross_track_error, max(1e-3, current_speed))
        return float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            return -4.0
        return float(np.clip(0.5 * (target_speed - current_speed), -4.0, 2.0))


class PIDLateralController(BaseController):
    def __init__(self, kp, ki, kd, heading_gain, max_integral, wheelbase, max_steering_angle):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.heading_gain = heading_gain
        self.max_integral = max_integral
        self.dt = 0.1
        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_angle)
        self.derivative_alpha = 0.1
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.goal_reached = False

    def find_nearest_point(self, front_car_x, front_car_y, reference_path):
        nearest_index = 0
        nearest_distance = float('inf')
        for i, (path_x, path_y) in enumerate(reference_path):
            distance = math.hypot(path_x - front_car_x, path_y - front_car_y)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i
        return nearest_index

    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        if not reference_path or len(reference_path) < 2:
            return 0.0

        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        front_car_x = car_x + (self.wheelbase / 2.0) * math.cos(car_yaw)
        front_car_y = car_y + (self.wheelbase / 2.0) * math.sin(car_yaw)
        nearest_index = self.find_nearest_point(front_car_x, front_car_y, reference_path)

        if nearest_index >= len(reference_path) - 1:
            self.goal_reached = True
            return 0.0

        nearest_point = reference_path[nearest_index]
        next_point = reference_path[min(nearest_index + 1, len(reference_path) - 1)]
        path_dx = next_point[0] - nearest_point[0]
        path_dy = next_point[1] - nearest_point[1]
        path_yaw = math.atan2(path_dy, path_dx)

        heading_error = path_yaw - car_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        dx = nearest_point[0] - front_car_x
        dy = nearest_point[1] - front_car_y
        error_vector = np.array([dx, dy])
        normal_vector = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        cross_track_error = float(np.dot(error_vector, normal_vector))
        error = cross_track_error + self.heading_gain * heading_error

        self.integral_error += error * self.dt
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)

        derivative_raw = (error - self.prev_error) / self.dt
        derivative = (1 - self.derivative_alpha) * derivative_raw + self.derivative_alpha * self.prev_derivative
        self.prev_error = error
        self.prev_derivative = derivative

        steering_angle = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        return float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            return -4.0
        return float(np.clip(0.5 * (target_speed - current_speed), -4.0, 2.0))


class MPCController(BaseController):
    def __init__(self, N, dt, wheelbase, max_steer, max_accel):
        self.N = N
        self.dt = dt
        self.wheelbase = wheelbase
        self.max_steer = math.radians(max_steer)
        self.max_accel = max_accel
        self.weight_cte = 10.0
        self.weight_heading = 10.0
        self.weight_steer = 5.0
        self.weight_steer_rate = 50.0
        self.weight_accel = 1.0
        self.weight_accel_rate = 10.0
        self.weight_speed = 3.0
        self.prev_control_seq = np.zeros(2 * self.N)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def vehicle_model(self, state, steer, accel):
        x, y, yaw, v = state
        beta = math.atan(0.5 * math.tan(steer))
        x += v * math.cos(yaw + beta) * self.dt
        y += v * math.sin(yaw + beta) * self.dt
        yaw += (v / self.wheelbase) * math.tan(steer) * self.dt
        yaw = self.normalize_angle(yaw)
        v = np.clip(v + accel * self.dt, 0.0, 30.0)
        return np.array([x, y, yaw, v])

    def find_nearest_point(self, x, y, ref_path):
        distance = [math.hypot(px - x, py - y) for px, py in ref_path]
        return int(np.argmin(distance))

    def preprocess_reference_path(self, ref_path):
        if len(ref_path) < 2:
            raise ValueError("Reference path too short")
        ref_path_yaw = []
        for i in range(len(ref_path) - 1):
            x0, y0 = ref_path[i][:2]
            x1, y1 = ref_path[i + 1][:2]
            ref_path_yaw.append((x0, y0, math.atan2(y1 - y0, x1 - x0)))
        return ref_path_yaw

    def cost_function(self, vars, initial_state, ref_path, target_speed):
        steer_seq = vars[: self.N]
        accel_seq = vars[self.N :]
        state = np.copy(initial_state)
        cost = 0.0
        for i in range(self.N):
            steer = steer_seq[i]
            accel = accel_seq[i]
            state = self.vehicle_model(state, steer, accel)
            x, y, yaw, v = state
            ref_x, ref_y, ref_yaw = ref_path[i]
            dx = x - ref_x
            dy = y - ref_y
            cte = -math.sin(yaw) * dx + math.cos(yaw) * dy
            heading_error = self.normalize_angle(ref_yaw - yaw)
            speed_error = v - target_speed
            cost += self.weight_cte * cte ** 2
            cost += self.weight_heading * heading_error ** 2
            cost += self.weight_steer * steer ** 2
            cost += self.weight_accel * accel ** 2
            cost += self.weight_speed * speed_error ** 2
            if i > 0:
                cost += self.weight_steer_rate * (steer_seq[i] - steer_seq[i - 1]) ** 2
                cost += self.weight_accel_rate * (accel_seq[i] - accel_seq[i - 1]) ** 2
        return cost

    def compute_control(self, current_state, reference_path, current_speed, target_speed):
        if not reference_path or len(reference_path) < 2:
            return 0.0, 0.0
        x = current_state.position.x
        y = current_state.position.y
        orientation_q = current_state.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        initial_state = np.array([x, y, yaw, current_speed])
        ref_path_yaw = self.preprocess_reference_path(reference_path)
        nearest_idx = self.find_nearest_point(x, y, reference_path)
        horizon_path = ref_path_yaw[nearest_idx: nearest_idx + self.N]
        if len(horizon_path) < self.N:
            horizon_path += [horizon_path[-1]] * (self.N - len(horizon_path))
        steer_bounds = [(-self.max_steer, self.max_steer)] * self.N
        accel_bounds = [(-self.max_accel, self.max_accel)] * self.N
        bounds = steer_bounds + accel_bounds
        result = minimize(
            self.cost_function,
            self.prev_control_seq,
            args=(initial_state, horizon_path, target_speed),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 50, 'disp': False},
        )
        if not result.success:
            return 0.0, 0.0
        u_seq = np.clip(result.x, [b[0] for b in bounds], [b[1] for b in bounds])
        self.prev_control_seq = u_seq
        steer_seq = u_seq[: self.N]
        accel_seq = u_seq[self.N :]
        return float(steer_seq[0]), float(accel_seq[0])

    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        raise NotImplementedError("Use compute_control for MPC")

    def compute_acceleration(self, current_speed, target_speed):
        return 0.0


class Speedgoat(Node):
    def __init__(self):
        super().__init__('speedgoat_controller')
        self.declare_parameter('control_method', 'pid')
        self.declare_parameter('wheelbase', 2.9)
        self.declare_parameter('target_speed', 5.0)
        self.declare_parameter('max_steering_angle', 30.0)
        self.declare_parameter('max_acceleration', 5.0)
        self.declare_parameter('steering_smoothing_alpha', 0.25)
        self.declare_parameter('steering_rate_limit_deg_per_sec', 90.0)
        self.declare_parameter('steering_deadband_deg', 0.5)
        self.declare_parameter('path_delay_frames', 0)
        self.declare_parameter('debug_log_every_n_frames', 20)
        self.declare_parameter('enable_debug_topics', True)

        method = self.get_parameter('control_method').value
        wheelbase = self.get_parameter('wheelbase').value
        self.target_speed = self.get_parameter('target_speed').value
        max_steer = self.get_parameter('max_steering_angle').value
        max_accel = self.get_parameter('max_acceleration').value
        smoothing_alpha = self.get_parameter('steering_smoothing_alpha').value
        steering_rate_limit_deg = self.get_parameter('steering_rate_limit_deg_per_sec').value
        steering_deadband_deg = self.get_parameter('steering_deadband_deg').value
        path_delay_frames = self.get_parameter('path_delay_frames').value
        self.debug_log_every_n_frames = self.get_parameter('debug_log_every_n_frames').value
        self.enable_debug_topics = self.get_parameter('enable_debug_topics').value

        if method == 'pure_pursuit':
            self.controller = PurePursuitController(1.0, 1.0, wheelbase, max_steer)
        elif method == 'stanley':
            self.controller = StanleyController(5.0, wheelbase, max_steer)
        elif method == 'pid':
            self.controller = PIDLateralController(1.7, 0.01, 0.1, 1.0, 1.0, wheelbase, max_steer)
        elif method == 'mpc':
            self.controller = MPCController(15, 0.1, wheelbase, max_steer, max_accel)
        else:
            raise ValueError(f'Unknown Method: {method}')

        self.path_buffer = DelayedPathBuffer(delay_frames=path_delay_frames)
        self.steering_protector = SteeringProtector(
            SteeringProtectionConfig(
                max_angle_rad=math.radians(max_steer),
                smoothing_alpha=smoothing_alpha,
                rate_limit_rad_per_sec=math.radians(steering_rate_limit_deg),
                deadband_rad=math.radians(steering_deadband_deg),
                nominal_dt=0.1,
            )
        )

        self.create_subscription(Path, '/reference_path', self.path_callback, 1)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 20)

        self.reference_path: List[Point] = []
        self.latest_path_input: List[Point] = []
        self.last_path_stamp = None
        self.last_odom_time_sec = None
        self.frame_counter = 0

        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 20)
        self.steer_raw_pub = self.create_publisher(Float64, '/steering_angle/raw', 20)
        self.accel_pub = self.create_publisher(Float64, '/acceleration', 20)
        self.control_metrics_pub = self.create_publisher(Float64MultiArray, '/control/debug/metrics', 20)

        self.host = '10.42.0.10'
        self.port = 5500
        self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def path_callback(self, path_msg: Path):
        incoming_path = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        if not incoming_path:
            self.get_logger().warn('Path vuoto ricevuto.')
            return

        previous_input = self.latest_path_input
        self.latest_path_input = incoming_path
        self.reference_path = self.path_buffer.push(incoming_path)
        self.last_path_stamp = path_msg.header.stamp

        path_delta = mean_abs_path_delta(previous_input, incoming_path)
        if path_delta > 0.0 and self.enable_debug_topics:
            self.get_logger().debug(
                f'Path update: points={len(incoming_path)} mean_abs_delta={path_delta:.3f} delayed_points={len(self.reference_path)}'
            )

    def odom_callback(self, odom_msg: Odometry):
        if not self.reference_path:
            self.get_logger().warn('Reference path non ancora ricevuto.')
            return

        current_pose = odom_msg.pose.pose
        current_speed = odom_msg.twist.twist.linear.x
        dt = self._compute_dt(odom_msg)

        try:
            if isinstance(self.controller, MPCController):
                raw_steer, accel = self.controller.compute_control(
                    current_pose,
                    self.reference_path,
                    current_speed,
                    self.target_speed,
                )
            else:
                raw_steer = self.controller.compute_steering_angle(current_pose, self.reference_path, current_speed)
                accel = self.controller.compute_acceleration(current_speed, self.target_speed)
        except Exception as exc:
            self.get_logger().error(f'Controller Error: {exc}')
            return

        steering_metrics = self.steering_protector.apply(raw_steer, dt)
        self.publish_control(steering_metrics.final_angle_rad, accel)
        self.publish_debug(raw_steer, steering_metrics, current_speed)

        udp_data = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, steering_metrics.final_angle_rad, accel],
            dtype=np.float64,
        )
        udp_send(self.socket_udp, (self.host, self.port), udp_data.tobytes())

    def publish_control(self, steering_angle: float, acceleration: float):
        steer_msg = Float64()
        steer_msg.data = math.degrees(steering_angle)
        self.steer_pub.publish(steer_msg)

        accel_msg = Float64()
        accel_msg.data = float(acceleration)
        self.accel_pub.publish(accel_msg)

    def publish_debug(self, raw_steer: float, steering_metrics, current_speed: float):
        raw_msg = Float64()
        raw_msg.data = math.degrees(raw_steer)
        self.steer_raw_pub.publish(raw_msg)

        path_age_sec = self._path_age_sec()
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            math.degrees(steering_metrics.raw_angle_rad),
            math.degrees(steering_metrics.smoothed_angle_rad),
            math.degrees(steering_metrics.final_angle_rad),
            1.0 if steering_metrics.rate_limited else 0.0,
            1.0 if steering_metrics.deadband_applied else 0.0,
            float(len(self.reference_path)),
            path_age_sec,
            current_speed,
        ]
        self.control_metrics_pub.publish(metrics_msg)

        self.frame_counter += 1
        if self.debug_log_every_n_frames > 0 and self.frame_counter % self.debug_log_every_n_frames == 0:
            self.get_logger().info(
                'control_debug speed={:.2f} m/s path_points={} path_age={:.3f} s raw={:.2f} deg final={:.2f} deg '
                'rate_limited={} deadband={}'.format(
                    current_speed,
                    len(self.reference_path),
                    path_age_sec,
                    math.degrees(steering_metrics.raw_angle_rad),
                    math.degrees(steering_metrics.final_angle_rad),
                    1 if steering_metrics.rate_limited else 0,
                    1 if steering_metrics.deadband_applied else 0,
                )
            )

    def _compute_dt(self, odom_msg: Odometry) -> float:
        timestamp = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        if self.last_odom_time_sec is None:
            self.last_odom_time_sec = timestamp
            return 0.1
        dt = max(1e-3, timestamp - self.last_odom_time_sec)
        self.last_odom_time_sec = timestamp
        return dt

    def _path_age_sec(self) -> float:
        if self.last_path_stamp is None:
            return 0.0
        now = self.get_clock().now().nanoseconds * 1e-9
        path_time = self.last_path_stamp.sec + self.last_path_stamp.nanosec * 1e-9
        return max(0.0, now - path_time)


def main(args=None):
    rclpy.init(args=args)
    node = Speedgoat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
