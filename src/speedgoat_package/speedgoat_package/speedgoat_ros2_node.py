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
    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        raise NotImplementedError

    def compute_acceleration(self, current_speed, target_speed):
        raise NotImplementedError

# ========================== Pure Pursuit ==========================
class PurePursuitController(BaseController):
    def __init__(self, min_lookahead_distance, lookahead_gain, wheelbase, max_steering_angle):
        self.min_lookahead_distance = min_lookahead_distance
        self.lookahead_gain = lookahead_gain

        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_angle)

        self.goal_reached = False
    
    # Trova il punto del reference path più vicino al veicolo, ad almeno una distanza pari a lookahead distance
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

            if forward and distance >= lookahead_distance:
                if distance < best_distance:
                    best_distance = distance
                    best_index = i

        return best_index

    def compute_steering_angle(self, current_pose, reference_path, current_speed):
        if not reference_path:
            print("[PurePursuit] ⚠️Reference path vuoto")
            return 0.0

        # Estrae posizione e orientamento del veicolo
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
        # Calcola lookahead dinamico
        lookahead_distance = max(self.min_lookahead_distance, self.lookahead_gain * current_speed ** (1/2.5))

        idx = self.find_lookahead_point(car_x, car_y, yaw, reference_path, lookahead_distance)

        if idx == -1:
            print(f"[PurePursuit] Nessun punto valido oltre {lookahead_distance:.2f}m")
            distance = [np.hypot(x - car_x, y - car_y) for x, y in reference_path]
            idx = int(np.argmin(distance))
            if np.hypot(reference_path[-1][0] - car_x, reference_path[-1][1] - car_y) < 1.0:
                self.goal_reached = True
                print(f"[PurePursuit] Goal completato!")
                return 0.0

        target_x, target_y = reference_path[idx]
        dx = target_x - car_x
        dy = target_y - car_y

        alpha = math.atan2(dy, dx) - yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        if lookahead_distance < 1e-3:
            print("[PurePursuit] Lookahead distance troppo piccola, ritorno angolo nullo")
            return 0.0

        steer_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), lookahead_distance)
        steer_angle = np.clip(steer_angle, -self.max_steering_angle, self.max_steering_angle)

        # Riduci piccole oscillazioni
        if abs(steer_angle) < math.radians(0.5):
            steer_angle = 0.0

        # Logging
        print(f"[PurePursuit] 🚗Pos: ({car_x:.2f}, {car_y:.2f}), Yaw: {math.degrees(yaw):.1f}°")
        print(f"[PurePursuit] 🎯Target: ({target_x:.2f}, {target_y:.2f})")
        print(f"[PurePursuit] 💡Lookahead Distance = {lookahead_distance:.2f} m, 🔁Steering Angle: {math.degrees(steer_angle):.2f}°")
    
        return steer_angle

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            print("[PurePursuit] ✅Goal completato -> 🛑Stop veicolo")
            return -4.0

        accel = 0.5 * (target_speed - current_speed)
        accel = np.clip(accel, -4.0, 2.0)
        print(f"[PurePursuit] ⚡Accelerazione: {accel:.2f} m/s² (Target speed: {target_speed:.2f} m/s, Current speed: {current_speed:.2f} m/s)")

        return accel
        
# ========================== Stanley  ==========================
class StanleyController:
    def __init__(self, k, wheelbase, max_steering_angle):
        self.k = k

        self.wheelbase = wheelbase
        self.max_steering_angle = math.radians(max_steering_angle)

        self.goal_reached = False
        self.last_steering_angle = 0.0
    
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
            print("[Stanley] ⚠️ Reference path vuoto o troppo corto.")
            return 0.0
        
        # Posizione e orientamento del veicolo
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Asse anteriore del veicolo
        front_car_x = car_x + (self.wheelbase/(1.7)) * math.cos(car_yaw)
        front_car_y = car_y + (self.wheelbase/(1.7)) * math.sin(car_yaw)

        # Nearest point del reference path rispetto all'asse anteriore del veicolo
        nearest_index = self.find_nearest_point(front_car_x, front_car_y, reference_path)

        if nearest_index >= len(reference_path) - 1:
            print(f"[Stanley] ✅ Goal completato!")
            self.goal_reached = True
            return 0.0
        
        nearest_point = reference_path[nearest_index]
        next_point = reference_path[min(nearest_index + 1, len(reference_path) - 1)]
        
        # Orientamento del path
        path_dx = next_point[0] - nearest_point[0]
        path_dy = next_point[1] - nearest_point[1]
        path_yaw = math.atan2(path_dy, path_dx)

        # Heading error
        heading_error = path_yaw - car_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Cross-track error
        dx = nearest_point[0] - front_car_x
        dy = nearest_point[1] - front_car_y
        error_vector = np.array([dx, dy])
        normal_vector = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        cross_track_error = np.dot(error_vector, normal_vector)

        # Stanley Control Low
        steering_angle = heading_error + math.atan2(self.k * cross_track_error, max(1e-3, current_speed))
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Filtro low-pass per evitare bruschi cambiamenti (Steering Smoother)
        alpha = 0.2
        steering_angle = alpha * self.last_steering_angle + (1 - alpha) * steering_angle
        self.last_steering_angle = steering_angle

        # Riduci piccole oscillazioni (Deadband)
        if abs(steering_angle) < math.radians(0.5):
            steering_angle = 0.0

        print(f"[Stanley] 🚗Position: ({car_x:.2f}, {car_y:.2f}), Yaw: {math.degrees(car_yaw):.1f}°")
        print(f"[Stanley] 🎯Target point: ({nearest_point[0]:.2f}, {nearest_point[1]:.2f})")
        print(f"[Stanley] 🔄Heading error: {math.degrees(heading_error):.2f}°")
        print(f"[Stanley] ↔️Cross-track error: {cross_track_error:.3f} m")
        print(f"[Stanley] 📐Steering angle: {math.degrees(steering_angle):.2f}°")
        
        return steering_angle

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            print("[Stanley] ✅Goal completato -> 🛑Stop veicolo")
            return -4.0

        acceleration = 0.5 * (target_speed - current_speed)
        acceleration = np.clip(acceleration, -4.0, 2.0)

        print(f"[Stanley] ⚡Accelerazione: {acceleration:.2f} m/s² (Target speed: {target_speed:.2f} m/s, Current speed: {current_speed:.2f} m/s)")
        return acceleration

# ========================== PID  ==========================
class PIDLateralController(BaseController):
    def __init__(self, kp, ki, kd, heading_gain, max_integral, wheelbase, max_steering_angle):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.heading_gain = heading_gain
        self.max_integral = max_integral

        self.dt = 0.1
        self.wheelbase = wheelbase 
        self.max_steering_angle= math.radians(max_steering_angle)

        self.derivative_alpha = 0.1
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

        self.goal_reached = False
        self.last_steering_angle = 0.0
  
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
            print("[Stanley] ⚠️ Reference path vuoto o troppo corto.")
            return 0.0
        
        # Posizione e orientamento del veicolo
        car_x = current_pose.position.x
        car_y = current_pose.position.y
        orientation_q = current_pose.orientation
        _, _, car_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Asse anteriore del veicolo
        front_car_x = car_x + (self.wheelbase/(2)) * math.cos(car_yaw)
        front_car_y = car_y + (self.wheelbase/(2)) * math.sin(car_yaw)

        # Nearest point del reference path rispetto all'asse anteriore del veicolo
        nearest_index = self.find_nearest_point(front_car_x, front_car_y, reference_path)

        if nearest_index >= len(reference_path) - 1:
            print(f"[Stanley] ✅ Goal completato!")
            self.goal_reached = True
            return 0.0
        
        nearest_point = reference_path[nearest_index]
        next_point = reference_path[min(nearest_index + 1, len(reference_path) - 1)]
        
        # Yaw della traiettoria
        path_dx = next_point[0] - nearest_point[0]
        path_dy = next_point[1] - nearest_point[1]
        path_yaw = math.atan2(path_dy, path_dx)

        # Heading error
        heading_error = path_yaw - car_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Cross-track error (dot product tra vettori)
        dx = nearest_point[0] - front_car_x
        dy = nearest_point[1] - front_car_y
        error_vector = np.array([dx, dy])
        normal_vector = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
        cross_track_error = np.dot(error_vector, normal_vector)
        
        # Errore sommato
        error = cross_track_error + self.heading_gain * heading_error

        #steering_cte = self.kp * cross_track_error + self.ki * self.integral_error + self.kd * derivative
        #steering_heading = self.heading_gain * heading_error
        #steering_angle = steering_cte + steering_heading

        # Aggiorna integrale con antiwindup
        self.integral_error += error * self.dt
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        
        # Calcola derivata e la filtra per ridurre rumore
        derivative_raw = (error - self.prev_error) / self.dt
        derivative = (1 - self.derivative_alpha) * derivative_raw + self.derivative_alpha * self.prev_derivative
        
        self.prev_error = error
        self.prev_derivative = derivative

        # PID
        steering_angle = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Filtro low-pass sulla sterzata
        alpha = 0.2
        steering_angle = alpha * self.last_steering_angle + (1 - alpha) * steering_angle
        self.last_steering_angle = steering_angle

        # Deadband: evita micro-oscillazioni
        if abs(steering_angle) < math.radians(0.5):
            steering_angle = 0.0

        return steering_angle

    def compute_acceleration(self, current_speed, target_speed):
        if self.goal_reached:
            print("[Stanley] ✅Goal completato -> 🛑Stop veicolo")
            return -4.0

        acceleration = 0.5 * (target_speed - current_speed)
        acceleration = np.clip(acceleration, -4.0, 2.0)

        print(f"[Stanley] ⚡Accelerazione: {acceleration:.2f} m/s² (Target speed: {target_speed:.2f} m/s, Current speed: {current_speed:.2f} m/s)")
        return acceleration

# ========================== MPC  ==========================
class MPCController(BaseController):
    def __init__(self, N, dt, wheelbase, max_steer, max_accel):
        self.N = N     # Orizzonte di predizione
        self.dt = dt   # Passo temporale
        self.wheelbase = wheelbase
        self.max_steer = math.radians(max_steer)
        self.max_accel = max_accel

        # Pesi della funzione di costo
        self.weight_cte = 10.0             # Cross-track error
        self.weight_heading = 10.0         # Heading error
        self.weight_steer = 5.0            # Penalizza sterzata
        self.weight_steer_rate = 50.0      # Variazione variazione sterzata
        self.weight_accel = 1.0            # Penalizza accelerazione
        self.weight_accel_rate = 10.0      # Penalizza variazione accelerazione
        self.weight_speed = 3.0            # Velocità target

        # Stato iniziale
        self.prev_control_seq = np.zeros(2 * self.N)    # [steer_seq, accel_seq]
    
    # Normalizza l’angolo in [-pi, pi]
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    # Bicycle Model
    def vehicle_model(self, state, steer, accel):
        x, y, yaw, v = state
        beta = math.atan(0.5 * math.tan(steer))  # Slip angle semplificata

        # Aggiorna stato
        x += v * math.cos(yaw + beta) * self.dt
        y += v * math.sin(yaw + beta) * self.dt
        
        yaw += (v / self.wheelbase) * math.tan(steer) * self.dt
        yaw = self.normalize_angle(yaw)
    
        v = np.clip(v + accel * self.dt, 0.0, 30.0)  # Clip per realismo

        return np.array([x, y, yaw, v])
    
    # Trova il punto più vicino sul path di riferimento
    def find_nearest_point(self, x, y, ref_path):
        distance = [math.hypot(px - x, py - y) for px, py in ref_path]
        return int(np.argmin(distance))

    # Calcola lo yaw tra ogni coppia di punti consecutivi nel path
    def preprocess_reference_path(self, ref_path):
        if len(ref_path) < 2:
            raise ValueError("Reference path too short")
        
        ref_path_yaw = []
        for i in range(len(ref_path) - 1):
            x0, y0 = ref_path[i][:2]
            x1, y1 = ref_path[i + 1][:2]
            yaw = math.atan2(y1 - y0, x1 - x0)
            ref_path_yaw.append((x0, y0, yaw))

        return ref_path_yaw

    # Funzione obiettivo da minimizzare
    def cost_function(self, vars, initial_state, ref_path, target_speed):
        steer_seq = vars[:self.N]
        accel_seq = vars[self.N:]
        state = np.copy(initial_state)
        cost = 0.0

        for i in range(self.N):
            steer = steer_seq[i]
            accel = accel_seq[i]

            # Predizione stato successivo
            state = self.vehicle_model(state, steer, accel)
            x, y, yaw, v = state
            
            # Punto riferimento path
            ref_x, ref_y, ref_yaw = ref_path[i]

            # Cross-track, Heading and Speed errors
            dx = x - ref_x
            dy = y - ref_y
            cte = -math.sin(yaw) * dx + math.cos(yaw) * dy
            heading_error = self.normalize_angle(ref_yaw - yaw)
            speed_error = v - target_speed

            # Accumula costi
            cost += self.weight_cte * cte ** 2
            cost += self.weight_heading * heading_error ** 2
            cost += self.weight_steer * steer ** 2
            cost += self.weight_accel * accel ** 2
            cost += self.weight_speed * speed_error ** 2
            
            # Penalizza variazioni di controllo
            if i > 0:
                cost += self.weight_steer_rate * (steer_seq[i] - steer_seq[i - 1]) ** 2
                cost += self.weight_accel_rate * (accel_seq[i] - accel_seq[i - 1]) ** 2

        return cost

    def compute_control(self, current_state, reference_path, current_speed, target_speed):
        if not reference_path or len(reference_path) < 2:
            print("[MPC WARNING] Reference path too short")
            return 0.0, 0.0
        
        # Converti stato attuale
        x = current_state.position.x
        y = current_state.position.y
        orientation_q = current_state.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        v = current_speed

        initial_state = np.array([x, y, yaw, v])

        ref_path_yaw = self.preprocess_reference_path(reference_path)
        nearest_idx = self.find_nearest_point(x, y, reference_path)
        horizon_path = ref_path_yaw[nearest_idx : nearest_idx + self.N]

        if len(horizon_path) < self.N:
            horizon_path += [horizon_path[-1]] * (self.N - len(horizon_path))

        steer_bounds = [(-self.max_steer, self.max_steer)] * self.N
        accel_bounds = [(-self.max_accel, self.max_accel)] * self.N
        bounds = steer_bounds + accel_bounds

        # Ottimizzazione con SLSQP
        result = minimize(
            self.cost_function,
            self.prev_control_seq,
            args=(initial_state, horizon_path, target_speed),
            method='SLSQP',
            bounds=bounds, 
            options={'maxiter': 50, 'disp': False}
        )

        if result.success:
            u_seq = np.clip(result.x, [b[0] for b in bounds], [b[1] for b in bounds])
            self.prev_control_seq = u_seq
            steer_seq = u_seq[:self.N]
            accel_seq = u_seq[self.N:]
            steer = steer_seq[0]
            accel = accel_seq[0]

            print(f"[MPC] steer: {math.degrees(steer):.2f}°, accel: {accel:.2f}")
            return steer, accel
        else:
            print(f"[MPC ERROR] Optimization failed: {result.message}")
            return 0.0, 0.0

# ========================== Main ==========================
class Speedgoat(Node):
    def __init__(self):
        super().__init__('speedgoat_controller')
        self.path_end = False
            
        self.declare_parameter('control_method', 'pid')
        self.declare_parameter('wheelbase', 2.9)
        self.declare_parameter('target_speed', 5.0)
        self.declare_parameter('max_steering_angle', 30.0)
        self.declare_parameter('max_acceleration', 5.0)

        method = self.get_parameter('control_method').value
        wheelbase = self.get_parameter('wheelbase').value
        target_speed = self.get_parameter('target_speed').value
        max_steer = self.get_parameter('max_steering_angle').value
        max_accel = self.get_parameter('max_acceleration').value
        
        if method == 'pure_pursuit':
            self.controller = PurePursuitController(
                min_lookahead_distance = 1.0,
                lookahead_gain = 1.0,
                wheelbase = wheelbase,
                max_steering_angle = max_steer
            )
        elif method == 'stanley':
            self.controller = StanleyController(
                k = 5.0,
                wheelbase = wheelbase,
                max_steering_angle = max_steer
            )
        elif method == 'pid':
            self.controller = PIDLateralController(
                kp = 1.7,
                ki = 0.01,
                kd = 0.1,
                heading_gain= 1.0,
                max_integral = 1.0,
                wheelbase = wheelbase,
                max_steering_angle=max_steer
            )
        elif method == 'mpc':
            self.controller = MPCController(
                N = 15,
                dt = 0.1,
                wheelbase = wheelbase,
                max_steer = max_steer,
                max_accel=max_accel
            )
        else:
            self.get_logger().error(f"Unknown Method: {method}")
            raise ValueError

        #self.odom_sub = Subscriber(self, Odometry, '/odom')
        #self.path_sub = Subscriber(self, Path, '/reference_path')
        
        #self.sync = ApproximateTimeSynchronizer(
        #    [self.odom_sub, self.path_sub], 
        #    queue_size=10, 
        #    slop=0.1
        #)
        #self.sync.registerCallback(self.control_callback)

        self.create_subscription(Path, '/reference_path', self.path_callback, 1)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 20)

        self.reference_path = []  # Buffer per il path statico
        
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 20)
        self.accel_pub = self.create_publisher(Float64, '/acceleration', 20)

        self.host = "10.42.0.10"
        self.port = 5500             
        self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.target_speed = target_speed
        self.last_reference_path = []

    def path_callback(self, path_msg):
        if len(path_msg.poses) == 0:
            self.get_logger().warn("Path vuoto ricevuto.")
            return
        self.reference_path = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self.get_logger().info(f"Reference path ricevuto con {len(self.reference_path)} punti.")

    #def control_callback(self, odom_msg, path_msg):
    #    if len(path_msg.poses) == 0:
    #        if len(self.last_reference_path) == 0:
    #            self.get_logger().warn("No Reference Path!")
    #            return
    #        reference_path = self.last_reference_path
    #    else:
    #        reference_path = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
    #        self.last_reference_path = reference_path


    def odom_callback(self, odom_msg):
        if len(self.reference_path) == 0:
            self.get_logger().warn("Reference path non ancora ricevuto.")
            return
        
        current_pose = odom_msg.pose.pose
        current_speed = odom_msg.twist.twist.linear.x  
        
        try:
            steer = self.controller.compute_steering_angle(current_pose, self.reference_path, current_speed)  
            accel = self.controller.compute_acceleration(current_speed, self.target_speed)

        except Exception as e:
            self.get_logger().error(f"Controller Error: {str(e)}")
            return

        self.publish_control(steer, accel)

        self.get_logger().info(f"[INFO] Velocità: {current_speed:.2f} m/s | Punti path: {len(self.reference_path)}")
        self.get_logger().info(f"[INFO] Sterzo calcolato: {math.degrees(steer):.2f}° | Accelerazione: {accel:.2f}")
        
        udp_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, steer, accel]).astype(np.float64)
        # udp_data = np.array([steer, accel, current_speed], dtype=np.float64)
        udp_send(self.socket_udp, (self.host, self.port), udp_data.tobytes())

    def publish_control(self, steering_angle, acceleration):
        steer_msg = Float64()
        steer_msg.data = math.degrees(steering_angle)
        self.steer_pub.publish(steer_msg)
        
        accel_msg = Float64()
        accel_msg.data = float(acceleration)
        self.accel_pub.publish(accel_msg)

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


