import socket
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Bool


class TimedValue:
    def __init__(self, default_value: float):
        self.value = float(default_value)
        self.last_update_time: Optional[float] = None
        self.default_value = float(default_value)

    def update(self, value: float, now_sec: float):
        self.value = float(value)
        self.last_update_time = now_sec

    def get(self, now_sec: float, timeout_sec: float) -> float:
        if self.last_update_time is None:
            return self.default_value
        if timeout_sec > 0.0 and (now_sec - self.last_update_time) > timeout_sec:
            return self.default_value
        return self.value


class UdpSenderNode(Node):
    def __init__(self):
        super().__init__('udp_sender_node')

        self.declare_parameter('target_host', '10.42.0.10')
        self.declare_parameter('target_port', 5500)
        self.declare_parameter('dtype', 'float64')
        self.declare_parameter('enable_send', True)
        self.declare_parameter('send_period_sec', 0.05)
        self.declare_parameter('payload_length', 12)

        self.declare_parameter('use_speedgoat_topics', True)
        self.declare_parameter('use_radar_target_topics', True)

        self.declare_parameter('topic_steer', '/steering_angle')
        self.declare_parameter('topic_accel', '/acceleration')
        self.declare_parameter('topic_dist_rel', '/radar_target/dist_rel')
        self.declare_parameter('topic_v_rel', '/radar_target/v_rel')
        self.declare_parameter('topic_obj_valid', '/radar_target/obj_valid')

        self.declare_parameter('slot_steer', 6)
        self.declare_parameter('slot_accel', 7)
        self.declare_parameter('slot_dist_rel', 8)
        self.declare_parameter('slot_v_rel', 9)
        self.declare_parameter('slot_obj_valid', 10)
        self.declare_parameter('slot_counter', 11)

        self.declare_parameter('default_steer', 0.0)
        self.declare_parameter('default_accel', 0.0)
        self.declare_parameter('default_dist_rel', 0.0)
        self.declare_parameter('default_v_rel', 0.0)
        self.declare_parameter('default_obj_valid', 0.0)

        self.declare_parameter('timeout_steer_sec', 0.3)
        self.declare_parameter('timeout_accel_sec', 0.3)
        self.declare_parameter('timeout_dist_rel_sec', 0.3)
        self.declare_parameter('timeout_v_rel_sec', 0.3)
        self.declare_parameter('timeout_obj_valid_sec', 0.3)
        
        self.declare_parameter('counter_max', 1000000)
        self.declare_parameter('debug_print_payload', True)

        self.target_host = self.get_parameter('target_host').value
        self.target_port = int(self.get_parameter('target_port').value)
        self.dtype = self.get_parameter('dtype').value
        self.enable_send = bool(self.get_parameter('enable_send').value)
        self.send_period_sec = float(self.get_parameter('send_period_sec').value)
        self.payload_length = int(self.get_parameter('payload_length').value)

        self.use_speedgoat_topics = bool(self.get_parameter('use_speedgoat_topics').value)
        self.use_radar_target_topics = bool(self.get_parameter('use_radar_target_topics').value)

        self.topic_steer = self.get_parameter('topic_steer').value
        self.topic_accel = self.get_parameter('topic_accel').value
        self.topic_dist_rel = self.get_parameter('topic_dist_rel').value
        self.topic_v_rel = self.get_parameter('topic_v_rel').value
        self.topic_obj_valid = self.get_parameter('topic_obj_valid').value

        self.slot_steer = int(self.get_parameter('slot_steer').value)
        self.slot_accel = int(self.get_parameter('slot_accel').value)
        self.slot_dist_rel = int(self.get_parameter('slot_dist_rel').value)
        self.slot_v_rel = int(self.get_parameter('slot_v_rel').value)
        self.slot_obj_valid = int(self.get_parameter('slot_obj_valid').value)
        self.slot_counter = int(self.get_parameter('slot_counter').value)

        self.timeout_steer_sec = float(self.get_parameter('timeout_steer_sec').value)
        self.timeout_accel_sec = float(self.get_parameter('timeout_accel_sec').value)
        self.timeout_dist_rel_sec = float(self.get_parameter('timeout_dist_rel_sec').value)
        self.timeout_v_rel_sec = float(self.get_parameter('timeout_v_rel_sec').value)
        self.timeout_obj_valid_sec = float(self.get_parameter('timeout_obj_valid_sec').value)

        self.counter_max = int(self.get_parameter('counter_max').value)
        self.debug_print_payload = bool(self.get_parameter('debug_print_payload').value)

        self.steer_value = TimedValue(self.get_parameter('default_steer').value)
        self.accel_value = TimedValue(self.get_parameter('default_accel').value)
        self.dist_rel_value = TimedValue(self.get_parameter('default_dist_rel').value)
        self.v_rel_value = TimedValue(self.get_parameter('default_v_rel').value)
        self.obj_valid_value = TimedValue(self.get_parameter('default_obj_valid').value)

        self.counter = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (self.target_host, self.target_port)

        if self.use_speedgoat_topics:
            self.create_subscription(Float64, self.topic_steer, self.steer_callback, 20)
            self.create_subscription(Float64, self.topic_accel, self.accel_callback, 20)

        if self.use_radar_target_topics:
            self.create_subscription(Float32, self.topic_dist_rel, self.dist_rel_callback, 20)
            self.create_subscription(Float32, self.topic_v_rel, self.v_rel_callback, 20)
            self.create_subscription(Bool, self.topic_obj_valid, self.obj_valid_callback, 20)

        self.timer = self.create_timer(self.send_period_sec, self.send_udp)

        self.get_logger().info(
            f"UDP sender started: target={self.target_host}:{self.target_port}, "
            f"period={self.send_period_sec}s, payload_length={self.payload_length}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def steer_callback(self, msg: Float64):
        self.steer_value.update(msg.data, self.now_sec())

    def accel_callback(self, msg: Float64):
        self.accel_value.update(msg.data, self.now_sec())

    def dist_rel_callback(self, msg: Float32):
        # self.get_logger().info(f"dist_rel callback: {msg.data}")
        self.dist_rel_value.update(msg.data, self.now_sec())

    def v_rel_callback(self, msg: Float32):
        # self.get_logger().info(f"v_rel callback: {msg.data}")
        self.v_rel_value.update(msg.data, self.now_sec())

    def obj_valid_callback(self, msg: Bool):
        # self.get_logger().info(f"obj_valid callback: {msg.data}")
        self.obj_valid_value.update(1.0 if msg.data else 0.0, self.now_sec())

    def set_slot(self, payload, index: int, value: float):
        if 0 <= index < len(payload):
            payload[index] = float(value)

    def build_payload(self):
        now = self.now_sec()
        payload = [0.0] * self.payload_length

        steer = self.steer_value.get(now, self.timeout_steer_sec)
        accel = self.accel_value.get(now, self.timeout_accel_sec)
        dist_rel = self.dist_rel_value.get(now, self.timeout_dist_rel_sec)
        v_rel = self.v_rel_value.get(now, self.timeout_v_rel_sec)
        obj_valid = self.obj_valid_value.get(now, self.timeout_obj_valid_sec)

        self.set_slot(payload, self.slot_steer, steer)
        self.set_slot(payload, self.slot_accel, accel)
        self.set_slot(payload, self.slot_dist_rel, dist_rel)
        self.set_slot(payload, self.slot_v_rel, v_rel)
        self.set_slot(payload, self.slot_obj_valid, obj_valid)
        self.set_slot(payload, self.slot_counter, self.counter)

        return payload
    
    def increment_counter(self):
        self.counter += 1
        if self.counter > self.counter_max:
            self.counter = 0
        # self.get_logger().info(f"counter={self.counter}")

    def send_udp(self):
        try:
            payload = self.build_payload()

            if self.dtype == 'float64':
                arr = np.array(payload, dtype=np.float64)
            elif self.dtype == 'float32':
                arr = np.array(payload, dtype=np.float32)
            else:
                self.get_logger().error(f"Unsupported dtype: {self.dtype}")
                return

            raw = arr.tobytes()

            if self.enable_send:
                self.sock.sendto(raw, self.server_address)

            if self.debug_print_payload:
                self.get_logger().info(
                    f"counter={self.counter}, "
                    f"dist_rel={payload[self.slot_dist_rel]}, "
                    f"v_rel={payload[self.slot_v_rel]}, "
                    f"obj_valid={payload[self.slot_obj_valid]}"
                )
                
            self.increment_counter()
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()