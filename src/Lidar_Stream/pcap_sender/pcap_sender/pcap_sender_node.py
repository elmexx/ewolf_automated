import rclpy
from rclpy.node import Node
from scapy.all import rdpcap, sendp, IP, UDP
import time

class PcapSenderNode(Node):
    def __init__(self):
        super().__init__('pcap_sender')
        self.declare_parameter('pcap_file', '')
        self.declare_parameter('dst_ip','localhost')
        self.declare_parameter('dst_port', 33333)

        pcap_file = self.get_parameter('pcap_file').get_parameter_value().string_value
        dst_ip = self.get_parameter('dst_ip').get_parameter_value().string_value
        dst_port = self.get_parameter('dst_port').get_parameter_value().integer_value
        self.get_logger().info("message 0")
        self.resend_packets(pcap_file, dst_ip, dst_port)

    def resend_packets(self, pcap_file, dst_ip, dst_port):
        packets = rdpcap(pcap_file)
        clk = packets[0].time
        self.get_logger().info("message 1")
        for packet in packets:
            self.get_logger().info("message 2")
            if IP in packet:
                self.get_logger().info("message 3")
                packet[IP].src = dst_ip
                if UDP in packet:
                    self.get_logger().info("message 4")
                    packet[UDP].dport = dst_port
                sendp(packet)
                self.get_logger().info("send package to %d" % dst_port)
        # for packet in packets:
        #     time.sleep(packet.time - clk)
        #     clk = packet.time
        #     sendp(packet)     

def main(args=None):
    rclpy.init(args=args)
    node = PcapSenderNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
