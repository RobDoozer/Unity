import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import threading
import queue
import math

incoming_text = queue.Queue()

class LidarScanNode(Node):
    def __init__(self):
        super().__init__('lidar_scan_node')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.05, self.publish_scan)

        self.ranges = [float('inf')] * 360
        self.received = set()

def publish_scan(self):
        while not incoming_text.empty():
            raw = incoming_text.get()
            try:
                angle_str, range_str = raw.strip().split(",")
                angle_deg = int(round(float(angle_str))) % 360
                range_val = float(range_str)
                self.ranges[angle_deg] = range_val
                self.received.add(angle_deg)
            except ValueError:
                self.get_logger().warn(f"Ungültige Daten empfangen: {raw}")
                continue

        if len(self.received) >= 360:
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.angle_min = 0.0
            msg.angle_max = 2 * math.pi
            msg.angle_increment = math.radians(1.0)
            msg.time_increment = 0.0
            msg.scan_time = 1.0
            msg.range_min = 0.1
            msg.range_max = 10.0
            msg.ranges = self.ranges.copy()

            self.publisher.publish(msg)
            self.get_logger().info("Scan veröffentlicht (360°)")

            # Zurücksetzen für nächsten Scan
            self.ranges = [float('inf')] * 360
            self.received.clear()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    print("[ROS2] Warte auf UDP-Daten von Unity...")

    buffer = ""
    while True:
        data, _ = sock.recvfrom(65536)
        text = data.decode('utf-8')
        buffer += text
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            incoming_text.put(line)

def main(args=None):
    rclpy.init(args=args)
    threading.Thread(target=udp_listener, daemon=True).start()
    node = LidarScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

