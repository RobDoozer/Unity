import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import threading
import queue
import math

# Thread-sicherer Queue-Puffer
incoming_text = queue.Queue()

class LidarScanNode(Node):
    def __init__(self):
        super().__init__('lidar_scan_node')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.05, self.publish_scan)

        self.ranges = [float('inf')] * 500  # 500 Punkte wegen 0.72°
        self.received = set()
        self.last_publish_time = self.get_clock().now()

    def publish_scan(self):
        while not incoming_text.empty():
            raw = incoming_text.get()
            try:
                angle_str, range_str = raw.strip().split(",")
                angle_deg = float(angle_str)
                range_val = float(range_str)
                angle_index = int(angle_deg / 0.72) % 500

                # Debug-Ausgabe
                self.get_logger().debug(f"Empfangen: {angle_deg:.1f}° → {range_val:.2f} m → Index {angle_index}")

                # Speichern
                self.ranges[angle_index] = range_val
                self.received.add(angle_index)
            except ValueError:
                self.get_logger().warn(f"Ungültige Daten empfangen: {raw}")
                continue

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        last_sec = self.last_publish_time.nanoseconds * 1e-9
        time_diff = now_sec - last_sec

        if time_diff >= 0.1:
            if all(math.isinf(r) for r in self.ranges):
                self.get_logger().warn("Kein einziger gültiger Messwert empfangen — Scan übersprungen.")
                return

            msg = LaserScan()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'laser'

            msg.angle_min = 0.0
            msg.angle_max = 2 * math.pi
            msg.angle_increment = math.radians(0.72)
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.1
            msg.range_max = 10.0
            msg.ranges = self.ranges.copy()

            self.publisher.publish(msg)
            self.get_logger().info(f"Scan veröffentlicht (Messpunkte: {len(self.received)}) {msg.ranges[0]}")
            

            self.ranges = [float('inf')] * 500
            self.received.clear()
            self.last_publish_time = now


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    print("[ROS2] Warte auf UDP-Daten von Unity...")

    while True:
        data, _ = sock.recvfrom(65536)
        text = data.decode('utf-8')
        for line in text.strip().split("\n"):
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
