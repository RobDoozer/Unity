import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class TestScan(Node):
    def __init__(self):
        super().__init__("test_scan")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.timer = self.create_timer(0.1, self.publish)
        self.scan = LaserScan()
        self.scan.header.frame_id = "base_link"
        self.scan.angle_min = 0.0
        self.scan.angle_max = 6.28
        self.scan.range_min = 0.1
        self.scan.range_max = 10.0
        self.scan.ranges = [1.0] * 500  # 500 gültige Werte

    def publish(self):
        self.scan.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.scan)

rclpy.spin(TestScan())
