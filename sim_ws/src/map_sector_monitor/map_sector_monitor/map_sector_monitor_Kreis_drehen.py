import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class TimedMapSectorMonitor(Node):
    def __init__(self):
        super().__init__('timed_map_sector_monitor')

        self.sector_count = 16
        self.threshold = 1  # Radius in Metern

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None  # Orientierung (Yaw)
        self.map_msg = None

        # Subscriptions
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.pose_callback,
            10)

        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/grid_prob_map',
            self.map_callback,
            10)

        # Bool-Publisher je Sektor
        self.bool_publishers = [
            self.create_publisher(Bool, f'/bool_topic_{i+1}', 10)
            for i in range(self.sector_count)
        ]

        # Timer für regelmäßige Auswertung
        self.create_timer(1.0, self.check_collisions)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Berechne Yaw aus Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def check_collisions(self):
        if self.map_msg is None or self.robot_x is None or self.robot_yaw is None:
            self.get_logger().warn('Karte oder Pose noch nicht verfügbar.')
            return

        msg = self.map_msg
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Status je Sektor initialisieren
        blocked = [False] * self.sector_count

        for i, val in enumerate(msg.data):
            if val < 50:
                continue  # Nur belegte Zellen berücksichtigen

            col = i % width
            row = i // width
            cell_x = col * resolution + origin_x
            cell_y = row * resolution + origin_y

            dx = cell_x - self.robot_x
            dy = cell_y - self.robot_y
            distance = math.hypot(dx, dy)

            if distance > self.threshold:
                continue  # Außerhalb des Radius

            # Winkel relativ zum Roboter
            angle = math.atan2(dy, dx)
            relative_angle = (angle - self.robot_yaw + 2 * math.pi) % (2 * math.pi)

            # Sektor berechnen
            sector = int(relative_angle / (2 * math.pi / self.sector_count))
            blocked[sector] = True

        # Ergebnisse veröffentlichen
        for i in range(self.sector_count):
            msg = Bool()
            msg.data = blocked[i]
            self.bool_publishers[i].publish(msg)
            self.get_logger().info(f"Published {msg.data} on /bool_topic_{i+1}")

def main(args=None):
    rclpy.init(args=args)
    node = TimedMapSectorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
