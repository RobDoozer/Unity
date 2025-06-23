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
        self.threshold = 0.7  # 70 cm Radius

        self.robot_x = None
        self.robot_y = None
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

        # 16 Publisher für Bool-Themen
        self.bool_publishers = [
            self.create_publisher(Bool, f'/bool_topic_{i+1}', 10)
            for i in range(self.sector_count)
        ]

        self.create_timer(1.0, self.check_collisions)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def check_collisions(self):
        if self.map_msg is None or self.robot_x is None:
            self.get_logger().warn('Karte oder Pose noch nicht verfügbar.')
            return

        msg = self.map_msg
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        blocked = [False] * self.sector_count

        for i, val in enumerate(msg.data):
            if val < 50:
                continue

            col = i % width
            row = i // width
            cell_x = col * resolution + origin_x
            cell_y = row * resolution + origin_y

            dx = cell_x - self.robot_x
            dy = cell_y - self.robot_y
            distance = math.hypot(dx, dy)

            if distance > self.threshold:
                continue

            angle = (math.atan2(dy, dx) + 2 * math.pi) % (2 * math.pi)
            # Drehe Winkel so, dass 0 Grad = Norden
            angle = (angle + math.pi / 2) % (2 * math.pi)
            sector = int(angle / (2 * math.pi / self.sector_count))
            blocked[sector] = True

        # Publiziere jeden Sektor als Bool
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

