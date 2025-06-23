import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class TimedMapSectorMonitor(Node):
    def __init__(self):
        super().__init__('timed_map_sector_monitor')

        self.sector_count = 16  # Anzahl der Sektoren

        # Rechteckparameter relativ zum Roboter
        self.vorderkante = 0.5        # Meter nach vorne (Roboter-Frame +Y) (Abstand Vorderkante Rollstuhl zu KOS Rollstuhl)
        self.hinterkante = 0.5        # Meter nach hinten (-Y) (Abstand Hinterkante zu KOS Rollstuhl)
        self.rechte_kante = 0.35       # Meter nach rechts (+X) (Abstand rechte Kante zu KOS Rollstuhl)
        self.linke_kante = 0.35        # Meter nach links (-X)  (Abstand linke Kante zu KOS Rollstuhl)

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None  # Orientierung des Roboters (Yaw in Radiant)
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

        # Bool-Publisher für jeden Sektor
        self.bool_publishers = [
            self.create_publisher(Bool, f'/bool_topic_{i+1}', 10)
            for i in range(self.sector_count)
        ]

        # Timer zur regelmäßigen Auswertung
        self.create_timer(1.0, self.check_collisions)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Orientierung (Yaw) aus Quaternion berechnen
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

        # Initialisiere Blockierungsstatus je Sektor
        blocked = [False] * self.sector_count

        # Vorberechnung: Cos/Sin für Rotation ins Roboter-Frame
        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)

        for i, val in enumerate(msg.data):
            if val < 50:
                continue  # Nur Zellen mit hoher Wahrscheinlichkeit prüfen

            col = i % width
            row = i // width
            cell_x = col * resolution + origin_x
            cell_y = row * resolution + origin_y

            # Delta-Koordinaten zur Roboterposition (Map-Frame)
            dx = cell_x - self.robot_x
            dy = cell_y - self.robot_y

            # Koordinaten ins Roboter-Koordinatensystem rotieren
            local_x = dx * cos_yaw - dy * sin_yaw
            local_y = dx * sin_yaw + dy * cos_yaw

            # Prüfen, ob Zelle innerhalb des Rechtecks liegt
            if (local_x < -self.linke_kante or local_x > self.rechte_kante or
                local_y < -self.hinterkante or local_y > self.vorderkante):
                continue

            # Berechne Winkel im Roboter-Frame für Sektorzuordnung
            relative_angle = (math.atan2(local_y, local_x) + 2 * math.pi) % (2 * math.pi)

            # Sektor bestimmen
            sector = int(relative_angle / (2 * math.pi / self.sector_count))
            blocked[sector] = True

        # Ergebnis je Sektor veröffentlichen
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
