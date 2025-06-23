import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker
import math

class TimedMapSectorMonitor(Node):
    def __init__(self):
        super().__init__('timed_map_sector_monitor')

        self.sector_count = 16

        # Rechteckdimensionen in Roboter-Koordinaten
        self.hinterkante = 1.5
        self.vorderkante = 1.0
        self.rechte_kante = 0.35
        self.linke_kante = 0.35

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.map_msg = None

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

        self.bool_publishers = [
            self.create_publisher(Bool, f'/bool_topic_{i+1}', 10)
            for i in range(self.sector_count)
        ]

        self.marker_pub = self.create_publisher(Marker, 'rectangle_marker', 10)
        self.marker_line_pub = self.create_publisher(Marker, 'rectangle_outline_marker', 10)
        self.marker_sector_pub = self.create_publisher(Marker, 'sector_markers', 10)

        self.create_timer(1.0, self.check_collisions)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

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

        blocked = [False] * self.sector_count

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        for i, val in enumerate(msg.data):
            if val < 50:
                continue

            col = i % width
            row = i // width
            cell_x = col * resolution + origin_x
            cell_y = row * resolution + origin_y

            dx = cell_x - self.robot_x
            dy = cell_y - self.robot_y

            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw

            if (local_x < -self.linke_kante or local_x > self.rechte_kante or
                local_y < -self.hinterkante or local_y > self.vorderkante):
                continue

            angle = (math.atan2(local_y, local_x) + 2 * math.pi) % (2 * math.pi)
            angle = (angle + math.pi) % (2 * math.pi)  # 180° drehen
            sector = int(angle / (2 * math.pi / self.sector_count))
            blocked[sector] = True

        for i in range(self.sector_count):
            msg = Bool()
            msg.data = blocked[i]
            self.bool_publishers[i].publish(msg)
            self.get_logger().info(f"Published {msg.data} on /bool_topic_{i+1}")

        self.publish_rectangle_marker()
        self.publish_rectangle_outline_marker()
        self.publish_sector_visualization()

    def compute_rectangle_center(self):
        center_offset = (self.vorderkante - self.hinterkante) / 2.0
        center_x = self.robot_x + center_offset * math.cos(self.robot_yaw)
        center_y = self.robot_y + center_offset * math.sin(self.robot_yaw)
        return center_x, center_y

    def publish_rectangle_marker(self):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle_fill"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        width = self.rechte_kante + self.linke_kante
        length = self.vorderkante + self.hinterkante
        center_x, center_y = self.compute_rectangle_center()

        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = 0.01

        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = length
        marker.scale.y = width
        marker.scale.z = 0.02

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.2

        self.marker_pub.publish(marker)

    def publish_rectangle_outline_marker(self):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle_outline"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        corners = [
            (-self.linke_kante, -self.hinterkante),
            (self.rechte_kante, -self.hinterkante),
            (self.rechte_kante, self.vorderkante),
            (-self.linke_kante, self.vorderkante),
            (-self.linke_kante, -self.hinterkante),
        ]

        center_x, center_y = self.compute_rectangle_center()
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        marker.points = []
        for x_r, y_r in corners:
            x_map = center_x + (x_r * cos_yaw - y_r * sin_yaw)
            y_map = center_y + (x_r * sin_yaw + y_r * cos_yaw)
            pt = Point(x=x_map, y=y_map, z=0.02)
            marker.points.append(pt)

        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_line_pub.publish(marker)

    def publish_sector_visualization(self):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return

        marker_lines = Marker()
        marker_lines.header.frame_id = "map"
        marker_lines.header.stamp = self.get_clock().now().to_msg()
        marker_lines.ns = "sector_lines"
        marker_lines.id = 2
        marker_lines.type = Marker.LINE_LIST
        marker_lines.action = Marker.ADD
        marker_lines.scale.x = 0.02
        marker_lines.color.r = 0.0
        marker_lines.color.g = 1.0
        marker_lines.color.b = 0.0
        marker_lines.color.a = 1.0

        center_x = self.robot_x
        center_y = self.robot_y
        radius = max(self.vorderkante + self.hinterkante, self.rechte_kante + self.linke_kante) * 1.5

        for i in range(self.sector_count):
            angle = self.robot_yaw + (2 * math.pi / self.sector_count) * i
            angle = (angle + math.pi) % (2 * math.pi)  # 180° drehen
            x_end = center_x + radius * math.cos(angle)
            y_end = center_y + radius * math.sin(angle)

            p1 = Point(x=center_x, y=center_y, z=0.05)
            p2 = Point(x=x_end, y=y_end, z=0.05)
            marker_lines.points.append(p1)
            marker_lines.points.append(p2)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "sector_text"
            text_marker.id = 100 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center_x + (radius + 0.3) * math.cos(angle)
            text_marker.pose.position.y = center_y + (radius + 0.3) * math.sin(angle)
            text_marker.pose.position.z = 0.1
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = str(i + 1)

            self.marker_sector_pub.publish(text_marker)

        self.marker_sector_pub.publish(marker_lines)


def main(args=None):
    rclpy.init(args=args)
    node = TimedMapSectorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

