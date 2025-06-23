#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import math
import socket
import json
import threading

PORT = 5000

def quaternion_to_euler(x, y, z, w):
    """
    Convert Quaternion to Euler angles (roll, pitch, yaw) in radians.
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def rotate_quaternion_x(qx, qy, qz, qw, angle_rad):
    """
    Rotate quaternion by angle_rad around X axis.
    """
    # Rotation quaternion around X axis
    sin_half = math.sin(angle_rad / 2.0)
    cos_half = math.cos(angle_rad / 2.0)
    rx, ry, rz, rw = sin_half, 0.0, 0.0, cos_half

    # Quaternion multiplication: q_new = r * q
    new_w = rw*qw - rx*qx - ry*qy - rz*qz
    new_x = rw*qx + rx*qw + ry*qz - rz*qy
    new_y = rw*qy - rx*qz + ry*qw + rz*qx
    new_z = rw*qz + rx*qy - ry*qx + rz*qw

    return new_x, new_y, new_z, new_w

class OdometryTCPServer(Node):
    def __init__(self, host='0.0.0.0', port=PORT):
        super().__init__('odometry_tcp_server')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 30)
        self.get_logger().info(f"Starte TCP-Server auf Port {port}...")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, port))
        self.sock.listen(5)

        thread = threading.Thread(target=self.listen_loop, daemon=True)
        thread.start()

    def listen_loop(self):
        while True:
            conn, addr = self.sock.accept()
            with conn:
                try:
                    data = conn.recv(1024)
                    if data:
                        json_str = data.decode('utf-8')
                        odom_data = json.loads(json_str)
                        self.publish_odometry(odom_data)
                except Exception as e:
                    self.get_logger().error(f"Fehler beim Empfangen: {e}")

    def publish_odometry(self, data):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        # Koordinatensystem anpassen: y <-> z tauschen
        ros_x = data["x"]
        ros_y = data["z"]
        ros_z = data["y"]

        msg.pose.pose.position = Point(x=ros_x, y=ros_y, z=ros_z)

        # Quaternion rotieren um -90° um X Achse, um Unity-Koordinatensystem zu ROS zu konvertieren
        qx, qy, qz, qw = data["qx"], data["qy"], data["qz"], data["qw"]
        # negative 90 Grad = -pi/2
        new_qx, new_qy, new_qz, new_qw = rotate_quaternion_x(qx, qy, qz, qw, -math.pi/2)
        msg.pose.pose.orientation = Quaternion(x=new_qx, y=new_qy, z=new_qz, w=new_qw)

        self.odom_pub.publish(msg)

        # Euler-Winkel aus konvertierter Quaternion
        roll, pitch, yaw = quaternion_to_euler(new_qx, new_qy, new_qz, new_qw)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(
            f"Odometry gesendet: x={ros_x:.2f}, y={ros_y:.2f}, z={ros_z:.2f}, "
            f"roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°, yaw={yaw_deg:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTCPServer(port=PORT)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

