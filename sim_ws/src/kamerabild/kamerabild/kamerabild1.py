#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import base64
import numpy as np
import cv2
import subprocess
import os
import time

class UnityImageReceiver(Node):
    def __init__(self):
        super().__init__('unity_image_receiver')

        # Subscription für Unity Bilddaten
        self.subscription = self.create_subscription(
            StringMsg,
            'unity_camera/image',  # Unity-Topic
            self.listener_callback,
            10
        )

        # Publisher für Image-Nachricht
        self.image_publisher = self.create_publisher(
            ImageMsg,
            '/image_raw',
            10
        )

        # Publisher für String-Nachricht, die das Bild protokolliert
        self.log_publisher = self.create_publisher(
            StringMsg,
            '/image_log',
            10
        )

        self.bridge = CvBridge()

        # Starte RViz2 mit gespeicherter Konfiguration
        self.launch_rviz2()

        self.get_logger().info('Unity Image Receiver Node started.')

    def launch_rviz2(self):
        rviz_config = os.path.expanduser('~/unity_image_view.rviz')  # Pfad zur .rviz-Datei
        if os.path.exists(rviz_config):
            try:
                subprocess.Popen(['rviz2', '-d', rviz_config])
                self.get_logger().info('RViz2 gestartet.')
            except Exception as e:
                self.get_logger().error(f"Fehler beim Start von RViz2: {e}")
        else:
            self.get_logger().warn('RViz2-Konfigurationsdatei nicht gefunden.')

    def listener_callback(self, msg: StringMsg):
        try:
            # Bilddaten dekodieren
            img_data = base64.b64decode(msg.data)

            # Debugging: Gib die Größe der dekodierten Daten aus, um zu sehen, ob Bilddaten empfangen wurden
            self.get_logger().info(f"Bilddaten empfangen. Größe: {len(img_data)} Bytes")

            np_arr = np.frombuffer(img_data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn('Leeres Bild empfangen')
                return

            # Bild als ROS-Nachricht konvertieren
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Bilddaten veröffentlichen
            self.image_publisher.publish(ros_image)

            # Protokollnachricht für das Bild senden
            log_msg = StringMsg()
            log_msg.data = f"Bild empfangen und veröffentlicht: {cv_image.shape[0]}x{cv_image.shape[1]} px"
            self.log_publisher.publish(log_msg)

            self.get_logger().info(f"Bild empfangen: {cv_image.shape[0]}x{cv_image.shape[1]} px")

        except Exception as e:
            self.get_logger().error(f"Fehler beim Verarbeiten des Bildes: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UnityImageReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

