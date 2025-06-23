import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MultiBoolTestNode(Node):
    def __init__(self):
        super().__init__('multi_bool_test_node')

        self.topic_names = [f'/bool_topic_{i+1}' for i in range(16)]
        self.publishers_list = [self.create_publisher(Bool, topic, 10) for topic in self.topic_names]

        self.current_index = 0
        self.current_value = False

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Sekunde Intervall

    def timer_callback(self):
        msg = Bool()
        msg.data = self.current_value

        publisher = self.publishers_list[self.current_index]
        publisher.publish(msg)
        self.get_logger().info(f'Published {msg.data} on {self.topic_names[self.current_index]}')

        # Nächster Boolean in der Liste, toggle Wert
        self.current_index += 1
        if self.current_index >= len(self.topic_names):
            self.current_index = 0
            self.current_value = not self.current_value

def main(args=None):
    rclpy.init(args=args)
    node = MultiBoolTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

