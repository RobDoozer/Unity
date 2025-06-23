import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MultiBoolRelayNode(Node):
    def __init__(self):
        super().__init__('multi_bool_relay_node')

        self.topic_names = [f'/bool_topic_{i+1}' for i in range(16)]
        self.last_states = {name: None for name in self.topic_names}

        self.publisher_dict = {}
        self.subscription_list = []  # Name geändert

        for topic in self.topic_names:
            unity_topic = topic + '_unity'
            self.publisher_dict[topic] = self.create_publisher(Bool, unity_topic, 10)

            sub = self.create_subscription(
                Bool,
                topic,
                self.make_callback(topic),
                10
            )
            self.subscription_list.append(sub)
            self.get_logger().info(f'Subscribed to {topic}, publishing to {unity_topic}')

    def make_callback(self, topic_name):
        def callback(msg):
            last_state = self.last_states[topic_name]
            if last_state is None:
                self.get_logger().info(f'Initial state for {topic_name}: {msg.data}')
            elif msg.data != last_state:
                self.get_logger().info(f'State change on {topic_name}: {last_state} -> {msg.data}')

                out_msg = Bool()
                out_msg.data = msg.data
                self.publisher_dict[topic_name].publish(out_msg)
                self.get_logger().info(f'Published to {topic_name}_unity: {msg.data}')

            self.last_states[topic_name] = msg.data

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = MultiBoolRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

