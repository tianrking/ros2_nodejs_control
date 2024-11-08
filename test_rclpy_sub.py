import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            '/topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)

    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()