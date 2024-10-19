import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class TestPubParams(Node):

    def __init__(self):
        super().__init__("test_pub_params")

        self.publisher_ = self.create_publisher(String, "test_topic", 10)
        self.timer_ = self.create_timer(0.5, self.publish_topic)
        self.get_logger().info("Node Publishing ... ")

    def publish_topic(self):
        msg = String()
        msg.data = "Secret Data"
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = TestPubParams()
    rclpy.spin(node)  # keeps the node alive
    rclpy.shutdown()


if __name__ == '__main__':
    main()
