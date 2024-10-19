import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class TestSub(Node):

    def __init__(self):
        super().__init__("test_sub")

        self.subscriber_ = self.create_subscription(
            String, "test_topic", self.callback_subscribe_topic, 10)
        # self.timer_ = self.create_timer(0.5, self.publish_topic)
        self.get_logger().info("Node Subscribing ... ")

    def callback_subscribe_topic(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):

    rclpy.init(args=args)
    node = TestSub()
    rclpy.spin(node)  # keeps the node alive
    rclpy.shutdown()


if __name__ == '__main__':
    main()
