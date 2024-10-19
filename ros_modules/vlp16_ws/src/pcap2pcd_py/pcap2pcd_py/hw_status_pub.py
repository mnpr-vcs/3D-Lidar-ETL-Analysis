import rclpy
from rclpy.node import Node

from point_cloud_interfaces.msg import HardwareStatus

class HWStatusPub(Node):
    
    def __init__(self):
        super().__init__("hw_status_pub")
        
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hw_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info('Hardware status publishing ---')
        
    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = "Nothing here"
        self.hw_status_pub_.publish(msg)
        

def main(args=None):
    
    rclpy.init(args=args)
    node = HWStatusPub()
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()