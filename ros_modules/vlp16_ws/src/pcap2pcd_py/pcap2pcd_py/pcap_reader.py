import rclpy
from rclpy.node import Node

class PcapReader(Node):
    
    def __init__(self):
        super().__init__("reader_node")
        
        self.counter_ = 0
        self.get_logger().info('Pcap Reader ... ')
        self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        
        self.counter_ += 1
        self.get_logger().info('Count :' + str(self.counter_))


def main(args=None):
    
    rclpy.init(args=args)
    
    node = PcapReader()
    
    rclpy.spin(node) # keeps the node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()