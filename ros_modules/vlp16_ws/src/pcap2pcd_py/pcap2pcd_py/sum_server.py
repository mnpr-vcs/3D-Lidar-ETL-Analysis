import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SumServer(Node):
    
    def __init__(self):
        super().__init__("sum_server")
        
        self.server_ = self.create_service(AddTwoInts, 'sum_two_int', self.add_two_ints)
        self.get_logger().info('Server Ready for request')
        
    def add_two_ints(self, request, response):
        
        response.sum = request.a + request.b
        self.get_logger().info(f'\nRequest :{request.a} + {request.b}\nResponse :{response.sum}\n')
        return response
        

def main(args=None):
    
    rclpy.init(args=args)
    node = SumServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()