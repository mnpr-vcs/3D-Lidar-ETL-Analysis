import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

from functools import partial

class SumClient(Node):
    
    def __init__(self):
        super().__init__("sum_client")
        
        self.client_ = self.create_client(AddTwoInts, 'sum_two_int')
        self.call_sum_server(1, 66)
        
        
    def call_sum_server(self, a, b):
        
        # wait for server
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Server')
        
        # request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # response
        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_response, a = a, b = b))
        
    def callback_response(self, future, a, b):
             
        try:
            response = future.result()
            self.get_logger().info(f'\nRequest :{a} + {b}\nResponse :{response.sum}\n')
        except Exception as e:
            self.get_logger().error('Service call failed %r' %(e,))

def main(args=None):
    
    rclpy.init(args=args)
    node = SumClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()