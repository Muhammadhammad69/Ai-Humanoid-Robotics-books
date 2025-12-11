import sys
import rclpy
from rclpy.node import Node
from my_ros2_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        # Create a service client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request() # Create a request object

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Call the service asynchronously

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run my_ros2_pkg add_client <int_a> <int_b>')
        return -1

    minimal_client = AddTwoIntsClient()
    minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().error('Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
