import rclpy
from rclpy.node import Node
from my_ros2_pkg.srv import AddTwoInts # Import your custom service type

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service server that listens on the 'add_two_ints' service
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending response: sum: %d' % response.sum)
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_service = AddTwoIntsService()
    rclpy.spin(add_two_ints_service)
    add_two_ints_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
