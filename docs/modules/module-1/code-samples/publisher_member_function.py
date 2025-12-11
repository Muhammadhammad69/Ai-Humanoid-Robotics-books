import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard ROS 2 message type for strings

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher that publishes String messages to the 'chatter' topic
        # The queue_size is 10, meaning it will buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.i = 0
        # Create a timer that calls the timer_callback function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy library
    minimal_publisher = MinimalPublisher() # Create the node
    rclpy.spin(minimal_publisher) # Keep the node alive until it's manually stopped or ROS shuts down
    minimal_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shutdown rclpy library

if __name__ == '__main__':
    main()
