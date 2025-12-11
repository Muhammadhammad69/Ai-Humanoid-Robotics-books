# Chapter 3: Python Programming with rclpy

## Overview
This chapter focuses on developing ROS 2 nodes using Python and the rclpy client library. You'll learn how to create, structure, and run Python-based ROS 2 nodes that can participate in complex robotic systems. We'll cover the essential patterns, best practices, and advanced techniques for building robust Python nodes that can handle real-world robotic applications including sensor processing, control systems, and communication with other nodes.

Python is one of the most popular languages for robotics development due to its simplicity, extensive libraries, and strong support for scientific computing. The rclpy library provides a Python interface to ROS 2's core functionality, enabling rapid prototyping and development of robotic applications while maintaining the benefits of ROS 2's distributed architecture.

## Learning Objectives
- Master the fundamentals of rclpy for Python-based ROS 2 development
- Create nodes that implement different communication patterns (topics, services, actions)
- Implement proper node lifecycle management and resource handling
- Apply threading and async patterns for responsive node behavior
- Use ROS 2 parameters and logging effectively in Python nodes
- Structure complex Python nodes for maintainability and reusability

## Key Concepts

### rclpy Architecture
rclpy is the Python client library for ROS 2 that provides Python bindings to the ROS Client Library (rcl). It handles the low-level communication with the DDS middleware while providing a Pythonic interface for creating nodes, publishers, subscribers, and other ROS 2 constructs.

### Node Structure and Lifecycle
ROS 2 nodes in Python follow a standard structure with initialization, execution, and cleanup phases. Proper lifecycle management is crucial for resource handling and system stability, especially in long-running robotic applications.

### Asynchronous Programming
Effective ROS 2 nodes often use asynchronous programming patterns to handle multiple tasks concurrently, such as processing sensor data while responding to service requests and updating parameters.

### Parameter Management
ROS 2 provides a parameter system that allows nodes to be configured at runtime, making them more flexible and reusable across different robotic platforms and scenarios.

## Technical Deep Dive

### rclpy Fundamentals

The rclpy library provides Python access to ROS 2 functionality through a set of classes and functions that mirror the ROS 2 concepts:

**Node Class**: The base class for all ROS 2 nodes in Python. It provides methods for creating publishers, subscribers, services, and other ROS 2 entities.

**Initialization**: The `rclpy.init()` function initializes the ROS 2 client library and must be called before creating any nodes.

**Spinning**: The `rclpy.spin()` function keeps the node running and processes callbacks, messages, and service requests.

### Node Lifecycle Management

A properly structured Python node should follow these lifecycle steps:

1. **Initialization**: Set up the node, create publishers/subscribers, initialize internal state
2. **Execution**: Process callbacks, timers, and other events
3. **Cleanup**: Destroy resources and clean up before shutdown

### Threading and Concurrency

ROS 2 nodes can use different execution models:
- **Single-threaded**: Default executor processes callbacks sequentially
- **Multi-threaded**: MultiThreadedExecutor processes callbacks in parallel
- **Custom**: Custom executors for specialized use cases

### Parameter Handling

Parameters in rclpy allow nodes to be configured at runtime:
- Declared with default values and constraints
- Accessible via command line, launch files, or parameter server
- Changeable during runtime with callbacks

### Python Node Architecture (Text Diagram)
```
+-----------------------------------------------------------+
|                   Python Node Architecture                |
|                                                           |
|  +-------------------+     +-------------------------+    |
|  |   rclpy Library   |---->|   ROS 2 Middleware    |    |
|  |   (Python)        |     |   (DDS Implementation)|    |
|  +-------------------+     +-------------------------+    |
|           |                           |                    |
|           v                           v                    |
|  +-------------------+     +-------------------------+    |
|  |   Node Class      |     |   Topic/Service/Action  |    |
|  |   (User Code)     |---->|   Communication       |    |
|  |                   |     |   Infrastructure      |    |
|  | - Publishers      |     |                       |    |
|  | - Subscribers     |     |                       |    |
|  | - Services        |     |                       |    |
|  | - Actions         |     |                       |    |
|  | - Parameters      |     |                       |    |
|  | - Timers          |     |                       |    |
|  +-------------------+     +-------------------------+    |
|           |                           |                    |
|           v                           v                    |
|  +-------------------+     +-------------------------+    |
|  |   Callback Queue  |---->|   Message Queue       |    |
|  |   (Executor)      |     |   (DDS/RMW)           |    |
|  |                   |     |                       |    |
|  | - Timer Callbacks |     | - Incoming Messages   |    |
|  | - Subscription    |     | - Service Requests    |    |
|  |   Callbacks       |     | - Action Goals        |    |
|  | - Service         |     |                       |    |
|  |   Callbacks       |     |                       |    |
|  +-------------------+     +-------------------------+    |
+-----------------------------------------------------------+
```

## Code Examples

### Basic Node Structure with Parameters

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ParameterizedNode(Node):
    """Example node demonstrating parameter usage"""

    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('sensor_range', 10.0)

        # Access parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.sensor_range = self.get_parameter('sensor_range').value

        # Create publisher with QoS profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(String, 'robot_status', qos_profile)

        # Create timer based on parameter
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info(
            f'Node initialized: {self.robot_name}, '
            f'publish rate: {self.publish_rate}Hz, '
            f'sensor range: {self.sensor_range}m'
        )

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.robot_name} is operational at {self.publish_rate}Hz'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Complex Node with Multiple Communication Patterns

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from example_interfaces.action import Fibonacci
from example_interfaces.srv import SetBool
import threading
import time


class ComplexRobotController(Node):
    """Advanced node demonstrating multiple communication patterns"""

    def __init__(self):
        super().__init__('complex_robot_controller')

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.position_publisher = self.create_publisher(Float64, 'current_position', 10)

        # Subscribers
        self.odometry_subscriber = self.create_subscription(
            Float64, 'odometry', self.odometry_callback, 10)

        # Service Server
        self.service = self.create_service(
            SetBool, 'enable_controller', self.service_callback)

        # Action Server
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_fibonacci,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Internal state
        self.controller_enabled = True
        self.current_position = 0.0
        self.active_goals = {}

        # Timer for position updates
        self.position_timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('Complex robot controller initialized')

    def odometry_callback(self, msg):
        """Handle incoming odometry data"""
        self.current_position = msg.data
        if self.controller_enabled:
            self.get_logger().debug(f'Position updated: {self.current_position}')

    def publish_position(self):
        """Publish current position periodically"""
        msg = Float64()
        msg.data = self.current_position
        self.position_publisher.publish(msg)

    def service_callback(self, request, response):
        """Handle enable/disable service requests"""
        self.controller_enabled = request.data
        response.success = True
        response.message = f'Controller {"enabled" if self.controller_enabled else "disabled"}'
        self.get_logger().info(f'Controller {response.message}')
        return response

    def goal_callback(self, goal_request):
        """Accept or reject goals"""
        self.get_logger().info(f'Received goal request: {goal_request}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject goal cancellation requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_fibonacci(self, goal_handle):
        """Execute the Fibonacci action"""
        self.get_logger().info('Executing goal...')

        # Create feedback and result messages
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()

        # Generate Fibonacci sequence
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.sequence = feedback_msg.sequence
                self.get_logger().info('Goal canceled')
                return result_msg

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug(f'Feedback: {feedback_msg.sequence[-1]}')

            # Sleep to simulate work
            time.sleep(0.5)

        goal_handle.succeed()
        result_msg.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded with result: {result_msg.sequence}')
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    node = ComplexRobotController()

    # Use multi-threaded executor for concurrent processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Launch File for the Complex Node
```python
# complex_robot_controller_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot',
        description='Name of the robot'
    )

    # Get launch configuration
    robot_name = LaunchConfiguration('robot_name')

    # Create the complex robot controller node
    controller_node = Node(
        package='my_robot_package',
        executable='complex_robot_controller',
        name='complex_controller',
        parameters=[
            {'robot_name': robot_name},
            {'publish_rate': 10.0},
            {'sensor_range': 5.0}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_mux/input/teleop'),
            ('odometry', 'odom')
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_node
    ])
```

## Common Pitfalls
- **Threading Issues**: Improper handling of callbacks can cause race conditions. Use appropriate synchronization mechanisms when accessing shared data.
- **Resource Management**: Forgetting to destroy nodes and their components can lead to memory leaks in long-running systems.
- **Parameter Validation**: Not validating parameter values can lead to runtime errors or unexpected behavior.
- **Exception Handling**: Unhandled exceptions in callbacks can crash the entire node. Always implement proper error handling.
- **Timer Precision**: Using timers for time-critical applications without considering system load and scheduling latency.

## Checkpoints / Mini-Exercises
1. Create a Python node that subscribes to sensor data and publishes processed information using appropriate QoS settings
2. Implement a parameterized node that changes its behavior based on runtime parameters
3. Build a node that uses both services and actions to provide different types of functionality
4. Create a launch file that starts multiple instances of your node with different parameter configurations
5. Develop a node that properly handles exceptions and gracefully degrades when components fail

## References
- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Python Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-in-a-class-CPP.html)
- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)