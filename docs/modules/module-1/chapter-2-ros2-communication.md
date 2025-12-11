# Chapter 2: ROS 2 Communication Patterns

## Overview
This chapter delves deep into the core communication mechanisms of ROS 2: topics, services, and actions. We'll explore how these communication patterns enable different types of interactions between nodes in a robotic system. Understanding these patterns is crucial for designing effective robotic architectures, particularly for humanoid robots that require coordinated sensing, planning, and control. You'll learn when to use each pattern and how to implement them effectively in your robotic applications.

Communication in ROS 2 is built around three fundamental patterns that serve different purposes: topics for asynchronous data streaming, services for synchronous request-response interactions, and actions for goal-oriented operations with feedback. Each pattern has its own use cases and characteristics that make it suitable for specific types of robot communication needs.

## Learning Objectives
- Master the three core communication patterns in ROS 2: topics, services, and actions
- Understand the appropriate use cases for each communication pattern
- Implement publisher and subscriber nodes for topic-based communication
- Create service servers and clients for request-response communication
- Develop action servers and clients for goal-oriented operations
- Apply Quality of Service (QoS) settings appropriately for different communication needs

## Key Concepts

### Topics (Publish-Subscribe)
Topics implement an asynchronous, many-to-many communication pattern where publishers send messages to named topics and subscribers receive messages from those topics. This loose coupling allows for flexible system design where publishers and subscribers don't need to know about each other.

### Services (Request-Response)
Services provide synchronous communication where a client sends a request to a server and waits for a response. This pattern is ideal for operations that require immediate results or for remote procedure calls between nodes.

### Actions (Goal-Oriented)
Actions are designed for long-running operations that require goal setting, feedback during execution, and final results. They're perfect for operations like navigation, manipulation, or any task that takes time to complete and requires monitoring.

### Quality of Service (QoS) Policies
QoS policies allow fine-tuning of communication behavior for each pattern, specifying requirements for reliability, durability, history, deadlines, and liveliness to match the needs of different robotic applications.

## Technical Deep Dive

### Topic Communication

Topics in ROS 2 use a publish-subscribe pattern where data flows from publishers to subscribers through named channels. This pattern is ideal for sensor data distribution, robot state broadcasting, and other scenarios where data needs to be shared with multiple interested parties.

**Topic Characteristics:**
- Asynchronous: Publishers don't wait for subscribers
- Many-to-many: Multiple publishers and subscribers can use the same topic
- Data-driven: Communication is triggered by data availability
- Stateless: No memory of previous interactions

**QoS Settings for Topics:**
- **Reliability**: Reliable (all messages delivered) or Best-effort (messages may be dropped)
- **Durability**: Volatile (late-joining subscribers miss data) or Transient-local (historical data available)
- **History**: Keep-all (store all messages) or Keep-last (store only recent messages)
- **Depth**: Number of messages to store when using Keep-last

### Service Communication

Services implement synchronous request-response communication. A service client sends a request to a service server, which processes the request and returns a response. This pattern is ideal for operations that require immediate results.

**Service Characteristics:**
- Synchronous: Client waits for server response
- One-to-one: One client communicates with one server at a time
- Request-response: Structured communication with defined inputs and outputs
- Stateful: Server may maintain state between requests

**Service Message Structure:**
- Request: Data sent from client to server
- Response: Data sent from server to client
- Defined in .srv files with request and response sections

### Action Communication

Actions are designed for long-running operations that may take significant time to complete. They provide goal setting, feedback during execution, and result reporting.

**Action Characteristics:**
- Goal-oriented: Client sends a goal to server
- Feedback: Server provides periodic updates on progress
- Result: Server sends final outcome when complete
- Cancelable: Operations can be canceled mid-execution
- Unique IDs: Multiple concurrent goals can be tracked

**Action Message Structure:**
- Goal: Request sent from client to server
- Feedback: Periodic updates from server to client
- Result: Final outcome from server to client

### Communication Pattern Comparison (Text Diagram)
```
TOPICS (Publish-Subscribe):
Publisher Node A         Topic Bus         Subscriber Node B
     |------------------->|------------------->|
     |                    |                    |
     | (sensor_data)      | (sensor_data)      | (processes data)
     |                    |                    |
     v                    v                    v
  [Data Stream]      [ROS 2 Middleware]    [Data Processing]

SERVICES (Request-Response):
Client Node A            Service Bus          Server Node B
     |------------------->|------------------->|
     | Request            | Request            | Process
     |<-------------------|<-------------------|
     | Response           | Response           | Response
     |<-------------------|<-------------------|
     | (receives result)  |                    | (sends result)

ACTIONS (Goal-Oriented):
Client Node A            Action Server        Server Node C
     |------------------->|------------------->|
     | Goal               | Goal               | Execute
     |                    |                    | Task
     |<-------------------|<-------------------|
     | Feedback           | Feedback           | Send
     |                    |                    | Progress
     |<-------------------|<-------------------|
     | Result             | Result             | Complete
```

## Code Examples

### Topic Publisher and Subscriber Example

Publisher Node:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publisher for sensor data
        self.publisher_ = self.create_publisher(LaserScan, 'laser_scan', 10)

        # Create timer to publish data periodically
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Simulate sensor data
        self.angle = 0.0

    def publish_sensor_data(self):
        msg = LaserScan()

        # Set up laser scan parameters
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate simulated range data
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [2.0 + 0.5 * math.sin(self.angle + i * 0.1) for i in range(num_readings)]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published laser scan with {len(msg.ranges)} readings')
        self.angle += 0.01

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Subscriber Node:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # Create subscription to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.scan_callback,
            10)  # QoS profile
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Process laser scan data to detect obstacles
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Min obstacle distance: {min_distance:.2f}m')

            # Check for close obstacles
            if min_distance < 1.0:
                self.get_logger().warn('OBSTACLE DETECTED - TOO CLOSE!')

                # Could trigger emergency stop or navigation planning here
                self.handle_obstacle_detected(min_distance)

    def handle_obstacle_detected(self, distance):
        # Placeholder for obstacle handling logic
        self.get_logger().info(f'Initiating obstacle avoidance for distance: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Example

Service Definition (in .srv file):
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Service Server:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Service Client:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Pitfalls
- **Topic QoS Mismatch**: Publishers and subscribers with incompatible QoS settings won't communicate. Always ensure matching policies for reliable communication.
- **Service Blocking**: Service calls block the calling thread; use async calls when possible to avoid blocking the main execution flow.
- **Action Goal Conflicts**: Multiple action clients can interfere with each other if not properly managed with unique goal IDs.
- **Memory Leaks**: Forgetting to destroy nodes and publishers/subscribers can lead to resource leaks in long-running systems.
- **Network Discovery**: Nodes may not discover each other due to network configuration issues or different DDS implementations.

## Checkpoints / Mini-Exercises
1. Create a topic publisher that broadcasts robot joint states and a subscriber that logs the data
2. Implement a service that calculates the Euclidean distance between two 3D points
3. Design an action server that simulates robot movement to a goal position with feedback
4. Configure different QoS settings for a high-frequency sensor topic vs. a low-frequency command topic
5. Create a launch file that starts multiple publisher/subscriber pairs with different QoS configurations

## References
- [ROS 2 Communication Patterns](https://docs.ros.org/en/humble/Concepts/About-Topics-Services-Actions.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html)
- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Actions-In-Python.html)