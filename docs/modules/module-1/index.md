# Module 1: ROS 2 Foundations

## Overview
Welcome to Module 1: ROS 2 Foundations. This module provides a comprehensive introduction to the Robot Operating System 2 (ROS 2), the middleware framework that powers modern robotics applications. You'll learn the core concepts, architecture, and programming patterns that form the foundation for building complex robotic systems. This module establishes the essential knowledge needed for all subsequent modules in the AI-Humanoid Robotics curriculum.

ROS 2 is a flexible framework for writing robot software that provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It's designed to support the development of complex, distributed robotic systems with real-time performance and high reliability requirements.

## Learning Objectives
- Understand the fundamental concepts and architecture of ROS 2
- Master the core communication patterns: topics, services, and actions
- Develop proficiency in Python programming with rclpy for ROS 2
- Create and work with URDF (Unified Robot Description Format) models
- Implement complete ROS 2 systems with multiple interconnected nodes
- Apply best practices for ROS 2 development and debugging

## Key Concepts
### ROS 2 Architecture
ROS 2 is built on a distributed architecture where computation is spread across multiple processes and potentially multiple machines. The system uses DDS (Data Distribution Service) as the underlying communication middleware, providing reliable message delivery and discovery mechanisms.

### Nodes, Topics, Services, and Actions
The fundamental building blocks of ROS 2 include nodes (processes that perform computation), topics (named buses over which nodes exchange messages), services (request/response communication), and actions (goal-oriented communication with feedback).

### Quality of Service (QoS) Settings
QoS settings in ROS 2 allow fine-tuning of communication behavior including reliability, durability, and history policies. These settings are crucial for real-time and safety-critical robotic applications.

### Package and Launch System
ROS 2 packages organize code and resources, while launch files provide mechanisms to start multiple nodes with specific configurations simultaneously.

## Technical Deep Dive
ROS 2 represents a complete redesign of the original ROS framework to address real-time performance, security, and scalability requirements. The architecture is built around the DDS (Data Distribution Service) middleware, which provides a standardized interface for real-time, scalable, and fault-tolerant data exchange.

### Core Architecture Components
The ROS 2 architecture consists of several layers:
- **Application Layer**: User-written nodes and applications
- **Client Library Layer**: rclpy (Python) and rclcpp (C++)
- **ROS Client Library (rcl)**: Common C-based interface
- **DDS Abstraction Layer**: Abstracts different DDS implementations
- **DDS Implementation**: Actual DDS vendor implementations (Fast DDS, Cyclone DDS, etc.)

### Communication Patterns in Detail
**Topics** implement a publish-subscribe pattern where publishers send messages to topics without knowledge of subscribers. Subscribers receive messages from topics without knowledge of publishers. This loose coupling enables flexible system composition.

**Services** implement a request-response pattern where a client sends a request and waits for a response from a server. This synchronous communication is ideal for operations that require immediate results.

**Actions** implement a goal-oriented pattern with feedback and result reporting. They're suitable for long-running operations where progress monitoring is important.

### Quality of Service Policies
ROS 2 QoS policies include:
- **Reliability**: Reliable vs. best-effort delivery
- **Durability**: Volatile vs. transient-local history
- **History**: Keep-all vs. keep-last policies
- **Deadline**: Time constraints for data delivery
- **Liveliness**: Node availability monitoring

## Code Examples
### Basic ROS 2 Node (Python)
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### System Architecture (Text Description)
```
+-------------------+    +-------------------+    +-------------------+
|   Publisher Node  |    |   Service Server  |    |   Action Server   |
|                   |    |                   |    |                   |
| - Publishes data  |    | - Responds to     |    | - Handles long-   |
| - Topics: sensor  |    |   requests        |    |   running tasks   |
|   readings        |    | - Services: robot |    | - Actions: move   |
|                   |    |   control         |    |   to location     |
+-------------------+    +-------------------+    +-------------------+
         |                         |                         |
         |                         |                         |
         v                         v                         v
+--------+-------------------------+-------------------------+--------+
|                        ROS 2 Middleware Layer                     |
|                      (DDS Implementation)                         |
|  +----------------+  +----------------+  +----------------+      |
|  |   Topic Bus    |  |  Service Bus   |  |  Action Bus    |      |
|  | - sensor_data  |  | - robot_cmd    |  | - move_to_goal |      |
|  | - cmd_vel      |  | - get_status   |  | - trajectory   |      |
|  +----------------+  +----------------+  +----------------+      |
+-------------------------------------------------------------------+
         |                         |                         |
         |                         |                         |
         v                         v                         v
+-------------------+    +-------------------+    +-------------------+
|   Subscriber      |    |   Service Client  |    |   Action Client   |
|   Node            |    |                   |    |                   |
| - Subscribes to   |    | - Sends requests  |    | - Sends goals     |
|   sensor data     |    | - Robot control   |    | - Monitors        |
| - Processes data  |    |   commands        |    |   progress        |
+-------------------+    +-------------------+    +-------------------+
```

## Common Pitfalls
- **Node Discovery Issues**: Nodes may fail to communicate due to network configuration or DDS implementation conflicts
- **QoS Mismatch**: Publishers and subscribers with incompatible QoS settings won't communicate
- **Resource Management**: Forgetting to destroy nodes properly can lead to memory leaks
- **Threading Issues**: Improper handling of callbacks can cause race conditions
- **Package Dependencies**: Missing or incorrect package dependencies can prevent nodes from running

## Checkpoints / Mini-Exercises
1. Create a publisher node that publishes temperature readings and a subscriber that logs them to console
2. Implement a service that calculates the distance between two points and test it with a client
3. Set up a launch file that starts multiple nodes with different QoS settings
4. Create a custom message type and use it in a publisher-subscriber pair
5. Debug a communication issue between nodes using ROS 2 command-line tools

## References
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [DDS Specification](https://www.omg.org/spec/DDS/About-DDS/)
