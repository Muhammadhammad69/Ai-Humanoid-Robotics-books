# Chapter 1: Introduction to ROS 2

## Overview
This chapter introduces the fundamental concepts of ROS 2 (Robot Operating System 2), the next-generation middleware framework for robotics. We'll explore the evolution from ROS 1 to ROS 2, understand the core architectural changes, and examine how ROS 2 addresses the needs of modern robotics applications including humanoid robots. By the end of this chapter, you'll have a solid understanding of ROS 2's architecture and be ready to start building your first ROS 2 applications.

ROS 2 represents a significant advancement over ROS 1, incorporating modern software engineering practices, improved security, real-time capabilities, and support for commercial and industrial applications. The framework provides a rich set of tools and libraries that enable rapid development of complex robotic systems.

## Learning Objectives
- Understand the history and evolution of ROS from version 1 to 2
- Explain the key architectural differences between ROS 1 and ROS 2
- Identify the core components and concepts of ROS 2
- Set up a basic ROS 2 development environment
- Run your first ROS 2 nodes and understand basic communication
- Use ROS 2 command-line tools for system introspection

## Key Concepts

### ROS 2 vs ROS 1
ROS 2 addresses several limitations of ROS 1 including single-threaded execution model, lack of security features, and absence of real-time support. The new architecture uses DDS (Data Distribution Service) as the underlying communication middleware, providing reliable message delivery and discovery mechanisms.

### DDS Middleware
DDS (Data Distribution Service) is an OMG standard for real-time, scalable, and fault-tolerant data exchange. In ROS 2, DDS handles node discovery, message routing, and Quality of Service (QoS) policies. Multiple DDS implementations are supported including Fast DDS, Cyclone DDS, and RTI Connext DDS.

### Nodes and Communication
Nodes in ROS 2 are processes that perform computation. They communicate through topics (publish-subscribe), services (request-response), and actions (goal-oriented communication). Each node has a unique name and can participate in multiple communication patterns simultaneously.

### Quality of Service (QoS)
QoS settings allow fine-tuning of communication behavior including reliability, durability, and history policies. These settings are crucial for real-time and safety-critical robotic applications, allowing developers to specify requirements for message delivery and data persistence.

## Technical Deep Dive

### Architecture Evolution
ROS 2 represents a complete redesign of the original ROS framework to address real-time performance, security, and scalability requirements. The key architectural changes include:

**DDS Integration**: ROS 2 uses DDS as the underlying communication middleware, providing a standardized interface for real-time, scalable, and fault-tolerant data exchange. This allows ROS 2 to support real-time applications and provides better network discovery and message delivery guarantees.

**Multi-threaded Execution**: Unlike ROS 1's single-threaded execution model, ROS 2 supports multi-threaded execution, enabling better performance for complex robotic applications.

**Security**: ROS 2 includes security features including authentication, access control, and encryption, making it suitable for commercial and industrial applications.

### Core Components

**Nodes**: The fundamental building blocks of ROS 2 applications. Each node runs independently and communicates with other nodes through messages.

**Topics**: Named buses over which nodes exchange messages using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics.

**Services**: Request-response communication pattern where a client sends a request and waits for a response from a server.

**Actions**: Goal-oriented communication with feedback and result reporting, suitable for long-running operations.

### Quality of Service Policies

ROS 2 QoS policies provide fine-grained control over communication behavior:

- **Reliability Policy**: Controls whether messages are delivered reliably or with best-effort
- **Durability Policy**: Determines whether messages persist for late-joining subscribers
- **History Policy**: Controls how many messages are stored for each topic
- **Deadline Policy**: Specifies timing constraints for message delivery
- **Liveliness Policy**: Monitors node availability

### ROS 2 Ecosystem

The ROS 2 ecosystem includes:
- **Client Libraries**: rclpy (Python), rclcpp (C++), and others
- **Tools**: Command-line tools for introspection, debugging, and visualization
- **Package System**: colcon build system and package management
- **Launch System**: Launch files for starting multiple nodes with configurations

### Basic ROS 2 Architecture (Text Diagram)
```
+-----------------------------------------------------------+
|                        ROS 2 System                       |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  |   Node A       |    |   Node B       |    |  Node C  | |
|  | (Publisher)    |    | (Subscriber)   |    | (Server) | |
|  |                |    |                |    |          | |
|  | - Publishes to |    | - Subscribes to|    | - Provides| |
|  |   /sensor_data |    |   /cmd_vel     |    |   service| |
|  +----------------+    +----------------+    +----------+ |
|         |                        |                   |    |
|         |                        |                   |    |
|         v                        v                   v    |
|  +----------------+    +----------------+    +----------+ |
|  |   Topic Bus    |    |   Topic Bus    |    | Service  | |
|  | (/sensor_data) |    | (/cmd_vel)     |    | Bus      | |
|  |                |    |                |    |          | |
|  | - Reliable     |    | - Best-effort  |    | - Sync   | |
|  | - Keep-all     |    | - Keep-last    |    | - Async  | |
|  +----------------+    +----------------+    +----------+ |
|         |                        |                   |    |
|         +------------------------+-------------------+    |
|                            |                                |
|                    +-------v-------+                        |
|                    | DDS Middleware|                        |
|                    | (FastDDS,     |                        |
|                    | CycloneDDS,   |                        |
|                    | ConnextDDS)   |                        |
|                    +---------------+                        |
+-----------------------------------------------------------+
```

## Code Examples

### Basic Publisher Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

### Basic Subscriber Node
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

### Running the Nodes
To run these nodes, save them as separate Python files and execute them in different terminals:

Terminal 1:
```bash
ros2 run my_package minimal_publisher
```

Terminal 2:
```bash
ros2 run my_package minimal_subscriber
```

## Common Pitfalls
- **Node Discovery Issues**: Nodes may fail to communicate due to network configuration or DDS implementation conflicts. Ensure both nodes use the same DDS implementation and are on the same network.
- **QoS Mismatch**: Publishers and subscribers with incompatible QoS settings won't communicate. Always ensure matching QoS policies for reliable communication.
- **Resource Management**: Forgetting to destroy nodes properly can lead to memory leaks. Always call `destroy_node()` before shutting down.
- **Package Dependencies**: Missing or incorrect package dependencies can prevent nodes from running. Ensure all required packages are properly declared in `package.xml`.
- **ROS Domain ID**: Multiple ROS 2 systems on the same network need different domain IDs to avoid interference.

## Checkpoints / Mini-Exercises
1. Set up a ROS 2 development environment and verify it works by running the talker/listener demo
2. Create a publisher node that publishes a custom message type with multiple fields
3. Use ROS 2 command-line tools to introspect the running system: `ros2 node list`, `ros2 topic list`, `ros2 topic echo`
4. Modify the publisher to use different QoS settings and observe the effects
5. Create a launch file that starts both publisher and subscriber nodes simultaneously

## References
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Specification](https://www.omg.org/spec/DDS/About-DDS/)
- [ROS 2 Design Articles](https://design.ros2.org/)