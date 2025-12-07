---
id: chapter-2-ros2-comm
title: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services

## Overview
This chapter delves into the fundamental communication mechanisms within ROS 2: Nodes, Topics, and Services. These concepts are the building blocks for any ROS 2 application, enabling modularity and distributed computing across various components of a robotic system. Understanding how these elements interact is crucial for developing robust and scalable control systems for humanoid robots, allowing for seamless integration of perception, planning, and actuation.

## Learning Objectives
- Define what a ROS 2 Node is and its role in a robotic system.
- Understand the concept of ROS 2 Topics for asynchronous data streaming.
- Learn how to create and manage ROS 2 Publishers and Subscribers using `rclpy`.
- Comprehend the functionality of ROS 2 Services for synchronous request-reply communication.
- Implement ROS 2 Service Servers and Clients using `rclpy`.
- Utilize ROS 2 command-line interface (CLI) tools to inspect nodes, topics, and services.

## Key Concepts
### ROS 2 Nodes
A Node is an executable process that performs computation. In a ROS 2 system, multiple nodes can cooperate to achieve a larger goal. For example, a humanoid robot might have separate nodes for camera processing, motor control, and navigation.

### ROS 2 Topics
Topics are a fundamental communication mechanism in ROS 2 for asynchronous, many-to-many data streaming. Nodes publish data (messages) to topics, and other nodes subscribe to those topics to receive the data. This is ideal for continuous streams of information like sensor readings (e.g., joint states, camera feeds) or control commands.

### ROS 2 Services
Services provide a synchronous request-reply communication model. A service client sends a request to a service server, and the server processes the request and sends a response back to the client. This is suitable for operations that require an immediate response, like querying a robot's current state or triggering a specific action.

## Technical Deep Dive

Nodes encapsulate specific functionalities. The `rclpy` client library in Python allows developers to easily create and manage these nodes, along with their publishers, subscribers, service servers, and clients.

### Node Communication with Topics
Publishers send messages on a specific topic, while subscribers receive them. The message type defines the data structure being communicated.
```
Node (Publisher) --(message: JointState)--> Topic: /joint_states --(message: JointState)--> Node (Subscriber)
```
For humanoid robots, topics are heavily used for broadcasting joint states, receiving desired joint positions, or distributing sensory data like force-torque readings from end-effectors.

### Node Communication with Services
Services involve a client-server paradigm. The client initiates a request, and the server responds.
```
Node (Client) --(request: AddTwoInts.Request)--> Service: /add_two_ints --(response: AddTwoInts.Response)--> Node (Server)
```
This can be used for discrete commands, such as requesting a humanoid robot to switch between walking gaits or to perform a specific manipulation sequence.

### Quality of Service (QoS) Policies
ROS 2 employs QoS policies to define communication characteristics like reliability, durability, and history. These are crucial for optimizing performance and ensuring message delivery guarantees, especially in real-time or safety-critical humanoid applications.

## Code Examples

### Simple Python Publisher (using `rclpy`)
This example demonstrates a node publishing string messages to a topic.

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
        msg.data = f'Hello from publisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
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
*Save this as `minimal_publisher.py`*

### Simple Python Subscriber (using `rclpy`)
This example demonstrates a node subscribing to the same topic and receiving messages.

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
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Save this as `minimal_subscriber.py`*

### Running the examples (conceptual `ros2 cli` commands)
First, ensure your Python scripts are executable (e.g., `chmod +x minimal_publisher.py`). Then, you might run them within a ROS 2 package or directly if paths are set up.

To run the publisher:
```bash
ros2 run <your_package_name> minimal_publisher
```
To run the subscriber (in a separate terminal):
```bash
ros2 run <your_package_name> minimal_subscriber
```

To inspect topics:
```bash
ros2 topic echo /topic
```

## Common Pitfalls
- **Message Type Mismatch**: Publishers and subscribers must use the exact same message type.
- **QoS Incompatibility**: Incompatible QoS settings between a publisher and subscriber can prevent communication.
- **Node Naming Conflicts**: Duplicate node names in the same ROS 2 domain can lead to unexpected behavior.

## Checkpoints / Mini-Exercises
1.  Modify the publisher example to send `sensor_msgs/msg/JointState` messages instead of `std_msgs/msg/String`.
2.  Create a service server that takes two integers and returns their sum, and a corresponding client that calls this service.
3.  Experiment with `ros2 topic bw` and `ros2 topic hz` to analyze topic performance.

## References
-   [ROS 2 Documentation - Concepts](https://docs.ros.org/en/humble/Concepts.html)
-   [ROS 2 Documentation - Tutorials (Writing a Simple Publisher and Subscriber)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
-   [ROS 2 Documentation - Tutorials (Writing a Simple Service and Client)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
-   [ROS 2 Documentation - Quality of Service Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)