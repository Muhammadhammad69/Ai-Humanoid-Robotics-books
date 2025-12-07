---
id: chapter-3-python-rclpy
title: Bridging Python Agents with rclpy
sidebar_position: 3
---

# Bridging Python Agents with `rclpy`

## Overview
This chapter focuses on `rclpy`, the official Python client library for ROS 2. `rclpy` allows Python developers to easily write ROS 2 nodes, enabling powerful integration of Python-based AI agents, machine learning algorithms, and high-level control logic into complex robotic systems, including humanoid platforms. We will explore how `rclpy` facilitates the creation of nodes, publishers, subscribers, services, and the use of parameters and custom messages, which are essential for building sophisticated robotic behaviors.

## Learning Objectives
- Understand the role and capabilities of `rclpy` in ROS 2 development.
- Learn to implement advanced `rclpy` features such as parameters.
- Master the creation and use of custom ROS 2 messages for complex data types.
- Grasp the basics of ROS 2 Actions and their implementation in `rclpy`.
- Integrate Python-based AI logic into ROS 2 nodes using `rclpy`.

## Key Concepts
### `rclpy` Client Library
`rclpy` is the Python client library for ROS 2. It provides a Pythonic interface to all core ROS 2 concepts, allowing Python programs to create nodes, communicate via topics and services, use parameters, and interact with actions. Its ease of use and the vast ecosystem of Python libraries make it a popular choice for AI and robotics applications.

### ROS 2 Parameters
Parameters are dynamic, configurable values associated with a ROS 2 node. They allow for runtime configuration of node behavior without recompiling the code. For example, a humanoid robot's walking gait node might expose parameters for stride length or walking speed, which can be adjusted on the fly.

### Custom Messages
While ROS 2 provides a rich set of standard message types, complex robotic applications often require custom data structures. Custom messages (`.msg` files) allow developers to define application-specific data types, ensuring efficient and type-safe communication between nodes. This is particularly useful for humanoid robots communicating complex sensor data (e.g., full body joint angles, force distributions) or intricate control commands.

### ROS 2 Actions
Actions (`.action` files) are a high-level communication mechanism designed for long-running, goal-oriented tasks. Unlike services, actions provide continuous feedback on their progress and allow for preemption (canceling an ongoing task). This is ideal for tasks like a humanoid robot navigating to a target location or performing a complex manipulation sequence.

## Technical Deep Dive

### Working with Parameters in `rclpy`
ROS 2 parameters are essentially key-value pairs. Nodes can declare, get, set, and describe parameters.
A Python node declares a parameter, then provides a callback to handle changes.

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'hello')
        self.declare_parameter('some_integer', 0)

        # Set up a callback to be notified when a parameter is changed
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_parameter':
                self.get_logger().info(f"Parameter 'my_parameter' changed to: {param.value}")
            elif param.name == 'some_integer':
                self.get_logger().info(f"Parameter 'some_integer' changed to: {param.value}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
To set a parameter from the CLI:
```bash
ros2 param set /parameter_node my_parameter "new_value"
ros2 param set /parameter_node some_integer 10
```

### Using Custom Messages with `rclpy`
First, define your custom message in a `.msg` file within a ROS 2 package (e.g., `my_robot_interfaces/msg/JointCommand.msg`):

```
# JointCommand.msg
string joint_name
float32 position
float32 velocity
```

After building your package, `rclpy` will generate Python classes for this message type.
A publisher using a custom message:
```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import JointCommand # Import your custom message

class CustomMessagePublisher(Node):

    def __init__(self):
        super().__init__('custom_message_publisher')
        self.publisher_ = self.create_publisher(JointCommand, 'joint_commands', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_pos = 0.0

    def timer_callback(self, ):
        msg = JointCommand()
        msg.joint_name = "shoulder_pan_joint"
        msg.position = self.current_pos
        msg.velocity = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Joint Command: {msg.joint_name}, Pos: {msg.position:.2f}')
        self.current_pos += 0.1
        if self.current_pos > 1.0:
            self.current_pos = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = CustomMessagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
And a corresponding subscriber would import `JointCommand` and use it in its callback.

### Basics of ROS 2 Actions in `rclpy`
Actions typically involve a goal, feedback, and result. A common example is a "NavigateToPose" action for a mobile robot. For humanoid robots, this could be a "PerformGait" or "ManipulateObject" action.

Define an action (e.g., `my_robot_interfaces/action/PerformTask.action`):

```
# Goal
string task_name
int32 target_iterations
---
# Result
int32 total_iterations_completed
bool success
---
# Feedback
float32 percentage_complete
string current_status
```
`rclpy` provides `action_server` and `action_client` classes to implement these.

## Common Pitfalls
- **Parameter Type Mismatch**: Attempting to set a parameter with a value of an incompatible type.
- **Custom Message Generation**: Forgetting to build the ROS 2 package after defining new `.msg` or `.action` files, leading to Python import errors.
- **Action Server vs. Service Server**: Using a service for a long-running task that requires continuous feedback, when an action would be more appropriate.

## Checkpoints / Mini-Exercises
1.  Create a Python node that declares a parameter for a humanoid robot's "head_tilt_angle" and allows it to be changed via the `ros2 param set` command.
2.  Define a custom message `FingerPosition.msg` with fields for `finger_index` (int32) and `curl_angle` (float32). Implement a publisher and subscriber for this custom message.
3.  Outline a simple action for a humanoid robot to "WaveHand". What would be the goal, feedback, and result for this action?

## References
-   [ROS 2 Documentation - About Parameters](https://docs.ros.org/en/humble/Concepts/About-Parameters.html)
-   [ROS 2 Documentation - Parameters (rclpy)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Understanding-ROS2-Parameters.html)
-   [ROS 2 Documentation - Custom Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
-   [ROS 2 Documentation - About Actions](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
-   [ROS 2 Documentation - Actions (rclpy)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Understanding-ROS2-Actions.html)