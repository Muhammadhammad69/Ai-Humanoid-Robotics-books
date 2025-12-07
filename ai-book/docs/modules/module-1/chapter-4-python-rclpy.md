# Chapter 4 - Python ROS Control Using `rclpy`

## Overview

In this chapter, we bridge the gap between Python-based AI agents and ROS 2 robot controllers using `rclpy`, the Python client library for ROS 2. You will learn how to leverage Python to create powerful ROS 2 nodes that can issue movement commands and directly influence the behavior of a simulated humanoid robot's joints. This is a critical step in enabling high-level AI algorithms to interact with the physical world.

---

## Learning Objectives
-   Understand the role and advantages of `rclpy` for Python-ROS 2 integration.
-   Create a ROS 2 Python node to publish joint commands.
-   Bridge Python AI agent logic to ROS controllers for basic robot movement.
-   Control individual joints of a simulated humanoid robot.

---

## `rclpy` Overview: Python's Gateway to ROS 2

`rclpy` is the official Python client library for ROS 2, providing a full-featured API to interact with the ROS 2 ecosystem. It allows Python developers to create ROS 2 nodes, publishers, subscribers, service clients, and service servers, effectively bringing the power of Python's vast ecosystem (e.g., machine learning, data analysis) to robotics.

The primary advantages of `rclpy` include:
*   **Ease of Use**: Python's syntax and extensive libraries make rapid prototyping and development straightforward.
*   **Integration with AI**: Many AI algorithms and frameworks (TensorFlow, PyTorch, etc.) are Python-native, making `rclpy` ideal for integrating these with robot control.
*   **Asynchronous Support**: `rclpy` is built with a strong focus on asynchronous programming, allowing for efficient handling of multiple callbacks and long-running operations.

In essence, `rclpy` enables your high-level AI decision-making code, often written in Python, to "speak" to the low-level robot control systems that communicate over ROS 2 topics and services.

## Bridging Python AI Agents to ROS Controllers

Imagine an AI agent that decides a humanoid robot needs to wave its hand. This high-level decision needs to be translated into specific joint angle commands and sent to the robot's motor controllers. `rclpy` facilitates this translation.

The typical flow for an AI agent controlling a robot through `rclpy` is:
1.  **Perception**: The AI agent receives sensor data (e.g., camera feed, joint states) via ROS 2 topics (using `rclpy` subscribers).
2.  **Decision-Making**: The AI processes this data and decides on an action (e.g., "move arm to wave").
3.  **Command Generation**: The AI agent translates the high-level action into low-level robot commands (e.g., desired joint positions or velocities).
4.  **Actuation**: The AI agent sends these commands to the robot's controllers via ROS 2 topics or services (using `rclpy` publishers or service clients).

## Writing Python Movement Commands: Joint Control Example

Let's consider a simplified example where we want to control a single joint of a humanoid robot. Many robot control systems use topics to receive joint commands, often defined by messages like `std_msgs/msg/Float64` for a single joint value or `sensor_msgs/msg/JointState` for multiple joints.

For this example, we'll assume a generic `JointPositionCommand` message type for simplicity, containing a `joint_name` (string) and `position` (float64). If this message type doesn't exist, you'd typically define it in a custom ROS 2 package, similar to how we defined `AddTwoInts` service in Chapter 3.

Let's assume a custom message `JointPositionCommand.msg` in `my_robot_interfaces/msg/`:
```
string joint_name
float64 position
```

### Example: Simple Joint Position Publisher

First, make sure you have your custom message type generated and your workspace built. For this example, we will assume the message type `JointPositionCommand` is available within the `my_robot_interfaces` package.

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import JointPositionCommand # Assuming custom message

class JointCommander(Node):

    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(JointPositionCommand, '/joint_commands', 10)
        self.declare_parameter('joint_name', 'shoulder_joint') # Default joint name
        self.declare_parameter('target_position', 0.0)      # Default target position
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Joint Commander Node started.')

    def timer_callback(self):
        joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        target_position = self.get_parameter('target_position').get_parameter_value().double_value

        msg = JointPositionCommand()
        msg.joint_name = joint_name
        msg.position = target_position
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: Joint="{msg.joint_name}", Position={msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()
    rclpy.spin(joint_commander)
    joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this, you would:
1.  **Define the custom message**: Create `my_robot_interfaces` package and `msg/JointPositionCommand.msg`.
2.  **Update `package.xml` and `CMakeLists.txt`** for `my_robot_interfaces` to build the custom message.
3.  **Build your workspace**: `colcon build`.
4.  **Source your workspace**: `source install/setup.bash`.
5.  **Run the node**: `ros2 run <your_pkg_name> joint_commander` (assuming you put `joint_commander.py` in your package and updated `setup.py`).

You can then change the target joint and position using ROS 2 parameters:
```bash
ros2 param set /joint_commander joint_name "elbow_joint"
ros2 param set /joint_commander target_position 0.5
```
You could subscribe to `/joint_commands` with `ros2 topic echo /joint_commands` to see the commands being sent.

### Controlling Robot Joints in a Simulated Humanoid

In a simulated humanoid, a controller (often part of a `ros2_control` setup) would subscribe to a topic like `/joint_commands` (or a more specific one like `/robot/joint_group_controller/commands`). The controller then translates these commands into actions on the simulated joints, updating their positions or velocities in the physics engine.

Your Python AI agent would simply need to generate the appropriate `JointPositionCommand` messages (or whatever message type the robot's controller expects) and publish them to the correct topic. The `rclpy` library handles all the underlying communication details.

---

## Key Takeaways

*   `rclpy` is the Python client library for ROS 2, enabling seamless integration of Python AI agents with robot control systems.
*   AI agents use `rclpy` to receive sensor data and send actuation commands via ROS 2 topics and services.
*   Controlling robot joints often involves publishing messages with desired joint positions or velocities to specific command topics.
*   ROS 2 parameters allow dynamic configuration of nodes, enabling AI agents to adapt their control strategies.

---

## Exercises

1.  **Implement a Joint State Subscriber**: Create a new `rclpy` node that subscribes to a hypothetical `/joint_states` topic (using `sensor_msgs/msg/JointState` message type) and prints the current position of a specific joint.
2.  **Sequential Joint Movement**: Modify the `JointCommander` example to send a sequence of joint position commands to a single joint over time (e.g., move from 0.0 to 1.0, then back to 0.0).
3.  **Basic AI Decision**: Imagine a simple AI that, if a simulated robot joint's position (`/joint_states`) is above a certain threshold, sends a command to move it back to a neutral position. Outline the `rclpy` logic for this AI agent.
4.  **Message Type Research**: Research the `control_msgs` package in ROS 2. Identify a message type within it that could be used for commanding a humanoid robot's arm, and explain how you would use it with `rclpy`.