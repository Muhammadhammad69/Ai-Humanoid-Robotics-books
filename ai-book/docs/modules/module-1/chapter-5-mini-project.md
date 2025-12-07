---
id: chapter-5-mini-project
title: Hands-On Projects & Exercises
sidebar_position: 5
---

# Hands-On Projects & Exercises

## Overview
This chapter provides practical, hands-on projects and exercises that consolidate the concepts learned in previous chapters. By engaging with these mini-projects, you will gain practical experience in implementing ROS 2 nodes, using `rclpy` for Python-based control, and working with URDF models for humanoid robots. These exercises are designed to be challenging yet achievable, providing a tangible sense of accomplishment and preparing you for more complex robotic endeavors.

## Learning Objectives
- Apply ROS 2 concepts (nodes, topics, services) in practical scenarios.
- Implement Python-based control logic for a humanoid robot using `rclpy`.
- Integrate URDF models with ROS 2 for simulation and control.
- Develop basic teleoperation capabilities for a robotic system.
- Reinforce understanding of ROS 2 communication patterns through practical projects.

## Key Concepts
### Teleoperation
Teleoperation refers to operating a robot remotely. In ROS 2, this typically involves a teleoperation node (e.g., receiving keyboard or joystick commands) publishing commands to another robot control node (e.g., joint controller). This is a fundamental concept for controlling humanoid robots, allowing human operators to guide their movements.

### Joint State Publishing
For robots to be visualized and controlled effectively in ROS 2, their current joint positions (joint states) need to be published to a specific topic, typically `/joint_states`. This information is consumed by visualization tools like RViz and by robot state publishers.

### Mini-Project Design
Effective mini-projects break down complex tasks into manageable steps, allowing learners to build upon their knowledge incrementally. Each project should have clear goals, provide necessary code examples, and include checkpoints for validation.

## Technical Deep Dive

### Teleoperating the 2-Link Humanoid Arm
We will extend our 2-link humanoid arm model from Chapter 4 and implement a simple teleoperation script using `rclpy`. This script will read keyboard inputs and publish commands to control the shoulder and elbow joints of our robot.

**System Architecture for Teleoperation (Text Description)**:
```
+-----------------+       +-----------------+       +-----------------+
|   Keyboard Input| ----> | Teleop Node (Py)| ----> | Joint Command   |
| (User)          |       | (Publisher)     |       | Topic           |
+-----------------+       +-----------------+       +-----------------+
                                    |                         ^
                                    v                         |
                                +-----------------+       +-----------------+
                                | Joint Commander | <---- | Joint States    |
                                | Node (Sub/Pub)  |       | Topic (Sensor)  |
                                +-----------------+       +-----------------+
                                          |
                                          v
                                +-----------------+
                                |  2-Link Humanoid|
                                |  Arm (URDF)     |
                                +-----------------+
```
The Teleop Node translates keyboard presses into joint commands, which are then published to a topic. A Joint Commander Node subscribes to these commands and updates the simulated (or real) robot's joints.

## Code Examples

### Python Script for Simple Teleoperation (using `rclpy`)
This example demonstrates a basic teleoperation node that reads keyboard input to control a simulated 2-link arm. You would need to have `pynput` or a similar library installed (e.g., `pip install pynput`) and handle non-blocking input appropriately in a real ROS 2 setup. For simplicity, this is a conceptual example for a ROS 2 node that would interact with a joint command publisher.

Let's assume we have `my_robot_interfaces/msg/JointCommand.msg` as defined in Chapter 3.

```python
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import JointCommand # Import your custom message
import termios, sys, os # For non-blocking keyboard input

# Global variable to store key press
KEY_PRESS = ''

# Function to get a single character input without waiting for Enter
def getch():
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)
    new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)
    try:
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(JointCommand, 'joint_commands', 10)
        self.joint_positions = {'shoulder_joint': 0.0, 'elbow_joint': 0.0}
        self.speed = 0.1 # Radians per key press
        self.create_timer(0.1, self.timer_callback) # Check for input periodically

        self.get_logger().info('Teleop Node Started. Use "qweasd" to control joints. Press "space" to exit.')
        self.get_logger().info('q/a: Shoulder Joint | w/s: Elbow Joint')

    def timer_callback(self):
        # This part would typically be handled by a blocking input listener in a separate thread
        # For conceptual example, assume KEY_PRESS is updated by an external input mechanism
        global KEY_PRESS

        try:
            char = getch() # Attempt to get input
            KEY_PRESS = char
        except:
            pass # No input, continue

        if KEY_PRESS == 'q': # Shoulder up
            self.joint_positions['shoulder_joint'] += self.speed
            self._publish_command('shoulder_joint', self.joint_positions['shoulder_joint'])
        elif KEY_PRESS == 'a': # Shoulder down
            self.joint_positions['shoulder_joint'] -= self.speed
            self._publish_command('shoulder_joint', self.joint_positions['shoulder_joint'])
        elif KEY_PRESS == 'w': # Elbow up
            self.joint_positions['elbow_joint'] += self.speed
            self._publish_command('elbow_joint', self.joint_positions['elbow_joint'])
        elif KEY_PRESS == 's': # Elbow down
            self.joint_positions['elbow_joint'] -= self.speed
            self._publish_command('elbow_joint', self.joint_positions['elbow_joint'])
        elif KEY_PRESS == ' ':
            self.get_logger().info('Exiting teleop node.')
            raise SystemExit # Exit condition

        KEY_PRESS = '' # Reset key press

    def _publish_command(self, joint_name, position):
        msg = JointCommand()
        msg.joint_name = joint_name
        msg.position = position
        msg.velocity = 0.5 # Example velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding {joint_name} to {position:.2f}')

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    try:
        rclpy.spin(teleop_node)
    except SystemExit: # Catch the exit
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Save this as `simple_teleop.py`*

### Python Script for Publishing Joint States
A node that publishes the current state of the robot's joints. This is crucial for visualization in RViz.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # Publish at 20 Hz

        # Initial joint positions
        self.shoulder_pos = 0.0
        self.elbow_pos = 0.0
        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_joint', 'elbow_joint'] # Names from our URDF
        
        # Simulate some movement (e.g., oscillating)
        self.shoulder_pos = math.sin(self.time * 0.5) * 0.5 # oscillates between -0.5 and 0.5 rad
        self.elbow_pos = math.cos(self.time * 0.7) * 0.7 + 0.7 # oscillates between 0 and 1.4 rad
        
        msg.position = [self.shoulder_pos, self.elbow_pos]
        msg.velocity = [] # Can be empty if not simulating velocities
        msg.effort = []   # Can be empty if not simulating efforts

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Joint States: Shoulder={self.shoulder_pos:.2f}, Elbow={self.elbow_pos:.2f}')
        self.time += 0.05

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Save this as `joint_state_publisher.py`*

## Common Pitfalls
-   **Blocking Input**: Teleoperation nodes can become unresponsive if keyboard input is handled in a blocking manner.
-   **Coordinate Frame Issues**: Incorrectly defining coordinate frames in URDF or when publishing transforms can lead to visualization errors in RViz.
-   **Missing Robot State Publisher**: For visualization in RViz, you typically need a `robot_state_publisher` node running which reads the URDF and `joint_states` and publishes the robot's full kinematic tree.

## Checkpoints / Mini-Exercises
1.  Implement a more robust teleoperation node that uses `curses` or `pynput` for non-blocking keyboard input and provides more granular control over joint speeds.
2.  Integrate the `simple_teleop.py` with a simulated robot (e.g., using `ros2_control` and Gazebo if set up) to control its arm in real-time.
3.  Visualize the `joint_state_publisher.py` output in RViz by setting up a basic RViz configuration for your 2-link arm.

## References
-   [ROS 2 Documentation - Tutorials (Create a teleop package)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Python-Package-And-Node.html) (Conceptual guide for package creation)
-   [ROS 2 Documentation - Tutorials (Robot State Publisher)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-With-Robot-State-Publisher.html)
-   [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
-   [pynput library documentation](https://pynput.readthedocs.io/en/latest/)