# Chapter 6 - Mini-Project: Build the Nervous System of a Humanoid

## Overview

This mini-project is your opportunity to integrate all the concepts learned in Module 1. You will design and implement the "nervous system" for a simple humanoid robot, enabling it to communicate and coordinate its basic functions using ROS 2. This hands-on experience will solidify your understanding of ROS 2 nodes, topics, services, and URDF, preparing you for more complex humanoid robotics challenges.

---

## Learning Objectives
-   Design and implement a multi-node ROS 2 system for basic humanoid control.
-   Utilize ROS 2 topics for continuous data exchange (e.g., joint commands).
-   Implement ROS 2 services for specific request/reply interactions.
-   Create a basic URDF model representing a humanoid structure.
-   Bridge simple Python AI logic to control the humanoid's joints via ROS 2.

---

## Project Goal: The Humanoid's Nervous System

Your goal is to build a minimal, functional ROS 2 "nervous system" for a humanoid robot. This system will consist of several Python ROS 2 nodes that communicate to:
1.  **Publish basic joint commands**.
2.  **Monitor simple sensor feedback** (simulated).
3.  **Provide a service for a specific action**.
4.  **Describe the robot's physical structure** using URDF.

This mini-project encourages you to think about how different robot functionalities are modularized and interact within the ROS 2 framework.

## Step-by-Step Guide

### Step 1: Create Your ROS 2 Package

If you haven't already, create a new ROS 2 Python package in your `~/ros2_ws/src` directory.
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_nervous_system
```

### Step 2: Define Custom Messages (Optional but Recommended)

For better clarity and structure, define a custom message for joint commands and joint states within `humanoid_nervous_system/msg`.

**`JointCommand.msg`**:
```
string joint_name
float64 position
```

**`JointState.msg`**:
```
string joint_name
float64 position
float64 velocity
float64 effort
```

Remember to update `package.xml` and `CMakeLists.txt` in `humanoid_nervous_system` to build these messages, then `colcon build`.

### Step 3: Implement the Joint Command Publisher Node (AI Agent)

Create a Python node, e.g., `ai_commander_node.py`, that acts as a simple AI agent. This node should:
*   Publish `JointCommand` messages to a `/joint_command` topic.
*   Periodically publish commands for at least two different joints (e.g., "right_shoulder", "left_shoulder") with alternating target positions.
*   You can use parameters to set the joint names and target positions.

Example snippet for `ai_commander_node.py`:
```python
# ai_commander_node.py
import rclpy
from rclpy.node import Node
from humanoid_nervous_system.msg import JointCommand # Your custom message

class AICommander(Node):
    def __init__(self):
        super().__init__('ai_commander')
        self.publisher_ = self.create_publisher(JointCommand, '/joint_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every second
        self.joint_names = ["right_shoulder", "left_shoulder"]
        self.target_positions = [0.5, -0.5]
        self.current_joint_idx = 0
        self.get_logger().info('AI Commander Node started.')

    def timer_callback(self):
        joint_name = self.joint_names[self.current_joint_idx]
        target_position = self.target_positions[self.current_joint_idx]

        msg = JointCommand()
        msg.joint_name = joint_name
        msg.position = target_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding {joint_name} to {target_position}')

        self.current_joint_idx = (self.current_joint_idx + 1) % len(self.joint_names)

def main(args=None):
    rclpy.init(args=args)
    ai_commander = AICommander()
    rclpy.spin(ai_commander)
    ai_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Implement a Simple Joint State Monitor Node

Create another Python node, e.g., `joint_state_monitor_node.py`, that acts as a sensor feedback listener. This node should:
*   Subscribe to a `/joint_states` topic (use `sensor_msgs/msg/JointState` or your custom `JointState.msg`).
*   Print the received joint names and positions.
*   (Self-check): If you run this without a `joint_state_publisher`, you won't see data. This is normal for now.

### Step 5: Implement a Robot Action Service

Create a ROS 2 service, e.g., `reset_robot_pose`.
*   Define a service message: `ResetPose.srv` (e.g., `bool success --- string message`).
*   Implement a service *server* node, `pose_reset_service.py`, that responds to requests. For simplicity, it can just print "Robot pose reset" and return `success=True`.
*   Implement a service *client* node, `reset_client.py`, to call this service.

### Step 6: Create a Basic Humanoid URDF Skeleton

In your `humanoid_nervous_system/urdf` directory (create it), describe a simple humanoid:
*   At least three links (e.g., `torso_link`, `head_link`, `arm_link`).
*   At least two joints (e.g., `neck_joint`, `shoulder_joint`).
*   Use `fixed` joints for simplicity initially, then try `revolute`.
*   Include basic `<visual>` and `<collision>` tags for each link.

**Example snippet (`simple_humanoid.urdf`)**:
```xml
<?xml version="1.0"?>
<robot name="mini_humanoid">
  <material name="white"> <color rgba="1 1 1 1"/> </material>
  <material name="black"> <color rgba="0 0 0 1"/> </material>

  <link name="torso_link">
    <visual> <geometry><box size="0.2 0.1 0.4"/></geometry> <material name="white"/> </visual>
    <collision> <geometry><box size="0.2 0.1 0.4"/></geometry> </collision>
    <inertial> <mass value="10"/> <origin xyz="0 0 0.2"/> <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/> </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="head_link">
    <visual> <geometry><sphere radius="0.1"/></geometry> <material name="black"/> </visual>
    <collision> <geometry><sphere radius="0.1"/></geometry> </collision>
    <inertial> <mass value="1"/> <origin xyz="0 0 0"/> <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/> </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_arm_link"/>
    <origin xyz="0 0.15 0.15"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_arm_link">
    <visual> <geometry><cylinder radius="0.03" length="0.3"/></geometry> <origin xyz="0 0 -0.15"/> <material name="white"/> </visual>
    <collision> <geometry><cylinder radius="0.03" length="0.3"/></geometry> <origin xyz="0 0 -0.15"/> </collision>
    <inertial> <mass value="0.5"/> <origin xyz="0 0 -0.15"/> <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/> </inertial>
  </link>

</robot>
```

### Step 7: Update `setup.py` and Build Your Package

Ensure all your new nodes and service client/server are added to the `console_scripts` entry points in `humanoid_nervous_system/setup.py`. Then, build your workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_nervous_system
source install/setup.bash
```

### Step 8: Verify Your Nervous System

1.  **Launch the AI Commander Node**:
    ```bash
    ros2 run humanoid_nervous_system ai_commander
    ```
2.  **Monitor Joint Commands**: In a new terminal:
    ```bash
    ros2 topic echo /joint_command
    ```
    You should see the joint commands being published.
3.  **Launch the Pose Reset Service**:
    ```bash
    ros2 run humanoid_nervous_system pose_reset_service
    ```
4.  **Call the Reset Service**: In a new terminal:
    ```bash
    ros2 run humanoid_nervous_system reset_client
    ```
    (You'll need to define the service client/server as outlined in Step 5).
5.  **Visualize URDF**:
    ```bash
    ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix humanoid_nervous_system)/share/humanoid_nervous_system/urdf/simple_humanoid.urdf
    ```
    (You might need to install `urdf_tutorial` and ensure the path is correct).

## Expected Outcome

By completing this mini-project, you will have a working ROS 2 package that demonstrates:
*   Inter-node communication via topics and services.
*   A Python-based agent publishing commands.
*   A fundamental understanding of robot description using URDF.
*   The basic components of a robot's "nervous system" are ready for further development and integration with higher-level AI.

---

## Key Takeaways

*   Building a robot's "nervous system" involves integrating various ROS 2 communication patterns.
*   Topics are excellent for streaming continuous commands like joint positions.
*   Services are useful for triggering specific, one-shot actions.
*   URDF provides the necessary structural description for visualization and simulation.
*   Python (`rclpy`) is a powerful tool for developing AI agents that interact with ROS 2.

---

## Exercises

1.  **Extend URDF**: Add more links and joints (e.g., a left arm, legs) to your `simple_humanoid.urdf` model.
2.  **Conditional AI Command**: Modify the `ai_commander_node.py` to only send a joint command if a certain condition is met (e.g., after receiving a specific message on another topic, simulating sensor input).
3.  **Implement Joint State Publisher**: Create a `joint_state_publisher_node.py` that publishes dummy `JointState` messages (or your custom `JointState.msg`) to `/joint_states`. Then verify your `joint_state_monitor_node` receives them.
4.  **Action Implementation (Advanced)**: Research ROS 2 Actions. Briefly describe how you would convert your `reset_robot_pose` service into a ROS 2 Action, considering feedback and preemption.