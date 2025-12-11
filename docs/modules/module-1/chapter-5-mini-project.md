# Chapter 5: Mini Project - Complete ROS 2 System

## Overview
This chapter brings together all the concepts learned in Module 1 by implementing a complete ROS 2 system for a simple mobile robot. You'll create a functional robot application that includes sensor processing, control systems, and navigation capabilities. This hands-on project demonstrates how to integrate topics, services, actions, parameters, and URDF models into a cohesive robotic system that can perform meaningful tasks.

The project involves building a robot that can navigate to specified goals while avoiding obstacles using sensor data. This requires implementing multiple nodes that communicate through various ROS 2 patterns, creating a realistic simulation environment, and integrating all components into a working system.

## Learning Objectives
- Integrate all Module 1 concepts into a complete robotic system
- Design and implement multiple interconnected ROS 2 nodes
- Create a complete robot model with URDF and simulation integration
- Implement sensor processing and control algorithms in Python
- Use launch files to start complex multi-node systems
- Debug and validate a complete ROS 2 application

## Key Concepts

### System Integration
Successfully combining multiple ROS 2 nodes with different communication patterns to achieve a complex task requires careful design and proper interfaces between components.

### Robot Behavior Architecture
The project demonstrates a layered architecture with perception (sensor processing), planning (navigation), and control (motor commands) components that work together to achieve robot autonomy.

### Simulation Integration
The project includes integration with Gazebo simulation and RViz visualization to create a complete development and testing environment.

### Multi-Node Coordination
Proper coordination between multiple nodes requires understanding of message timing, state management, and error handling across the distributed system.

## Technical Deep Dive

### Project Architecture

The complete system consists of several interconnected nodes:

**Sensor Processing Node**: Processes raw sensor data (LIDAR, IMU, odometry) and publishes processed information for other nodes to use.

**Navigation Node**: Implements path planning and obstacle avoidance algorithms, publishing velocity commands to move the robot toward goals.

**Controller Node**: Interfaces with the robot's hardware (or simulation) to execute motor commands and manage robot state.

**Visualization Node**: Publishes markers and other information for display in RViz.

### Communication Pattern Integration

The system uses multiple communication patterns:
- **Topics**: Sensor data, odometry, and velocity commands using appropriate QoS settings
- **Services**: For dynamic reconfiguration and system status queries
- **Actions**: For goal-oriented navigation tasks with feedback

### System Architecture (Text Diagram)
```
+-------------------+    +-------------------+    +-------------------+
|   Sensor Node     |    |  Navigation Node  |    |  Controller Node  |
|                   |    |                   |    |                   |
| - Process LIDAR   |    | - Plan paths      |    | - Execute motor   |
| - Process odometry|    | - Avoid obstacles |    |   commands        |
| - Publish sensor  |    | - Publish cmd_vel |    | - Manage state    |
|   data            |    | - Action server   |    | - Hardware I/F    |
+--------+----------+    +--------+----------+    +----------+--------+
         |                      |                          |
         | sensor_msgs          | geometry_msgs            | geometry_msgs
         | /scan, /odom         | /cmd_vel                 | /cmd_vel
         v                      v                          v
+--------+----------------------+--------------------------+--------+
|                     ROS 2 Middleware Layer                      |
|                    (DDS Implementation)                         |
+--------+----------------------+--------------------------+--------+
         |                      |                          |
         |                      |                          |
         v                      v                          v
+-------------------+    +-------------------+    +-------------------+
|   Visualization   |    |   Gazebo Sim      |    |      RViz         |
|      Node         |    |                   |    |                   |
| - Publish markers |    | - Robot physics   |    | - Display robot   |
| - Debug info      |    | - Sensor models   |    | - Show paths      |
| - TF transforms   |    | - Environment     |    | - Sensor data     |
+-------------------+    +-------------------+    +-------------------+
```

## Code Examples

### Complete Robot URDF Model
```xml
<?xml version="1.0"?>
<robot name="project_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://project_robot/meshes/base.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- LIDAR Mount -->
  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="laser_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Gazebo integration -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="wheel_left_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <gazebo reference="laser_link">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_sensor" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Navigation Node Implementation
```python
#!/usr/bin/env python3
"""
Navigation node for the complete ROS 2 system project
Implements obstacle avoidance and goal navigation
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from example_interfaces.action import NavigateToPose
import numpy as np
import math


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Navigation state
        self.current_pose = Point()
        self.target_pose = Point()
        self.is_navigating = False
        self.min_distance = 0.5  # minimum safe distance to obstacles

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Action server for navigation
        self.navigation_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_navigation)

        # Timer for navigation control loop
        self.nav_timer = self.create_timer(0.1, self.navigation_control)

        self.get_logger().info('Navigation node initialized')

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # Extract yaw from quaternion (simplified)
        # In a real implementation, you'd properly convert quaternion to euler

    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Filter out invalid ranges
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if valid_ranges:
            self.min_distance = min(valid_ranges)

    def calculate_navigation_command(self):
        """Calculate velocity command based on target and obstacles"""
        cmd = Twist()

        if not self.is_navigating:
            return cmd

        # Calculate distance to target
        dist_to_target = math.sqrt(
            (self.target_pose.x - self.current_pose.x)**2 +
            (self.target_pose.y - self.current_pose.y)**2
        )

        # Check if close enough to target
        if dist_to_target < 0.2:
            self.is_navigating = False
            self.get_logger().info('Reached target position')
            return cmd

        # Calculate angle to target
        angle_to_target = math.atan2(
            self.target_pose.y - self.current_pose.y,
            self.target_pose.x - self.current_pose.x
        )

        # Simple obstacle avoidance
        if self.min_distance < 0.8:
            # Stop and turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right
        else:
            # Move toward target
            cmd.linear.x = min(0.5, dist_to_target)  # Scale speed with distance
            cmd.angular.z = angle_to_target * 1.0  # Proportional to angle error

        return cmd

    def navigation_control(self):
        """Main navigation control loop"""
        if not self.is_navigating:
            return

        cmd = self.calculate_navigation_command()
        self.cmd_vel_publisher.publish(cmd)

    def execute_navigation(self, goal_handle):
        """Execute navigation action"""
        self.get_logger().info(f'Executing navigation to: ({goal_handle.request.pose.position.x}, {goal_handle.request.pose.position.y})')

        # Set target pose
        self.target_pose = goal_handle.request.pose.position
        self.is_navigating = True

        # Execute navigation until target reached
        while self.is_navigating and not goal_handle.is_cancel_requested:
            # Publish feedback
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose.position = self.current_pose
            goal_handle.publish_feedback(feedback_msg)

            # Small delay to allow other callbacks to process
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Check if cancelled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.is_navigating = False
            result = NavigateToPose.Result()
            result.error_code = 1  # Cancelled
            return result

        # Navigation completed successfully
        goal_handle.succeed()
        self.is_navigating = False
        result = NavigateToPose.Result()
        result.error_code = 0  # Success
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Launch File for Complete System
```python
# project_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes for the complete system
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0
        }],
        arguments=[PathJoinSubstitution([
            FindPackageShare('project_robot'),
            'urdf',
            'project_robot.urdf'
        ])]
    )

    navigation_node = Node(
        package='project_robot',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # RViz node
    rviz_config = PathJoinSubstitution([
        FindPackageShare('project_robot'),
        'rviz',
        'project_robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('project_robot'),
                'worlds',
                'simple_room.world'
            ])
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'project_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        navigation_node,
        rviz_node
    ])
```

## Common Pitfalls
- **Timing Issues**: Different nodes running at different rates can cause coordination problems. Use appropriate QoS settings and consider message timestamps.
- **Coordinate Frame Management**: Improper TF transforms can cause navigation errors. Ensure all frames are properly defined and connected.
- **Resource Contention**: Multiple nodes accessing shared resources without proper synchronization can cause race conditions.
- **Parameter Tuning**: Navigation parameters (speed, turning rate, obstacle detection thresholds) need careful tuning for stable operation.
- **Error Handling**: Not properly handling errors in one node can cause the entire system to fail.

## Checkpoints / Mini-Exercises
1. Implement the complete robot URDF and test it in RViz
2. Create the navigation node and test obstacle avoidance in simulation
3. Add a simple path planning algorithm to the navigation node
4. Create a launch file that starts the complete system with Gazebo and RViz
5. Test the complete system by sending navigation goals and observing robot behavior
6. Add a simple UI or service to set navigation goals dynamically

## References
- [ROS 2 Navigation Stack](https://navigation.ros.org/)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)
- [Gazebo ROS Integration](http://gazebosim.org/tutorials/?tut=ros2_overview)
- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)