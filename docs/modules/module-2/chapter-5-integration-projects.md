# Chapter 5: Integration Projects

## Overview
This chapter brings together all the concepts from Module 2 into comprehensive integration projects. You'll build complete digital twin environments that combine physics simulation in Gazebo with high-fidelity rendering in Unity, all integrated with ROS 2 for a complete robotic simulation ecosystem. These projects demonstrate the practical application of simulation techniques and provide a foundation for advanced robotic development workflows.

The integration projects in this chapter showcase how to create realistic digital twins that accurately represent physical robotic systems. Through hands-on implementation, you'll learn to orchestrate complex simulation environments that combine multiple sensors, realistic physics, and immersive visualization. These projects serve as templates for creating simulation environments for your own robotic applications.

## Learning Objectives
- Integrate Gazebo physics simulation with Unity rendering for complete digital twin environments
- Implement complex robotic systems with multiple sensors and actuators
- Create realistic simulation scenarios that test multiple robot capabilities
- Develop comprehensive simulation workflows that bridge development and deployment
- Validate simulation-to-reality transfer for robotic algorithms

## Key Concepts

### Digital Twin Architecture
Digital twins are virtual replicas of physical systems that enable simulation, analysis, and optimization. In robotics, digital twins bridge the gap between simulation and reality, allowing for safe testing and validation of robotic systems before deployment.

### Multi-Environment Integration
Successful robotic simulation requires seamless integration between physics simulation, visual rendering, and control systems. This involves coordinating multiple simulation environments and ensuring consistent timing and state management.

### Sensor Fusion in Simulation
Combining data from multiple simulated sensors to create a more accurate and reliable understanding of the environment. This requires proper timing synchronization and coordinate frame alignment across different sensor types.

### Simulation Validation
Comparing simulation results with theoretical models and, when possible, real-world data to validate the accuracy of the simulation environment.

## Technical Deep Dive

### Digital Twin Architecture

A complete digital twin architecture for robotics typically includes:

**Physics Layer**: Gazebo provides realistic physics simulation with accurate collision detection, contact forces, and material properties.

**Rendering Layer**: Unity provides high-fidelity visual rendering for realistic sensor simulation and human interaction.

**Control Layer**: ROS 2 acts as the communication backbone connecting simulation components with robotic software.

**Monitoring Layer**: Visualization tools and debugging interfaces for real-time monitoring of the simulation.

### Integration Patterns

Several patterns emerge when integrating different simulation components:

**Bridge Pattern**: Use specialized bridge nodes to translate between different coordinate systems and message formats.

**Synchronization Pattern**: Implement timing synchronization mechanisms to ensure consistent state across simulation components.

**Modularity Pattern**: Design simulation components to be modular and reusable across different projects.

### Digital Twin Integration Architecture (Text Diagram)
```
+------------------------------------------------------------------+
|                    Digital Twin Architecture                     |
|                                                                  |
|  +----------------+    +----------------+    +----------------+  |
|  |  Physical      |    |  Gazebo        |    |  Unity         |  |
|  |  Robot         |<-->|  Physics       |<-->|  Rendering     |  |
|  |  (Real World)  |    |  Simulation    |    |  (Visual)      |  |
|  +----------------+    +----------------+    +----------------+  |
|         |                       |                       |        |
|         | Real-time Data        | Physics &             | Visual
|         | & Commands            | Sensor Data           | Feedback
|         v                       v                       v        |
|  +----------------+    +----------------+    +----------------+  |
|  |  ROS 2         |<-->|  Simulation    |<-->|  Human User    |  |
|  |  Communication |    |  Bridge       |    |  (VR/AR/HMI)   |  |
|  |  Layer        |    |                |    |                |  |
|  +----------------+    +----------------+    +----------------+  |
|         |                       |                       |        |
|         | ROS Messages          | Sensor Fusion         | VR/AR
|         | & Services            | & Processing          | Interface
|         v                       v                       v        |
|  +----------------+    +----------------+    +----------------+  |
|  | Robotic        |    | Perception &   |    | Training &     |  |
|  | Control        |    | AI Algorithms  |    | Visualization  |  |
|  | Algorithms     |    | (CV, ML, etc.) |    | Applications   |  |
|  +----------------+    +----------------+    +----------------+  |
+------------------------------------------------------------------+
```

## Code Examples

### Complete Robot URDF with Gazebo and Unity Integration
```xml
<?xml version="1.0"?>
<robot name="integration_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="1.5" ixy="0.0" ixz="0.0" iyy="1.5" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.3</mu1>
    <mu2>0.3</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Left wheel -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0 0.3 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="wheel_left_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Right wheel -->
  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0 -0.3 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_link">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="wheel_right_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Upper arm -->
  <joint name="arm_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm_link">
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="upper_arm_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Lower arm -->
  <joint name="arm_elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="lower_arm_link">
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="lower_arm_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <!-- Sensors -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.25 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link"/>

  <joint name="camera_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.28 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>

  <joint name="imu_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="main_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor type="camera" name="main_camera">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="main_imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
        <topic_name>imu</topic_name>
        <body_name>imu_link</body_name>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.6</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### Gazebo World with Complex Environment
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="integration_world">
    <!-- Physics engine -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Robot -->
    <include>
      <uri>model://integration_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Complex environment objects -->
    <model name="table_1">
      <pose>3 2 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="obstacle_cylinder">
      <pose>-2 -1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.5 1.0 1</ambient>
            <diffuse>0.5 0.5 1.0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Dynamic objects -->
    <model name="moving_box">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.5 0.5 1</ambient>
            <diffuse>1.0 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### ROS 2 Integration Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import math
from tf2_ros import TransformBroadcaster
import tf_transformations


class IntegrationNode(Node):
    """Node that integrates all simulation components"""

    def __init__(self):
        super().__init__('integration_node')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create subscribers for all sensor types
        self.scan_subscription = self.create_subscription(
            LaserScan, '/integration_robot/scan', self.scan_callback, 10)

        self.image_subscription = self.create_subscription(
            Image, '/integration_robot/image_raw', self.image_callback, 10)

        self.imu_subscription = self.create_subscription(
            Imu, '/integration_robot/imu', self.imu_callback, 10)

        self.odom_subscription = self.create_subscription(
            Odometry, '/integration_robot/odom', self.odom_callback, 10)

        # Create publishers for control
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/integration_robot/cmd_vel', 10)

        # Publishers for processed data
        self.merged_scan_publisher = self.create_publisher(
            LaserScan, '/integration_robot/merged_scan', 10)

        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/integration_robot/fused_pose', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state variables
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None
        self.latest_odom = None
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Control parameters
        self.safe_distance = 0.8  # meters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

        # Create timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Integration node initialized')

    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.latest_scan = msg
        self.get_logger().debug(f'Received scan with {len(msg.ranges)} ranges')

        # Process scan data for obstacle detection
        processed_scan = self.process_scan_data(msg)
        self.merged_scan_publisher.publish(processed_scan)

    def image_callback(self, msg):
        """Process camera image data"""
        self.latest_image = msg
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height}')

        try:
            # Convert and process image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed_image = self.process_camera_image(cv_image)

            # Extract features or detect objects as needed
            features = self.extract_features(processed_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.latest_imu = msg

        # Extract orientation from IMU
        orientation_q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Convert quaternion to euler angles
        euler = tf_transformations.euler_from_quaternion(orientation_q)
        roll, pitch, yaw = euler

        self.get_logger().debug(f'IMU orientation: ({roll:.3f}, {pitch:.3f}, {yaw:.3f})')

    def odom_callback(self, msg):
        """Process odometry data"""
        self.latest_odom = msg

        # Update robot pose from odometry
        pose = msg.pose.pose
        self.robot_pose[0] = pose.position.x
        self.robot_pose[1] = pose.position.y

        # Convert orientation to heading
        orientation_q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(orientation_q)
        self.robot_pose[2] = euler[2]  # yaw

        # Extract velocities
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def control_loop(self):
        """Main control loop that integrates all sensor data"""
        cmd = Twist()

        if self.latest_scan is not None:
            # Check for obstacles using LiDAR data
            obstacle_info = self.detect_obstacles(self.latest_scan)

            if obstacle_info['closest_distance'] < self.safe_distance:
                # Obstacle detected, stop and turn
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed if obstacle_info['direction'] > 0 else -self.max_angular_speed
                self.get_logger().warn(f'Obstacle detected at {obstacle_info["closest_distance"]:.2f}m, turning')
            else:
                # Clear path, move forward
                cmd.linear.x = min(self.max_linear_speed, obstacle_info['closest_distance'] * 0.5)
                cmd.angular.z = 0.0
        else:
            # No scan data, stop robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

        # Publish fused pose estimate
        self.publish_fused_pose()

    def process_scan_data(self, scan_msg):
        """Process LiDAR data to remove noise and invalid readings"""
        processed_msg = LaserScan()
        processed_msg.header = scan_msg.header
        processed_msg.angle_min = scan_msg.angle_min
        processed_msg.angle_max = scan_msg.angle_max
        processed_msg.angle_increment = scan_msg.angle_increment
        processed_msg.time_increment = scan_msg.time_increment
        processed_msg.scan_time = scan_msg.scan_time
        processed_msg.range_min = scan_msg.range_min
        processed_msg.range_max = scan_msg.range_max

        # Apply simple filtering
        filtered_ranges = []
        for r in scan_msg.ranges:
            if scan_msg.range_min <= r <= scan_msg.range_max:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))

        processed_msg.ranges = filtered_ranges
        return processed_msg

    def detect_obstacles(self, scan_msg):
        """Detect obstacles in front of robot"""
        # Analyze the front 90 degrees of the scan
        center_idx = len(scan_msg.ranges) // 2
        front_range = 45  # Number of rays to consider on each side
        front_rays = scan_msg.ranges[center_idx - front_range:center_idx + front_range]

        # Filter valid ranges
        valid_ranges = [r for r in front_rays if scan_msg.range_min <= r <= scan_msg.range_max]

        if not valid_ranges:
            return {'closest_distance': float('inf'), 'direction': 0}

        closest_distance = min(valid_ranges)
        closest_idx = front_rays.index(closest_distance) if closest_distance in front_rays else center_idx

        # Determine direction of closest obstacle (positive = right, negative = left)
        direction = (closest_idx - front_range) / front_range

        return {'closest_distance': closest_distance, 'direction': direction}

    def process_camera_image(self, cv_image):
        """Process camera image for feature extraction"""
        # Apply basic image processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        return edges

    def extract_features(self, processed_image):
        """Extract features from processed image"""
        # Example: Find contours
        contours, hierarchy = cv2.findContours(
            processed_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        features = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Calculate bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                features.append({'x': x, 'y': y, 'width': w, 'height': h})

        return features

    def publish_fused_pose(self):
        """Publish fused pose estimate combining multiple sensors"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Set position from odometry
        pose_msg.pose.pose.position.x = self.robot_pose[0]
        pose_msg.pose.pose.position.y = self.robot_pose[1]
        pose_msg.pose.pose.position.z = 0.0

        # Set orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.robot_pose[2])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Set covariance (diagonal elements for position and orientation)
        pose_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # Position x
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # Position y
                                    0.0, 0.0, 999999, 0.0, 0.0, 0.0,  # Position z
                                    0.0, 0.0, 0.0, 999999, 0.0, 0.0,  # Rotation x
                                    0.0, 0.0, 0.0, 0.0, 999999, 0.0,  # Rotation y
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]  # Rotation z

        self.fused_pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Integration node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Unity Integration Script
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System.Collections.Generic;

public class UnityIntegrationController : MonoBehaviour
{
    [Header("Robot Components")]
    public GameObject baseLink;
    public GameObject leftWheel;
    public GameObject rightWheel;
    public GameObject upperArm;
    public GameObject lowerArm;
    public GameObject lidarSensor;
    public GameObject cameraSensor;

    [Header("ROS Settings")]
    public string robotNamespace = "/integration_robot";
    public float publishRate = 50.0f;  // Hz

    private ROSTCPConnector ros;
    private float lastPublishTime = 0.0f;

    // ROS message publishers and subscribers
    private MessageSubscriber<OdometryMsg> odomSubscriber;
    private MessageSubscriber<TwistMsg> cmdVelSubscriber;
    private Publisher<JointStateMsg> jointStatePublisher;
    private Publisher<OdomMsg> odomPublisher;

    void Start()
    {
        ros = ROSTCPConnector.instance;

        // Create publishers
        jointStatePublisher = ros.AcquirePublisher<JointStateMsg>($"{robotNamespace}/unity_joint_states");
        odomPublisher = ros.AcquirePublisher<OdomMsg>($"{robotNamespace}/unity_odom");

        // Subscribe to robot commands
        cmdVelSubscriber = ros.Subscribe<TwistMsg>($"{robotNamespace}/cmd_vel", OnVelocityCommandReceived);
        odomSubscriber = ros.Subscribe<OdometryMsg>($"{robotNamespace}/odom", OnOdomReceived);

        Debug.Log($"Unity integration controller initialized for namespace: {robotNamespace}");
    }

    void Update()
    {
        // Publish joint states at specified rate
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishJointStates();
            PublishOdom();
            lastPublishTime = Time.time;
        }

        // Update visualization based on current robot state
        UpdateRobotVisualization();
    }

    void OnVelocityCommandReceived(TwistMsg cmd)
    {
        // Update robot visualization based on velocity commands
        // This simulates how the robot would move in response to commands
        float linearX = (float)cmd.linear.x;
        float angularZ = (float)cmd.angular.z;

        // Simulate wheel rotation based on linear and angular velocity
        if (leftWheel != null && rightWheel != null)
        {
            // Calculate wheel rotations based on differential drive kinematics
            float leftSpeed = linearX - angularZ * 0.3f; // 0.3m wheel separation
            float rightSpeed = linearX + angularZ * 0.3f;

            leftWheel.transform.Rotate(Vector3.right, leftSpeed * 100 * Time.deltaTime);
            rightWheel.transform.Rotate(Vector3.right, rightSpeed * 100 * Time.deltaTime);
        }

        // Update base link position and rotation
        if (baseLink != null)
        {
            baseLink.transform.Translate(Vector3.forward * linearX * Time.deltaTime);
            baseLink.transform.Rotate(Vector3.up, angularZ * Mathf.Rad2Deg * Time.deltaTime);
        }
    }

    void OnOdomReceived(OdometryMsg odomMsg)
    {
        // Update robot position and orientation based on odometry
        if (baseLink != null)
        {
            baseLink.transform.position = new Vector3(
                (float)odomMsg.pose.pose.position.x,
                (float)odomMsg.pose.pose.position.y,
                (float)odomMsg.pose.pose.position.z
            );

            baseLink.transform.rotation = new Quaternion(
                (float)odomMsg.pose.pose.orientation.x,
                (float)odomMsg.pose.pose.orientation.y,
                (float)odomMsg.pose.pose.orientation.z,
                (float)odomMsg.pose.pose.orientation.w
            );
        }
    }

    void UpdateRobotVisualization()
    {
        // Update all robot components based on current state
        if (upperArm != null)
        {
            // Example: animate arm movement
            upperArm.transform.Rotate(Vector3.up, Mathf.Sin(Time.time) * 10 * Time.deltaTime, Space.Self);
        }

        if (lowerArm != null)
        {
            // Example: animate lower arm movement
            lowerArm.transform.Rotate(Vector3.forward, Mathf.Cos(Time.time) * 15 * Time.deltaTime, Space.Self);
        }
    }

    void PublishJointStates()
    {
        // Create and publish joint state message
        JointStateMsg jointStateMsg = new JointStateMsg();
        jointStateMsg.header = new HeaderMsg();
        jointStateMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        jointStateMsg.header.frame_id = "base_link";

        // Populate joint names
        jointStateMsg.name = new List<string> {
            "wheel_left_joint",
            "wheel_right_joint",
            "arm_shoulder_joint",
            "arm_elbow_joint"
        };

        // Calculate joint positions from Unity transforms
        jointStateMsg.position = new List<double> {
            leftWheel != null ? GetNormalizedWheelRotation(leftWheel) : 0.0,
            rightWheel != null ? GetNormalizedWheelRotation(rightWheel) : 0.0,
            upperArm != null ? upperArm.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad : 0.0,
            lowerArm != null ? lowerArm.transform.localRotation.eulerAngles.z * Mathf.Deg2Rad : 0.0
        };

        // Populate velocities and efforts if available
        jointStateMsg.velocity = new List<double> { 0.0, 0.0, 0.0, 0.0 };
        jointStateMsg.effort = new List<double> { 0.0, 0.0, 0.0, 0.0 };

        jointStatePublisher.Publish(jointStateMsg);
    }

    void PublishOdom()
    {
        // Publish odometry based on Unity transforms
        OdomMsg odomMsg = new OdomMsg();
        odomMsg.header = new HeaderMsg();
        odomMsg.header.stamp = new TimeStamp(ros.Clock.NowSec, ros.Clock.NowNsec);
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_link";

        // Set position
        if (baseLink != null)
        {
            odomMsg.pose.pose.position = new PointMsg(
                baseLink.transform.position.x,
                baseLink.transform.position.y,
                baseLink.transform.position.z
            );

            odomMsg.pose.pose.orientation = new QuaternionMsg(
                baseLink.transform.rotation.x,
                baseLink.transform.rotation.y,
                baseLink.transform.rotation.z,
                baseLink.transform.rotation.w
            );
        }

        // In a real implementation, you would calculate velocities
        // based on position changes over time
        odomMsg.twist.twist.linear = new Vector3Msg(0.0, 0.0, 0.0);
        odomMsg.twist.twist.angular = new Vector3Msg(0.0, 0.0, 0.0);

        odomPublisher.Publish(odomMsg);
    }

    double GetNormalizedWheelRotation(GameObject wheel)
    {
        // Normalize wheel rotation to a reasonable range
        if (wheel == null) return 0.0;

        // Assuming the wheel rotates around X axis
        float rotation = wheel.transform.localRotation.eulerAngles.x;

        // Convert to radians and normalize
        return (rotation % 360) * Mathf.Deg2Rad;
    }
}
```

## Common Pitfalls
- **Synchronization Issues**: Different update rates between Gazebo, Unity, and ROS 2 can cause desynchronization
- **Coordinate System Mismatches**: Inconsistent coordinate systems between different simulation components
- **Resource Management**: Complex simulations can consume significant computational resources
- **Integration Complexity**: Connecting multiple simulation tools requires careful configuration and testing
- **Timing Problems**: Message delays between components can affect real-time performance

## Checkpoints / Mini-Exercises
1. Build the complete robot model with all sensors and test it in Gazebo
2. Implement the Unity visualization and verify synchronization with Gazebo
3. Create a complex environment with multiple obstacles and dynamic objects
4. Integrate sensor data fusion in a ROS 2 node for navigation
5. Validate the complete digital twin by testing navigation in the integrated environment

## References
- [ROS 2 Integration Tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulation.html)
- [Gazebo-Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/gazebo_integration.md)
- [Digital Twin Architecture Patterns](https://ieeexplore.ieee.org/document/9143508)
- [Multi-Physics Simulation in Robotics](https://www.sciencedirect.com/science/article/pii/S0921889020303567)
- [Sensor Fusion in Simulated Environments](https://arxiv.org/abs/2007.13099)