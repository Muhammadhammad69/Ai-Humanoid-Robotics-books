# Chapter 5: Hands-On Mini Projects

## Overview
This chapter provides practical, hands-on projects that integrate all the concepts covered in Module 2. Students will build complete digital twin environments combining physics simulation, high-fidelity rendering, and sensor simulation. These projects reinforce the theoretical knowledge with practical implementation, allowing students to create fully functional humanoid robot simulations that connect to ROS 2 systems. Each project builds upon the previous concepts and demonstrates real-world applications of digital twin technology in humanoid robotics.

## Learning Objectives
- Integrate Gazebo physics simulation with Unity rendering for a complete digital twin
- Build a humanoid robot environment with realistic physics and sensor simulation
- Implement ROS 2 communication between simulated and physical robot systems
- Develop comprehensive simulation scenarios that test multiple robot capabilities
- Validate simulation-to-reality transfer for humanoid robot algorithms

## Key Concepts

### Digital Twin Integration
Digital twin integration involves connecting multiple simulation environments (Gazebo for physics, Unity for rendering) with ROS 2 communication systems. This creates a comprehensive virtual environment that accurately represents both the physical properties and visual characteristics of the real robot system.

### Multi-Modal Simulation
Multi-modal simulation combines different types of data and interaction methods, including visual rendering, physics simulation, and sensor data. This approach provides a more complete and realistic simulation environment for humanoid robots.

### Simulation Validation
Simulation validation involves comparing the behavior of robots in simulation with their real-world counterparts to ensure that the digital twin accurately represents the physical system. This includes validating physics parameters, sensor characteristics, and control responses.

### Humanoid Robot Scenarios
Humanoid robot scenarios in simulation involve creating complex environments and tasks that test the robot's locomotion, manipulation, and interaction capabilities. These scenarios often include dynamic obstacles, multiple tasks, and human interaction elements.

## Technical Deep Dive

### Project 1: Complete Humanoid Robot Digital Twin

**Objective**: Create a complete digital twin of a humanoid robot with physics simulation, sensor simulation, and Unity visualization.

**Steps**:
1. Create a humanoid robot URDF model with all necessary joints and links
2. Configure Gazebo with physics parameters, sensors, and plugins
3. Set up Unity environment for high-fidelity rendering
4. Connect everything through ROS 2 communication

**Complete URDF Example** (humanoid_robot.urdf):
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <joint name="torso_to_lidar" type="fixed">
    <parent link="torso"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_camera" type="fixed">
    <parent link="torso"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_imu" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- Gazebo Plugins for Sensors -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="humanoid_lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>10.0</max>
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
    <sensor type="depth" name="humanoid_camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <cameraName>camera</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <topicName>imu</topicName>
        <bodyName>imu_link</bodyName>
        <frameName>imu_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo Physics Parameters -->
  <gazebo reference="base_link">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <gazebo reference="torso">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <gazebo reference="left_forearm">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

</robot>
```

**Gazebo World File** (humanoid_environment.world):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_environment">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add obstacles -->
    <model name="table">
      <pose>2 0 0.4 0 0 0</pose>
      <link name="link">
        <pose>0 0 0.4 0 0 0</pose>
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

    <model name="box">
      <pose>-1 1 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.4 0.4 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 1 1</ambient>
            <diffuse>0.5 0.5 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
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

### Project 2: Navigation and Obstacle Avoidance

**Objective**: Implement navigation and obstacle avoidance using the digital twin with LiDAR and camera sensors.

**ROS 2 Navigation Node**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import math

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Robot state
        self.linear_velocity = 0.5
        self.angular_velocity = 0.0
        self.safe_distance = 0.5  # meters
        self.obstacle_detected = False

        # Create publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.camera_subscription = self.create_subscription(
            Image, 'camera/rgb/image_raw', self.camera_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Navigation Node has been started.')

    def lidar_callback(self, msg):
        # Process LiDAR data to detect obstacles
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.obstacle_detected = min_distance < self.safe_distance

            if self.obstacle_detected:
                # Find the angle of the closest obstacle
                min_idx = np.argmin(valid_ranges)
                angle_increment = msg.angle_increment
                closest_angle = msg.angle_min + min_idx * angle_increment

                # Set angular velocity based on obstacle position
                if closest_angle < -0.2:
                    self.angular_velocity = 0.5  # Turn right
                elif closest_angle > 0.2:
                    self.angular_velocity = -0.5  # Turn left
                else:
                    self.angular_velocity = 0.0  # Stop rotation
            else:
                self.angular_velocity = 0.0  # No obstacle, reset rotation

    def camera_callback(self, msg):
        # Process camera image for additional obstacle detection
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example: Simple color-based obstacle detection (red objects)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([170, 100, 100])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2
            red_pixels = cv2.countNonZero(mask)

            # If significant red pixels detected, consider it an obstacle
            if red_pixels > 1000:  # threshold can be adjusted
                self.get_logger().info('Visual obstacle detected')
                self.obstacle_detected = True
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def control_loop(self):
        # Main control loop
        twist_msg = Twist()

        if self.obstacle_detected:
            # Stop forward motion, rotate to avoid obstacle
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_velocity
        else:
            # Move forward
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Launch File** (launch_humanoid_navigation.py):
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to robot description and world
    robot_description_path = os.path.join(
        get_package_share_directory('your_robot_description_package'),
        'urdf',
        'humanoid_robot.urdf'
    )

    world_path = os.path.join(
        get_package_share_directory('your_robot_description_package'),
        'worlds',
        'humanoid_environment.world'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_path}],
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn Robot into Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'humanoid_robot'],
                        output='screen')

    # Navigation Node
    navigation_node = Node(
        package='your_navigation_package',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        navigation_node,
    ])
```

### Project 3: Unity Visualization Integration

**Unity C# Script for Robot Control Visualization**:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class RobotVisualizer : MonoBehaviour
{
    [Header("Robot Links")]
    public GameObject torso;
    public GameObject head;
    public GameObject leftArm;
    public GameObject leftForearm;

    [Header("Sensor Data")]
    public GameObject lidarPoint;
    public GameObject cameraView;

    private ROSTCPConnector ros;
    private MessageSubscriber laserSubscriber;
    private MessageSubscriber imuSubscriber;
    private MessageSubscriber jointStateSubscriber;

    void Start()
    {
        ros = ROSTCPConnector.instance;

        // Subscribe to sensor topics
        ros.Subscribe<LaserScanMsg>("/scan", OnLaserScanReceived);
        ros.Subscribe<ImuMsg>("/imu", OnImuReceived);
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointStateReceived);
    }

    void OnLaserScanReceived(LaserScanMsg msg)
    {
        // Visualize LiDAR data as points
        for (int i = 0; i < Mathf.Min(msg.ranges.Length, 100); i += 10) // Sample every 10th point
        {
            if (msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min)
            {
                float angle = msg.angle_min + i * msg.angle_increment;
                Vector3 position = new Vector3(
                    Mathf.Cos(angle) * msg.ranges[i],
                    0.5f, // Height above ground
                    Mathf.Sin(angle) * msg.ranges[i]
                );

                GameObject point = GameObject.Instantiate(lidarPoint, position, Quaternion.identity);
                Destroy(point, 0.1f); // Remove after 0.1 seconds
            }
        }
    }

    void OnImuReceived(ImuMsg msg)
    {
        // Update robot orientation based on IMU data
        if (torso != null)
        {
            // Convert quaternion from IMU to Unity rotation
            torso.transform.rotation = new Quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            );
        }
    }

    void OnJointStateReceived(JointStateMsg msg)
    {
        // Update robot joint positions based on joint state
        if (head != null && leftArm != null && leftForearm != null)
        {
            for (int i = 0; i < msg.name.Length; i++)
            {
                switch (msg.name[i])
                {
                    case "torso_to_head":
                        head.transform.localRotation = Quaternion.Euler(0, 0, Mathf.Rad2Deg * (float)msg.position[i]);
                        break;
                    case "torso_to_left_shoulder":
                        leftArm.transform.localRotation = Quaternion.Euler(Mathf.Rad2Deg * (float)msg.position[i], 0, 0);
                        break;
                    case "left_shoulder_to_elbow":
                        leftForearm.transform.localRotation = Quaternion.Euler(0, Mathf.Rad2Deg * (float)msg.position[i], 0);
                        break;
                }
            }
        }
    }
}
```

**Complete Project Architecture (Text Description)**:
```
+---------------------------------------------------------------------+
|                           DIGITAL TWIN                              |
|                                                                     |
|  +----------------+    +-------------------+    +----------------+  |
|  |  Gazebo        |    |  Unity            |    |  ROS 2         |  |
|  |  (Physics &    |    |  (Rendering &    |    |  (Control &   |  |
|  |  Sensors)      |<-->|  Visualization)  |<-->|  Communication)|  |
|  +----------------+    +-------------------+    +----------------+  |
|         |                        |                        |         |
|         v                        v                        v         |
|  +----------------+    +-------------------+    +----------------+  |
|  | Humanoid Robot |    | Humanoid Robot    |    | Navigation &   |  |
|  | (Physical      |    | (Visual Model)    |    | Control Nodes  |  |
|  | Properties)    |    |                   |    |                |  |
|  +----------------+    +-------------------+    +----------------+  |
|                                                                     |
|  +---------------------------------------------------------------+  |
|  |                    Unity Scene                                |  |
|  |  +----------------+    +----------------+    +-------------+  |  |
|  |  | Environment    |    | Robot Model    |    | UI/Controls |  |  |
|  |  | (Obstacles,    |    | (Articulated   |    | (Debugging, |  |  |
|  |  | Lighting, etc.)|    | Links)         |    | Monitoring)|  |  |
|  |  +----------------+    +----------------+    +-------------+  |  |
|  +---------------------------------------------------------------+  |
+---------------------------------------------------------------------+
```
This diagram shows the complete architecture of the digital twin system with all components integrated.

## Common Pitfalls
- **Coordinate System Mismatches**: Ensure consistent coordinate systems across Gazebo, Unity, and ROS 2 (e.g., Z-up in Gazebo vs Y-up in Unity).
- **Timing and Synchronization**: Simulation timing differences can cause desynchronization between physics and visualization.
- **Resource Management**: Complex digital twins can be computationally expensive; optimize models and reduce polygon counts where possible.
- **Integration Complexity**: Connecting multiple systems (Gazebo, Unity, ROS 2) requires careful configuration and testing.

## Checkpoints / Mini-Exercises
1. Build the complete humanoid robot model with all sensors and test it in Gazebo.
2. Implement the navigation algorithm and test obstacle avoidance in the simulated environment.
3. Connect Unity visualization to your Gazebo simulation and verify that robot movements are synchronized.
4. Create a custom environment with multiple obstacles and test your robot's navigation capabilities.
5. Add a simple manipulation task (e.g., grasping a block) to your digital twin and implement the control logic.

## References
- [ROS 2 Navigation Stack Tutorials](https://navigation.ros.org/tutorials/)
- [Gazebo Simulation Best Practices](https://classic.gazebosim.org/tutorials?cat=simulation)
- [Unity Robotics Integration Guide](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/)
- [Humanoid Robot Simulation Examples](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)