# Chapter 2: Gazebo Physics Simulation

## Overview
This chapter focuses on Gazebo, the premier physics simulation environment for robotics. You'll learn how to create realistic physics simulations with accurate collision detection, contact forces, and material properties. We'll explore Gazebo's various physics engines, world building techniques, and integration with ROS 2. By the end of this chapter, you'll be able to create sophisticated physics-based simulations for robotic systems.

Gazebo is a powerful 3D simulation environment that accurately simulates rigid-body physics, sensor data, and environment interactions. It provides a robust platform for simulating complex robots in realistic indoor and outdoor environments, making it essential for humanoid robot development. Understanding Gazebo's physics capabilities is crucial for creating simulations that accurately represent real-world robot behavior.

## Learning Objectives
- Understand Gazebo's architecture and physics engine capabilities
- Create realistic physics simulations with proper collision and visual models
- Build complex environments with various objects and terrains
- Integrate Gazebo with ROS 2 for seamless simulation workflows
- Configure physics parameters for realistic robot-environment interactions
- Implement sensor simulation within Gazebo environments

## Key Concepts

### Gazebo Architecture
Gazebo's architecture consists of a physics engine, rendering engine, and sensor system that work together to provide realistic simulation. The modular design allows for different physics engines (ODE, Bullet, DART) and rendering backends.

### Physics Engine Parameters
Key physics parameters include gravity, time step, solver settings, and collision detection parameters. These must be carefully configured to balance accuracy and performance.

### SDF (Simulation Description Format)
SDF is Gazebo's native format for describing robots, environments, and simulation parameters. It extends URDF capabilities to include physics, sensors, and plugins specific to simulation.

### World Building
Creating realistic environments involves defining terrain, objects, lighting, and physics properties that accurately represent the intended application scenario.

## Technical Deep Dive

### Gazebo Physics Engines

Gazebo supports multiple physics engines, each with different characteristics:

**ODE (Open Dynamics Engine)**:
- Default physics engine in older Gazebo versions
- Good for general-purpose simulation
- Supports complex joint types and constraints
- Well-tested and stable

**Bullet**:
- High-performance physics engine
- Good for real-time simulation
- Supports advanced collision detection
- Used in game development

**DART (Dynamic Animation and Robotics Toolkit)**:
- Modern physics engine with advanced features
- Better handling of complex kinematic chains
- More accurate contact simulation
- Good for humanoid robots

### Physics Parameter Configuration

Critical physics parameters that affect simulation quality:

**Time Step**: The simulation step size affects accuracy and stability. Smaller steps provide better accuracy but require more computation.

**Real-time Factor**: Controls how fast simulation runs relative to real time. A factor of 1.0 means simulation runs at the same speed as real time.

**Solver Parameters**: Include error reduction parameters (ERP) and constraint force mixing (CFM) that affect constraint stability.

**Collision Detection**: Parameters that determine how contacts are detected and processed.

### SDF vs URDF

While URDF describes robot kinematics, SDF extends this to include simulation-specific elements:

- Physics properties (mass, inertia, friction)
- Sensor definitions and plugins
- Visual and collision geometries
- Environment descriptions
- Simulation plugins and controllers

### Gazebo Architecture (Text Diagram)
```
+-----------------------------------------------------------+
|                        Gazebo System                      |
|                                                           |
|  +----------------+    +----------------+    +----------+ |
|  |   Physics      |    |   Rendering    |    |   Sensor | |
|  |   Engine       |    |   Engine       |    |   System | |
|  |   (ODE/Bullet/ |    |   (OGRE/      |    |   (Ray,  | |
|  |   DART)        |    |   OpenGL)      |    |   Camera,| |
|  +----------------+    +----------------+    |   IMU)   | |
|         |                       |             +----------+ |
|         | Physics Updates       | Rendering     |          |
|         | & Constraints         | & Visualization|         |
|         v                       v             |          |
|  +----------------+    +----------------+    |          |
|  |   Collision    |    |   GUI &        |    |          |
|  |   Detection    |    |   Visualization|    |          |
|  |   & Contact    |    |   (RViz/Gazebo)|    |          |
|  +----------------+    +----------------+    |          |
|         |                       |             |          |
|         +-----------------------+-------------+----------+
|                           |                               |
|                    +------v------+                        |
|                    |   ROS 2      |                        |
|                    |   Interface  |                        |
|                    |   (Plugins)  |                        |
|                    +--------------+                        |
+-----------------------------------------------------------+
```

## Code Examples

### Basic Gazebo World with Physics Configuration
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Include standard models -->
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Robot model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Complex environment objects -->
    <model name="table">
      <pose>2 0 0.4 0 0 0</pose>
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

    <!-- Objects with different physical properties -->
    <model name="friction_ball">
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>10.0</mu>
                <mu2>10.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.001</iyy>
            <iyz>0.0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="bouncy_ball">
      <pose>1 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <surface>
            <bounce>
              <restitution_coefficient>0.8</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.001</iyy>
            <iyz>0.0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### Robot Model with Gazebo Plugins
```xml
<?xml version="1.0"?>
<robot name="gazebo_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link with physics properties -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Revolute joint with actuator -->
  <joint name="base_to_upper_arm" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <gazebo reference="upper_arm">
    <material>Gazebo/Red</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>

  <!-- Joint with transmission for control -->
  <transmission name="trans_upper_arm">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="base_to_upper_arm">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_upper_arm">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo plugins for ROS 2 integration -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/my_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- LiDAR sensor plugin -->
  <gazebo reference="base_link">
    <sensor type="ray" name="laser_scanner">
      <pose>0.2 0 0.1 0 0 0</pose>
      <visualize>true</visualize>
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
      <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>laser_scanner_frame</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor plugin -->
  <gazebo reference="base_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
        <topicName>imu</topicName>
        <bodyName>base_link</bodyName>
        <frameName>base_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### ROS 2 Node for Gazebo Interaction
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class GazeboRobotController(Node):
    """Node that interfaces with Gazebo simulation"""

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # Publishers for robot control
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/my_robot/cmd_vel', 10)

        # Publishers for joint control (if using joint state control)
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray, '/my_robot/joint_commands', 10)

        # Subscribers for sensor data
        self.scan_subscription = self.create_subscription(
            LaserScan, '/my_robot/scan', self.scan_callback, 10)

        self.imu_subscription = self.create_subscription(
            Imu, '/my_robot/imu', self.imu_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Robot state variables
        self.scan_data = None
        self.imu_data = None
        self.obstacle_detected = False

        self.get_logger().info('Gazebo robot controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data from Gazebo"""
        self.scan_data = msg
        # Check for obstacles in front of robot
        if len(msg.ranges) > 0:
            # Check the front 30 degrees
            front_ranges = msg.ranges[len(msg.ranges)//2-15:len(msg.ranges)//2+15]
            valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_distance = min(valid_ranges)
                self.obstacle_detected = min_distance < 1.0  # 1 meter threshold
                self.get_logger().debug(f'Min front distance: {min_distance:.2f}m')

    def imu_callback(self, msg):
        """Process IMU data from Gazebo"""
        self.imu_data = msg
        # Extract orientation or acceleration data as needed
        self.get_logger().debug(f'IMU orientation: ({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z})')

    def control_loop(self):
        """Main control loop for robot in simulation"""
        cmd = Twist()

        if self.obstacle_detected:
            # Stop and turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right
            self.get_logger().info('Obstacle detected - turning')
        else:
            # Move forward
            cmd.linear.x = 0.3  # 0.3 m/s forward
            cmd.angular.z = 0.0
            self.get_logger().info('Moving forward')

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd)

    def publish_joint_commands(self):
        """Publish joint position commands if needed"""
        cmd = Float64MultiArray()
        # Example: send joint positions [0.1, 0.2, 0.3] radians
        cmd.data = [0.1, 0.2, 0.3]
        self.joint_cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Gazebo controller stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Common Pitfalls
- **Physics Instability**: Incorrect physics parameters can cause simulation instability or unrealistic behavior
- **Collision Mesh Quality**: Poor collision meshes can lead to incorrect physics interactions
- **Time Step Issues**: Too large time steps can cause inaccurate physics, too small can cause performance issues
- **Plugin Configuration**: Incorrect Gazebo plugin configuration can prevent proper ROS 2 integration
- **Solver Parameters**: Poor solver configuration can lead to constraint violations and unstable simulations

## Checkpoints / Mini-Exercises
1. Create a Gazebo world with different physical properties (friction, bounce)
2. Implement a robot model with proper inertial properties and physics plugins
3. Simulate a robot navigating through a complex environment with obstacles
4. Configure different physics engines and compare their behavior
5. Integrate sensor data from Gazebo into a ROS 2 control node

## References
- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics)
- [SDF Specification](http://sdformat.org/)
- [Gazebo ROS Integration](http://gazebosim.org/tutorials/?tut=ros2_overview)
- [Physics Simulation Best Practices](http://gazebosim.org/tutorials?tut=guided_b1)
- [Robotics Simulation with Gazebo](https://arxiv.org/abs/1807.00885)