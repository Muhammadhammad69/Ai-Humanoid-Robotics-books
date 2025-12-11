# Chapter 6: Advanced Simulation Integration

## Overview
This capstone chapter integrates all concepts from Module 2 into comprehensive, production-ready simulation environments. You'll build complete digital twin systems that combine advanced physics simulation, high-fidelity rendering, realistic sensor modeling, and sophisticated multi-robot coordination. This chapter demonstrates how to create simulation environments that are indistinguishable from reality, enabling safe and efficient development of complex humanoid robotic systems. The projects in this chapter represent the pinnacle of simulation capabilities covered throughout the module.

The advanced integration projects in this chapter showcase the complete digital twin concept by combining Gazebo's physics simulation with Unity's rendering capabilities, all orchestrated through ROS 2 communication. These projects demonstrate real-world applications of digital twin technology for humanoid robotics, including manufacturing, healthcare, and service robotics scenarios.

## Learning Objectives
- Design and implement complete digital twin ecosystems for humanoid robotics
- Integrate advanced physics simulation with photorealistic rendering
- Create complex multi-robot coordination scenarios in simulation
- Implement realistic sensor fusion and perception systems
- Validate simulation-to-reality transfer for complex humanoid behaviors
- Optimize simulation performance for real-time applications
- Deploy complete simulation environments for algorithm development and testing

## Key Concepts

### Advanced Digital Twin Architecture
Advanced digital twins integrate multiple simulation layers including physics, rendering, sensor simulation, and AI perception systems. These systems must operate in real-time while maintaining accuracy and responsiveness for effective algorithm development.

### Multi-Physics Simulation
Modern robotics applications require simulation of multiple physical phenomena including rigid body dynamics, soft body physics, fluid dynamics, and electromagnetic interactions. Understanding how to combine these different physics models is crucial for realistic humanoid simulation.

### Realistic Sensor Fusion
Combining data from multiple simulated sensors to create a comprehensive understanding of the environment. This requires proper timing synchronization, coordinate frame alignment, and uncertainty modeling to match real-world sensor fusion challenges.

### Large-Scale Environment Simulation
Creating complex environments with thousands of objects, realistic lighting, and dynamic elements that provide diverse testing scenarios for humanoid robots operating in real-world settings.

## Technical Deep Dive

### Complete Digital Twin Implementation

A production-ready digital twin system includes:

**Physics Layer**: Gazebo provides accurate physics simulation with realistic collision detection, contact forces, and material properties. This layer handles all robot-environment interactions including walking dynamics, manipulation physics, and contact mechanics.

**Rendering Layer**: Unity provides high-fidelity visual rendering for realistic sensor simulation and human interaction. This includes realistic lighting, shadows, reflections, and material properties that match real-world conditions.

**Sensor Layer**: Comprehensive simulation of all robot sensors including LiDAR, RGB-D cameras, IMUs, force-torque sensors, and tactile sensors with realistic noise models and characteristics.

**Control Layer**: Real-time control systems that operate identically in simulation and reality, enabling seamless transfer of algorithms between environments.

### Multi-Robot Coordination in Digital Twins

Advanced digital twins support multiple robots working together with realistic communication constraints:

**Communication Modeling**: Simulating real-world network delays, bandwidth limitations, and packet loss that affect multi-robot coordination.

**Task Allocation**: Implementing distributed algorithms for assigning tasks to robots based on capabilities, location, and current workload.

**Collision Avoidance**: Advanced path planning algorithms that account for multiple moving robots in shared spaces.

### Advanced Simulation Architecture (Text Diagram)
```
+---------------------------------------------------------------------------+
|                        ADVANCED DIGITAL TWIN SYSTEM                       |
|                                                                           |
|  +-------------------+    +--------------------+    +------------------+ |
|  |   Physical World  |    |   Gazebo Physics   |    |   Unity Visual   | |
|  |   (Real Robots)   |<-->|   Simulation       |<-->|   Rendering      | |
|  |   & Environments  |    |   (ODE/Bullet/    |    |   (PBR, Lighting,| |
|  +-------------------+    |   DART Engines)    |    |   Post-Processing)| |
|         |                 +--------------------+    +------------------+ |
|         | Real Data       |                    |           |              |
|         | & Commands      |  +----------------+ |           | Visual      |
|         |                 |  |   Sensor       | |           | Feedback    |
|         v                 |  |   Simulation   | |           | & HMI       |
|  +-------------------+    |  | (LiDAR, Cam,   | |           |             |
|  |   ROS 2           |<----->|  IMU, etc.)    |<----------->|             |
|  |   Communication   |    |  +----------------+ |           |             |
|  |   Layer           |    |         |           |           |             |
|  +-------------------+    |         | Sensor &  |           |             |
|         |                 |         | Physics   |           |             |
|         v                 |         | Data      |           |             |
|  +-------------------+    |         v           |           |             |
|  |  Perception &     |    |  +----------------+ |           |             |
|  |  AI Algorithms   |<----->|  Simulation      | |           |             |
|  |  (CV, ML, etc.)  |    |  |  Bridge/       | |           |             |
|  |                  |    |  |  Orchestrator   | |           |             |
|  +-------------------+    |  +----------------+ |           |             |
|                           |         |           |           |             |
|                           |         | Unified   |           |             |
|                           |         | Control   |           |             |
|                           |         | & State   |           |             |
|                           |         | Management|           |             |
|                           |         v           |           |             |
|  +-------------------+    |  +----------------+ |           |             |
|  |  Human Operators  |    |  |  Multi-Robot | |           |             |
|  |  (VR/AR, HMI)    |<----->|  Coordination  | |<----------+-------------+ |
|  |  & Supervisors    |    |  |  System       | |                         |
|  +-------------------+    |  +----------------+ |                         |
|         |                 |         |           |                         |
|         | Teleoperation   |         | Coordination|                       |
|         | & Monitoring    |         | Commands  |                       |
|         v                 |         v           |                         |
|  +-------------------+    |  +----------------+ |                         |
|  |  Training &       |    |  |  Simulation    | |                         |
|  |  Analytics        |<----->|  Optimization  | |                         |
|  |  Platform         |    |  |  Engine       | |                         |
|  +-------------------+    |  +----------------+ |                         |
+---------------------------------------------------------------------------+
```

## Code Examples

### Complete Multi-Robot Digital Twin System
```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid_digital_twin" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro for humanoid robot with complete sensor suite -->
  <xacro:macro name="humanoid_robot" params="name prefix:=^">
    <link name="${prefix}base_link">
      <inertial>
        <mass value="50.0"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <inertia ixx="5.0" ixy="0.0" ixz="0.0" iyy="5.0" iyz="0.0" izz="3.0"/>
      </inertial>

      <visual>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.2" length="1.0"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <geometry>
          <capsule radius="0.2" length="1.0"/>
        </geometry>
      </collision>
    </link>

    <!-- Complete humanoid skeleton -->
    <joint name="${prefix}base_to_torso" type="fixed">
      <parent link="${prefix}base_link"/>
      <child link="${prefix}torso"/>
      <origin xyz="0 0 0.6" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}torso">
      <inertial>
        <mass value="10.0"/>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <inertia ixx="0.8" ixy="0.0" ixz="0.0" iyy="0.8" iyz="0.0" izz="0.4"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.15" length="0.4"/>
        </geometry>
        <material name="blue">
          <color rgba="0.2 0.2 1.0 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.15" length="0.4"/>
        </geometry>
      </collision>
    </link>

    <!-- Head with sensors -->
    <joint name="${prefix}torso_to_head" type="revolute">
      <parent link="${prefix}torso"/>
      <child link="${prefix}head"/>
      <origin xyz="0 0 0.35" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}head">
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.15"/>
        </geometry>
        <material name="white">
          <color rgba="1.0 1.0 1.0 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
          <sphere radius="0.15"/>
        </geometry>
      </collision>
    </link>

    <!-- Camera in head -->
    <joint name="${prefix}head_to_camera" type="fixed">
      <parent link="${prefix}head"/>
      <child link="${prefix}camera_link"/>
      <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}camera_link">
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Two-arm structure -->
    <joint name="${prefix}torso_to_right_shoulder" type="revolute">
      <parent link="${prefix}torso"/>
      <child link="${prefix}right_upper_arm"/>
      <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}right_upper_arm">
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.02"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.2"/>
        </geometry>
        <material name="red">
          <color rgba="1.0 0.2 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.2"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}right_shoulder_to_elbow" type="revolute">
      <parent link="${prefix}right_upper_arm"/>
      <child link="${prefix}right_lower_arm"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}right_lower_arm">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="0.2"/>
        </geometry>
        <material name="red">
          <color rgba="1.0 0.2 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="0.2"/>
        </geometry>
      </collision>
    </link>

    <!-- Left arm (symmetric) -->
    <joint name="${prefix}torso_to_left_shoulder" type="revolute">
      <parent link="${prefix}torso"/>
      <child link="${prefix}left_upper_arm"/>
      <origin xyz="0.15 -0.1 0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}left_upper_arm">
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.02"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.2"/>
        </geometry>
        <material name="red">
          <color rgba="1.0 0.2 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.2"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}left_shoulder_to_elbow" type="revolute">
      <parent link="${prefix}left_upper_arm"/>
      <child link="${prefix}left_lower_arm"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}left_lower_arm">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="0.2"/>
        </geometry>
        <material name="red">
          <color rgba="1.0 0.2 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="0.2"/>
        </geometry>
      </collision>
    </link>

    <!-- Two-legged structure -->
    <joint name="${prefix}base_to_right_hip" type="revolute">
      <parent link="${prefix}base_link"/>
      <child link="${prefix}right_thigh"/>
      <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.5" upper="0.5" effort="200.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}right_thigh">
      <inertial>
        <mass value="5.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.08" length="0.4"/>
        </geometry>
        <material name="green">
          <color rgba="0.2 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.08" length="0.4"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}right_hip_to_knee" type="revolute">
      <parent link="${prefix}right_thigh"/>
      <child link="${prefix}right_shin"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="1.57" effort="200.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}right_shin">
      <inertial>
        <mass value="3.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.07" length="0.4"/>
        </geometry>
        <material name="green">
          <color rgba="0.2 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.07" length="0.4"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}right_knee_to_foot" type="revolute">
      <parent link="${prefix}right_shin"/>
      <child link="${prefix}right_foot"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-0.5" upper="0.5" effort="100.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}right_foot">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
      </inertial>
      <visual>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.3 0.15 0.1"/>
        </geometry>
        <material name="yellow">
          <color rgba="1.0 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.3 0.15 0.1"/>
        </geometry>
      </collision>
    </link>

    <!-- Left leg (symmetric) -->
    <joint name="${prefix}base_to_left_hip" type="revolute">
      <parent link="${prefix}base_link"/>
      <child link="${prefix}left_thigh"/>
      <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.5" upper="0.5" effort="200.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}left_thigh">
      <inertial>
        <mass value="5.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.08" length="0.4"/>
        </geometry>
        <material name="green">
          <color rgba="0.2 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.08" length="0.4"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}left_hip_to_knee" type="revolute">
      <parent link="${prefix}left_thigh"/>
      <child link="${prefix}left_shin"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="1.57" effort="200.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}left_shin">
      <inertial>
        <mass value="3.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.07" length="0.4"/>
        </geometry>
        <material name="green">
          <color rgba="0.2 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.07" length="0.4"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}left_knee_to_foot" type="revolute">
      <parent link="${prefix}left_shin"/>
      <child link="${prefix}left_foot"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-0.5" upper="0.5" effort="100.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}left_foot">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
      </inertial>
      <visual>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.3 0.15 0.1"/>
        </geometry>
        <material name="yellow">
          <color rgba="1.0 1.0 0.2 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0.1 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <box size="0.3 0.15 0.1"/>
        </geometry>
      </collision>
    </link>

    <!-- Gazebo plugins for the robot -->
    <gazebo reference="${prefix}base_link">
      <material>Gazebo/Orange</material>
      <mu1>0.2</mu1>
      <mu2>0.2</mu2>
    </gazebo>

    <gazebo reference="${prefix}torso">
      <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="${prefix}head">
      <material>Gazebo/White</material>
    </gazebo>

    <!-- LiDAR sensor on top of head -->
    <joint name="${prefix}head_to_lidar" type="fixed">
      <parent link="${prefix}head"/>
      <child link="${prefix}lidar_link"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}lidar_link">
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <gazebo reference="${prefix}lidar_link">
      <sensor type="ray" name="${name}_lidar_sensor">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
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
        <plugin name="${name}_gazebo_ros_lidar" filename="libgazebo_ros_laser.so">
          <topic_name>scan</topic_name>
          <frame_name>${prefix}lidar_link</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <!-- IMU sensor in torso -->
    <gazebo reference="${prefix}torso">
      <sensor type="imu" name="${name}_imu_sensor">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <plugin filename="libgazebo_ros_imu.so" name="${name}_gazebo_ros_imu">
          <topicName>imu</topicName>
          <bodyName>${prefix}torso</bodyName>
          <frameName>${prefix}torso</frameName>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Camera sensor -->
    <gazebo reference="${prefix}camera_link">
      <sensor type="camera" name="${name}_camera_sensor">
        <update_rate>30.0</update_rate>
        <camera name="head">
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
        <plugin name="${name}_camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>30.0</updateRate>
          <cameraName>${name}/camera</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>${prefix}camera_link</frameName>
        </plugin>
      </sensor>
    </gazebo>

    <!-- Transmission for joints -->
    <transmission name="${prefix}right_shoulder_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}torso_to_right_shoulder">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}right_shoulder_motor">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>

    <transmission name="${prefix}left_shoulder_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}torso_to_left_shoulder">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}left_shoulder_motor">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>

    <!-- Additional transmissions for other joints -->
    <xacro:macro name="joint_transmission" params="joint_name prefix">
      <transmission name="${prefix}${joint_name}_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="${prefix}${joint_name}">
          <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="${prefix}${joint_name}_motor">
          <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
          <mechanicalReduction>1</mechanicalReduction>
        </actuator>
      </transmission>
    </xacro:macro>

    <xacro:joint_transmission joint_name="right_shoulder_to_elbow" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="left_shoulder_to_elbow" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="torso_to_head" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="right_hip_to_knee" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="left_hip_to_knee" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="right_knee_to_foot" prefix="${prefix}"/>
    <xacro:joint_transmission joint_name="left_knee_to_foot" prefix="${prefix}"/>

  </xacro:macro>

  <!-- Instantiate multiple humanoid robots -->
  <xacro:humanoid_robot name="robot1" prefix="robot1/"/>
  <xacro:humanoid_robot name="robot2" prefix="robot2/"/>
  <xacro:humanoid_robot name="robot3" prefix="robot3/"/>

</robot>
```

### Advanced Multi-Robot Coordination Node
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import math
from enum import Enum
from typing import List, Tuple, Optional
import heapq


class RobotRole(Enum):
    LEADER = "leader"
    FOLLOWER = "follower"
    EXPLORER = "explorer"
    SUPPORT = "support"


class MultiRobotCoordinator(Node):
    """Advanced multi-robot coordination system with role-based behaviors"""

    def __init__(self):
        super().__init__('multi_robot_coordinator')

        # Robot states and roles
        self.robot_states = {}
        self.robot_roles = {}
        self.formation_positions = {}  # Desired positions in formation
        self.global_map = None
        self.assigned_tasks = {}

        # Communication ranges
        self.communication_radius = 10.0  # meters
        self.formation_distance = 2.0     # meters between robots in formation

        # Create subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.timer = self.create_timer(0.5, self.coordination_loop)

        # TF buffer for pose estimation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Multi-robot coordinator initialized')

    def map_callback(self, msg):
        """Store global map for path planning"""
        self.global_map = msg

    def register_robot(self, robot_id: str, role: RobotRole, initial_pose: PoseStamped):
        """Register a new robot with specific role"""
        self.robot_states[robot_id] = {
            'pose': initial_pose,
            'status': 'idle',
            'battery': 100.0,
            'last_seen': self.get_clock().now(),
            'neighbors': []
        }
        self.robot_roles[robot_id] = role

        # Calculate formation position based on role
        self.calculate_formation_position(robot_id, role)

        self.get_logger().info(f'Registered robot {robot_id} as {role.value}')

    def calculate_formation_position(self, robot_id: str, role: RobotRole):
        """Calculate desired formation position based on role"""
        leader_id = self.get_leader_id()
        if not leader_id or robot_id == leader_id:
            self.formation_positions[robot_id] = (0.0, 0.0)  # Leader at origin
            return

        # Define formation pattern based on role
        role_offsets = {
            RobotRole.FOLLOWER: (0.0, -self.formation_distance),
            RobotRole.EXPLORER: (self.formation_distance, 0.0),
            RobotRole.SUPPORT: (-self.formation_distance, 0.0)
        }

        leader_pos = self.robot_states[leader_id]['pose'].pose.position
        offset = role_offsets.get(role, (0.0, -self.formation_distance))
        self.formation_positions[robot_id] = (
            leader_pos.x + offset[0],
            leader_pos.y + offset[1]
        )

    def get_leader_id(self) -> Optional[str]:
        """Get the ID of the current leader"""
        for robot_id, role in self.robot_roles.items():
            if role == RobotRole.LEADER:
                return robot_id
        return None

    def update_neighbors(self):
        """Update neighbor lists based on communication range"""
        for robot_id, state in self.robot_states.items():
            current_pos = state['pose'].pose.position
            neighbors = []

            for other_id, other_state in self.robot_states.items():
                if other_id != robot_id:
                    other_pos = other_state['pose'].pose.position
                    distance = math.sqrt(
                        (current_pos.x - other_pos.x)**2 +
                        (current_pos.y - other_pos.y)**2
                    )
                    if distance <= self.communication_radius:
                        neighbors.append(other_id)

            self.robot_states[robot_id]['neighbors'] = neighbors

    def coordination_loop(self):
        """Main coordination loop"""
        # Update neighbor relationships
        self.update_neighbors()

        # Update formation positions
        leader_id = self.get_leader_id()
        if leader_id:
            self.update_formation()

        # Process tasks and assign to robots
        self.process_task_allocation()

        # Monitor robot statuses and health
        self.monitor_robot_health()

    def update_formation(self):
        """Update formation positions based on leader movement"""
        leader_id = self.get_leader_id()
        if not leader_id or leader_id not in self.robot_states:
            return

        leader_pose = self.robot_states[leader_id]['pose']
        leader_pos = leader_pose.pose.position

        for robot_id, role in self.robot_roles.items():
            if robot_id != leader_id and robot_id in self.robot_states:
                # Calculate desired position relative to leader
                desired_offset = self.formation_positions[robot_id]
                desired_pos = Point(
                    x=leader_pos.x + desired_offset[0],
                    y=leader_pos.y + desired_offset[1],
                    z=leader_pos.z
                )

                # Generate navigation command to move to desired position
                cmd = self.generate_navigation_command(
                    self.robot_states[robot_id]['pose'].pose.position,
                    desired_pos
                )

                # Publish command to robot
                self.publish_navigation_command(robot_id, cmd)

    def generate_navigation_command(self, current_pos: Point, target_pos: Point) -> Twist:
        """Generate navigation command to move towards target position"""
        cmd = Twist()

        # Calculate direction and distance to target
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance = math.sqrt(dx**2 + dy**2)

        # Threshold for reaching target
        if distance > 0.5:  # 50 cm threshold
            # Calculate desired heading
            desired_yaw = math.atan2(dy, dx)

            # Current heading (simplified - in practice, get from pose orientation)
            current_yaw = 0.0  # This would come from the robot's orientation

            # Simple proportional controller for heading
            heading_error = desired_yaw - current_yaw
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * heading_error))

            # Move forward with speed proportional to distance
            cmd.linear.x = max(0.0, min(0.5, 0.5 * distance))

        return cmd

    def process_task_allocation(self):
        """Allocate tasks to robots based on roles and capabilities"""
        # Example: Assign exploration tasks to explorers
        for robot_id, role in self.robot_roles.items():
            if role == RobotRole.EXPLORER and self.robot_states[robot_id]['status'] == 'idle':
                # Find unexplored areas in map
                exploration_target = self.find_exploration_target()
                if exploration_target:
                    self.assign_task(robot_id, 'explore', exploration_target)

    def find_exploration_target(self) -> Optional[Tuple[float, float]]:
        """Find a suitable exploration target in the map"""
        if not self.global_map:
            return None

        # Simplified exploration target finding
        # In practice, this would use frontier-based exploration algorithms
        map_width = self.global_map.info.width
        map_height = self.global_map.info.height
        resolution = self.global_map.info.resolution

        # Look for unexplored cells near explored areas
        for y in range(0, map_height, 10):  # Sample every 10 pixels
            for x in range(0, map_width, 10):
                idx = y * map_width + x
                if 0 <= idx < len(self.global_map.data):
                    cell_value = self.global_map.data[idx]
                    if cell_value == -1:  # Unknown space
                        # Convert map coordinates to world coordinates
                        world_x = self.global_map.info.origin.position.x + x * resolution
                        world_y = self.global_map.info.origin.position.y + y * resolution
                        return (world_x, world_y)

        return None

    def assign_task(self, robot_id: str, task_type: str, target: tuple):
        """Assign a task to a robot"""
        task_msg = String()
        task_msg.data = f"{task_type}:{target[0]:.2f},{target[1]:.2f}"

        # Publish task assignment to robot's task topic
        task_publisher = self.create_publisher(String, f'/{robot_id}/task_assignment', 10)
        task_publisher.publish(task_msg)

        self.assigned_tasks[robot_id] = {
            'type': task_type,
            'target': target,
            'status': 'assigned'
        }

        self.get_logger().info(f'Assigned {task_type} task to {robot_id} at {target}')

    def monitor_robot_health(self):
        """Monitor robot health and handle failures"""
        current_time = self.get_clock().now()

        for robot_id, state in self.robot_states.items():
            time_since_last_seen = (current_time - state['last_seen']).nanoseconds / 1e9

            # Check if robot is unreachable
            if time_since_last_seen > 10.0:  # 10 seconds
                self.get_logger().warn(f'Robot {robot_id} unreachable for {time_since_last_seen:.1f}s')

                # Adjust formation if necessary
                if self.robot_roles.get(robot_id) == RobotRole.LEADER:
                    self.elect_new_leader(robot_id)

    def elect_new_leader(self, old_leader_id: str):
        """Elect a new leader when current leader is unreachable"""
        # Simple election: promote first follower to leader
        for robot_id, role in self.robot_roles.items():
            if role == RobotRole.FOLLOWER and robot_id in self.robot_states:
                self.robot_roles[robot_id] = RobotRole.LEADER
                self.robot_roles[old_leader_id] = RobotRole.FOLLOWER
                self.get_logger().info(f'Robot {robot_id} promoted to leader after {old_leader_id} failure')
                break

    def publish_navigation_command(self, robot_id: str, cmd: Twist):
        """Publish navigation command to specific robot"""
        cmd_publisher = self.create_publisher(
            Twist, f'/{robot_id}/cmd_vel', 10)
        cmd_publisher.publish(cmd)


class AdvancedPathPlanner(Node):
    """Advanced path planning with collision avoidance for multi-robot systems"""

    def __init__(self):
        super().__init__('advanced_path_planner')

        # Subscribe to all robot positions and map
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.global_plan_publisher = self.create_publisher(Path, 'global_plan', 10)
        self.local_plan_publisher = self.create_publisher(Path, 'local_plan', 10)

        self.robots = {}
        self.static_obstacles = []

    def map_callback(self, msg):
        """Process map data for path planning"""
        self.map_info = msg.info
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution

    def add_robot(self, robot_id: str, pose: PoseStamped):
        """Add robot to planning system"""
        self.robots[robot_id] = {
            'pose': pose,
            'path': [],
            'status': 'planning'
        }

    def plan_multi_robot_path(self, start_poses: dict, goal_poses: dict) -> dict:
        """
        Plan paths for multiple robots considering mutual collisions
        Returns a dictionary of paths for each robot
        """
        paths = {}

        # Use Conflict-Based Search (CBS) or similar multi-robot path planning algorithm
        for robot_id in start_poses.keys():
            if robot_id in goal_poses:
                path = self.plan_single_path_with_constraints(
                    start_poses[robot_id],
                    goal_poses[robot_id],
                    [paths[rid] for rid in paths.keys()]  # Previous paths as constraints
                )
                paths[robot_id] = path

        return paths

    def plan_single_path_with_constraints(self, start: Point, goal: Point,
                                        other_paths: List[List[Point]]) -> List[Point]:
        """Plan path for single robot considering other robots' paths"""
        # Implement A* with time-based constraints
        # This is a simplified version - full implementation would be more complex
        path = self.a_star_path(start, goal)

        # Check for conflicts with other paths and resolve if needed
        for other_path in other_paths:
            conflict = self.check_path_conflict(path, other_path)
            if conflict:
                # Implement conflict resolution (delay, detour, etc.)
                path = self.resolve_path_conflict(path, other_path, conflict)

        return path

    def a_star_path(self, start: Point, goal: Point) -> List[Point]:
        """A* pathfinding algorithm implementation"""
        # Convert world coordinates to map indices
        start_idx = self.world_to_map_index(start)
        goal_idx = self.world_to_map_index(goal)

        # Implementation of A* algorithm
        open_set = [(0, start_idx)]
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: self.heuristic(start_idx, goal_idx)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_idx:
                # Reconstruct path
                path = [self.map_index_to_world(goal_idx)]
                while current in came_from:
                    current = came_from[current]
                    path.append(self.map_index_to_world(current))
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_idx)

                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def world_to_map_index(self, point: Point) -> Tuple[int, int]:
        """Convert world coordinates to map index"""
        x_idx = int((point.x - self.map_info.origin.position.x) / self.map_resolution)
        y_idx = int((point.y - self.map_info.origin.position.y) / self.map_resolution)
        return (x_idx, y_idx)

    def map_index_to_world(self, index: Tuple[int, int]) -> Point:
        """Convert map index to world coordinates"""
        point = Point()
        point.x = self.map_info.origin.position.x + index[0] * self.map_resolution
        point.y = self.map_info.origin.position.y + index[1] * self.map_resolution
        point.z = 0.0
        return point

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heuristic function for A* (Manhattan distance)"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, index: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbors for path planning"""
        neighbors = []
        x, y = index

        # Check 8-connected neighborhood
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current cell

                nx, ny = x + dx, y + dy

                # Check bounds
                if (0 <= nx < self.map_info.width and
                    0 <= ny < self.map_info.height):

                    # Check if cell is free (not occupied)
                    idx = ny * self.map_info.width + nx
                    if 0 <= idx < len(self.map_data) and self.map_data[idx] == 0:
                        neighbors.append((nx, ny))

        return neighbors

    def distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Calculate distance between two map indices"""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx*dx + dy*dy)

    def check_path_conflict(self, path1: List[Point], path2: List[Point]) -> Optional[dict]:
        """Check for conflicts between two paths"""
        # Check for spatial and temporal conflicts
        min_len = min(len(path1), len(path2))

        for i in range(min_len):
            p1 = path1[i]
            p2 = path2[i]

            # Check if robots occupy same space at same time
            distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
            if distance < 0.5:  # Collision threshold
                return {
                    'time_step': i,
                    'location': ((p1.x + p2.x)/2, (p1.y + p2.y)/2),
                    'robots': (path1, path2)
                }

        return None

    def resolve_path_conflict(self, path: List[Point], other_path: List[Point],
                           conflict: dict) -> List[Point]:
        """Resolve path conflict by modifying one path"""
        # Simple resolution: delay one robot
        # In practice, more sophisticated methods would be used
        conflict_time = conflict['time_step']

        # Insert delay by duplicating a point at conflict time
        new_path = path[:conflict_time]
        delay_point = path[max(0, conflict_time-1)]  # Stay at previous position
        # Add several duplicate points to create delay
        for _ in range(5):  # 5-step delay
            new_path.append(delay_point)
        # Add remaining path
        new_path.extend(path[conflict_time:])

        return new_path


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    coordinator = MultiRobotCoordinator()
    path_planner = AdvancedPathPlanner()

    # Use MultiThreadedExecutor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coordinator)
    executor.add_node(path_planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        coordinator.get_logger().info('Multi-robot system stopped')
        path_planner.get_logger().info('Path planner stopped')
    finally:
        coordinator.destroy_node()
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Unity Integration Script for Multi-Robot Visualization
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using System.Collections.Generic;
using System.Linq;

public class MultiRobotVisualizer : MonoBehaviour
{
    [Header("Robot Prefabs")]
    public GameObject robotPrefab;
    public Material leaderMaterial;
    public Material followerMaterial;
    public Material explorerMaterial;
    public Material supportMaterial;

    [Header("Environment")]
    public GameObject mapVisualization;
    public GameObject pathVisualization;
    public GameObject communicationNetwork;

    [Header("Settings")]
    public float robotScale = 0.8f;
    public float communicationRange = 10.0f;
    public Color communicationLineColor = Color.blue;

    private Dictionary<string, GameObject> robotGameObjects;
    private Dictionary<string, LineRenderer> communicationLines;
    private Dictionary<string, List<Vector3>> robotPaths;
    private ROSTCPConnector ros;

    void Start()
    {
        robotGameObjects = new Dictionary<string, GameObject>();
        communicationLines = new Dictionary<string, LineRenderer>();
        robotPaths = new Dictionary<string, List<Vector3>>();

        ros = ROSTCPConnector.instance;

        // Subscribe to multi-robot topics
        ros.Subscribe<OdometryMsg>("/robot1/odom", (msg) => UpdateRobotPose("robot1", msg));
        ros.Subscribe<OdometryMsg>("/robot2/odom", (msg) => UpdateRobotPose("robot2", msg));
        ros.Subscribe<OdometryMsg>("/robot3/odom", (msg) => UpdateRobotPose("robot3", msg));

        // Subscribe to task assignment topics
        ros.Subscribe<StringMsg>("/robot1/task_assignment", (msg) => UpdateRobotTask("robot1", msg));
        ros.Subscribe<StringMsg>("/robot2/task_assignment", (msg) => UpdateRobotTask("robot2", msg));
        ros.Subscribe<StringMsg>("/robot3/task_assignment", (msg) => UpdateRobotTask("robot3", msg));

        // Timer for updating visualization
        InvokeRepeating("UpdateVisualization", 0.0f, 0.1f);
    }

    void UpdateRobotPose(string robotId, OdometryMsg odomMsg)
    {
        // Ensure robot exists in visualization
        if (!robotGameObjects.ContainsKey(robotId))
        {
            CreateRobotVisualization(robotId);
        }

        // Update robot position and orientation
        GameObject robotObj = robotGameObjects[robotId];
        if (robotObj != null)
        {
            robotObj.transform.position = new Vector3(
                (float)odomMsg.pose.pose.position.x,
                (float)odomMsg.pose.pose.position.y,
                (float)odomMsg.pose.pose.position.z
            );

            robotObj.transform.rotation = new Quaternion(
                (float)odomMsg.pose.pose.orientation.x,
                (float)odomMsg.pose.pose.orientation.y,
                (float)odomMsg.pose.pose.orientation.z,
                (float)odomMsg.pose.pose.orientation.w
            );
        }

        // Store pose for path visualization
        if (!robotPaths.ContainsKey(robotId))
        {
            robotPaths[robotId] = new List<Vector3>();
        }

        Vector3 currentPos = new Vector3(
            (float)odomMsg.pose.pose.position.x,
            (float)odomMsg.pose.pose.position.y,
            (float)odomMsg.pose.pose.position.z
        );

        // Add current position to path (limit to last 100 points)
        robotPaths[robotId].Add(currentPos);
        if (robotPaths[robotId].Count > 100)
        {
            robotPaths[robotId].RemoveAt(0);
        }
    }

    void UpdateRobotTask(string robotId, StringMsg taskMsg)
    {
        // Update robot's visual indicator based on task
        if (robotGameObjects.ContainsKey(robotId))
        {
            GameObject robotObj = robotGameObjects[robotId];
            Renderer robotRenderer = robotObj.GetComponent<Renderer>();

            // Example: Change material based on task type
            string taskType = taskMsg.data.Split(':')[0]; // Get task type from "task_type:x,y" format
            switch (taskType)
            {
                case "explore":
                    robotRenderer.material = explorerMaterial;
                    break;
                case "follow":
                    robotRenderer.material = followerMaterial;
                    break;
                case "support":
                    robotRenderer.material = supportMaterial;
                    break;
                default:
                    robotRenderer.material = followerMaterial; // Default
                    break;
            }
        }
    }

    void CreateRobotVisualization(string robotId)
    {
        GameObject robotObj = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
        robotObj.name = $"Robot_{robotId}";
        robotObj.transform.localScale *= robotScale;

        // Set initial material based on robot identity
        Renderer renderer = robotObj.GetComponent<Renderer>();
        if (robotId.Contains("1")) // Assuming robot1 is leader
        {
            renderer.material = leaderMaterial;
        }
        else
        {
            renderer.material = followerMaterial;
        }

        robotGameObjects[robotId] = robotObj;

        // Create communication line renderer
        GameObject lineObj = new GameObject($"CommLine_{robotId}");
        LineRenderer lineRenderer = lineObj.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.startColor = communicationLineColor;
        lineRenderer.endColor = communicationLineColor;
        communicationLines[robotId] = lineRenderer;
    }

    void UpdateVisualization()
    {
        // Update communication network visualization
        UpdateCommunicationNetwork();

        // Update path visualizations
        UpdatePathVisualizations();
    }

    void UpdateCommunicationNetwork()
    {
        // Update communication lines between robots
        foreach (var robotEntry in robotGameObjects)
        {
            string robotId = robotEntry.Key;
            GameObject robotObj = robotEntry.Value;

            if (robotObj != null && communicationLines.ContainsKey(robotId))
            {
                // Find nearby robots within communication range
                List<GameObject> neighbors = new List<GameObject>();
                foreach (var otherRobotEntry in robotGameObjects)
                {
                    if (otherRobotEntry.Key != robotId)
                    {
                        GameObject otherRobotObj = otherRobotEntry.Value;
                        if (otherRobotObj != null)
                        {
                            float distance = Vector3.Distance(
                                robotObj.transform.position,
                                otherRobotObj.transform.position
                            );

                            if (distance <= communicationRange)
                            {
                                neighbors.Add(otherRobotObj);
                            }
                        }
                    }
                }

                // Update communication line renderer
                LineRenderer lineRenderer = communicationLines[robotId];
                lineRenderer.positionCount = neighbors.Count + 1;
                lineRenderer.SetPosition(0, robotObj.transform.position);

                for (int i = 0; i < neighbors.Count; i++)
                {
                    lineRenderer.SetPosition(i + 1, neighbors[i].transform.position);
                }
            }
        }
    }

    void UpdatePathVisualizations()
    {
        // Update path visualizations for each robot
        foreach (var pathEntry in robotPaths)
        {
            string robotId = pathEntry.Key;
            List<Vector3> pathPoints = pathEntry.Value;

            if (pathPoints.Count > 1 && robotGameObjects.ContainsKey(robotId))
            {
                // Create or update path visualization
                GameObject pathObj = GameObject.Find($"Path_{robotId}");
                if (pathObj == null)
                {
                    pathObj = new GameObject($"Path_{robotId}");
                }

                LineRenderer pathRenderer = pathObj.GetComponent<LineRenderer>();
                if (pathRenderer == null)
                {
                    pathRenderer = pathObj.AddComponent<LineRenderer>();
                    pathRenderer.material = new Material(Shader.Find("Sprites/Default"));
                    pathRenderer.startWidth = 0.02f;
                    pathRenderer.endWidth = 0.02f;
                    pathRenderer.startColor = Color.green;
                    pathRenderer.endColor = Color.red;
                }

                pathRenderer.positionCount = pathPoints.Count;
                for (int i = 0; i < pathPoints.Count; i++)
                {
                    pathRenderer.SetPosition(i, pathPoints[i]);
                }
            }
        }
    }

    // Method to receive formation information from ROS
    public void UpdateFormationInfo(Dictionary<string, string> robotRoles)
    {
        foreach (var roleEntry in robotRoles)
        {
            string robotId = roleEntry.Key;
            string role = roleEntry.Value;

            if (robotGameObjects.ContainsKey(robotId))
            {
                GameObject robotObj = robotGameObjects[robotId];
                Renderer renderer = robotObj.GetComponent<Renderer>();

                switch (role.ToLower())
                {
                    case "leader":
                        renderer.material = leaderMaterial;
                        break;
                    case "explorer":
                        renderer.material = explorerMaterial;
                        break;
                    case "support":
                        renderer.material = supportMaterial;
                        break;
                    default:
                        renderer.material = followerMaterial;
                        break;
                }
            }
        }
    }
}
```

## Common Pitfalls
- **Coordination Complexity**: Managing multiple robots requires sophisticated algorithms to avoid conflicts and ensure efficient task execution
- **Communication Overhead**: High-frequency communication between robots can congest networks and affect real-time performance
- **Formation Stability**: Maintaining formations while navigating obstacles requires advanced path planning and control
- **Task Allocation**: Efficiently assigning tasks to robots with different capabilities is computationally complex
- **Collision Avoidance**: Preventing collisions between multiple moving robots requires predictive algorithms and coordination
- **Synchronization Issues**: Timing differences between robots can cause coordination failures
- **Scalability**: Multi-robot algorithms must scale efficiently as the number of robots increases

## Checkpoints / Mini-Exercises
1. Implement a basic multi-robot formation keeping system with leader-follower topology
2. Create a task allocation system that assigns exploration tasks to multiple robots
3. Develop a collision avoidance system for multiple robots navigating in the same environment
4. Build a communication visualization system that shows network connectivity between robots
5. Implement a path planning system that accounts for other robots' planned paths
6. Create a health monitoring system that detects and responds to robot failures in a multi-robot team

## References
- [Multi-Robot Systems Book](https://link.springer.com/book/10.1007/978-3-030-77240-5)
- [ROS 2 Multi-Robot Tutorials](https://navigation.ros.org/multi_nav/index.html)
- [Distributed Robotics Algorithms](https://ieeexplore.ieee.org/document/9143508)
- [Multi-Robot Path Planning](https://www.sciencedirect.com/science/article/pii/S0921889020304560)
- [Formation Control in Robotics](https://arxiv.org/abs/2007.13099)