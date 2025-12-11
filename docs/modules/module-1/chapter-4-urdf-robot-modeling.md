# Chapter 4: URDF Robot Modeling

## Overview
This chapter explores URDF (Unified Robot Description Format), the standard XML-based format for representing robot models in ROS. You'll learn how to create detailed robot descriptions that include kinematic chains, visual and collision properties, and physical parameters. URDF is essential for simulation, visualization, and control of robotic systems, particularly for humanoid robots with complex kinematic structures.

URDF enables the description of a robot's physical and kinematic properties in a structured way that can be used by various ROS tools and simulation environments. Understanding URDF is crucial for creating robots that can be properly simulated, visualized in RViz, and controlled using ROS-based tools and algorithms.

## Learning Objectives
- Understand the structure and components of URDF files
- Create complete robot models with proper kinematic chains
- Define visual and collision properties for robot links
- Add physical properties including mass, inertia, and friction
- Integrate URDF models with simulation environments like Gazebo
- Validate and debug URDF models using ROS tools

## Key Concepts

### URDF Structure
URDF is an XML-based format that describes robots as collections of links connected by joints. The structure includes visual elements for rendering, collision elements for physics simulation, and physical properties for dynamics calculations.

### Links and Joints
Links represent rigid bodies in the robot, while joints define the connections and degrees of freedom between links. The combination forms the kinematic chain that determines how the robot moves.

### Visual and Collision Properties
Visual elements define how the robot appears in visualization tools, while collision elements define the shapes used for physics simulation and collision detection. These can be different to optimize performance and appearance.

### Inertial Properties
Mass, center of mass, and inertia tensors are crucial for physics simulation and control. Properly defined inertial properties ensure realistic robot behavior in simulation environments.

## Technical Deep Dive

### URDF File Structure

A URDF file typically contains:

**Robot Element**: The root element that contains the entire robot description.

**Link Elements**: Define rigid bodies with visual, collision, and inertial properties.

**Joint Elements**: Define connections between links with specific types (revolute, prismatic, fixed, etc.) and limits.

**Material Elements**: Define visual appearance properties like color and texture.

### Link Components

Each link in URDF can contain:
- **Visual**: Defines how the link appears in visualization
- **Collision**: Defines shapes for collision detection and physics
- **Inertial**: Defines mass, center of mass, and inertia tensor
- **Origin**: Defines the pose of the link relative to its parent

### Joint Types and Properties

Joints connect links and define their relative motion:
- **Fixed**: No relative motion between links
- **Revolute**: Single rotational degree of freedom with limits
- **Continuous**: Single rotational degree of freedom without limits
- **Prismatic**: Single translational degree of freedom with limits
- **Floating**: Six degrees of freedom (rarely used)
- **Planar**: Motion constrained to a plane

### URDF Robot Structure (Text Diagram)
```
Robot (Root)
  |
  +-- Link: base_link
  |   +-- Visual: Cylinder (body)
  |   +-- Collision: Cylinder (body)
  |   +-- Inertial: Mass=10kg
  |
  +-- Joint: base_to_torso (fixed)
  |   |
  |   +-- Link: torso
  |       +-- Visual: Box (torso)
  |       +-- Collision: Box (torso)
  |       +-- Inertial: Mass=8kg
  |
  +-- Joint: torso_to_head (revolute)
  |   |
  |   +-- Link: head
  |       +-- Visual: Sphere (head)
  |       +-- Collision: Sphere (head)
  |       +-- Inertial: Mass=2kg
  |
  +-- Joint: torso_to_left_arm (revolute)
      |
      +-- Link: left_upper_arm
          +-- Visual: Cylinder (upper arm)
          +-- Collision: Cylinder (upper arm)
          +-- Inertial: Mass=1kg
```

## Code Examples

### Basic URDF Robot Model
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
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
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
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
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
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
    <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

</robot>
```

### URDF with Gazebo Integration
```xml
<?xml version="1.0"?>
<robot name="gazebo_integrated_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include Gazebo-specific plugins and materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties for the base link -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Joint with actuator control -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="10"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties for the wheel -->
  <gazebo reference="wheel_link">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>10000000.0</kp>
    <kd>1.0</kd>
  </gazebo>

  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>base_to_wheel</left_joint>
      <right_joint>base_to_wheel</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.4</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### Python Script to Parse URDF
```python
#!/usr/bin/env python3
"""
Simple script to parse and analyze URDF files
"""
import xml.etree.ElementTree as ET
from collections import defaultdict


class URDFAnalyzer:
    def __init__(self, urdf_file_path):
        self.tree = ET.parse(urdf_file_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get('name')

    def get_links(self):
        """Get all links in the URDF"""
        links = []
        for link in self.root.findall('link'):
            links.append({
                'name': link.get('name'),
                'visual': link.find('visual') is not None,
                'collision': link.find('collision') is not None,
                'inertial': link.find('inertial') is not None
            })
        return links

    def get_joints(self):
        """Get all joints in the URDF"""
        joints = []
        for joint in self.root.findall('joint'):
            joints.append({
                'name': joint.get('name'),
                'type': joint.get('type'),
                'parent': joint.find('parent').get('link') if joint.find('parent') is not None else None,
                'child': joint.find('child').get('link') if joint.find('child') is not None else None
            })
        return joints

    def print_summary(self):
        """Print a summary of the URDF structure"""
        print(f"Robot: {self.robot_name}")
        print(f"Links: {len(self.get_links())}")
        print(f"Joints: {len(self.get_joints())}")

        print("\nLinks:")
        for link in self.get_links():
            print(f"  - {link['name']}: visual={link['visual']}, collision={link['collision']}, inertial={link['inertial']}")

        print("\nJoints:")
        for joint in self.get_joints():
            print(f"  - {joint['name']}: {joint['type']} ({joint['parent']} -> {joint['child']})")


def main():
    # Example usage
    # analyzer = URDFAnalyzer('path_to_urdf_file.urdf')
    # analyzer.print_summary()
    pass


if __name__ == '__main__':
    main()
```

## Common Pitfalls
- **Inertial Properties**: Incorrect mass or inertia values can cause simulation instability or unrealistic behavior
- **Joint Limits**: Forgetting to set appropriate joint limits can lead to kinematic errors or collisions
- **Collision vs Visual**: Using overly complex collision meshes can slow down physics simulation
- **Origin Definitions**: Incorrect origin definitions can cause links to be positioned incorrectly relative to each other
- **Material Definitions**: Missing or incorrect material definitions can affect visualization and physics properties

## Checkpoints / Mini-Exercises
1. Create a URDF model for a simple wheeled robot with differential drive
2. Add proper inertial properties to your robot model and validate them
3. Create a URDF with a gripper mechanism and test its kinematics
4. Integrate your URDF with Gazebo by adding appropriate plugins
5. Use RViz to visualize your URDF model and check for any visual artifacts

## References
- [URDF Documentation](http://wiki.ros.org/urdf)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo URDF Integration](http://gazebosim.org/tutorials/?tut=ros_urdf)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)