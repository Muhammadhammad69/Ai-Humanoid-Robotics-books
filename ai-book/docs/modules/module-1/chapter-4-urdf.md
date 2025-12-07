---
id: chapter-4-urdf
title: URDF for Humanoid Robots
sidebar_position: 4
---

# URDF for Humanoid Robots

## Overview
The Unified Robot Description Format (URDF) is an XML format used in ROS 2 to describe all aspects of a robot. For humanoid robots, URDF is crucial for defining their physical structure, including links (rigid bodies), joints (connections between links), and their visual and collision properties. This chapter will explore the essential elements of URDF, focusing on how to model the complex kinematics and dynamics of humanoid robots, enabling simulation in tools like Gazebo and visualization in RViz.

## Learning Objectives
- Understand the purpose and structure of URDF files.
- Learn to define links and joints within a URDF, including their properties.
- Model the visual and collision geometries of humanoid robot components.
- Integrate inertial properties for accurate simulation.
- Understand how URDF contributes to the control and simulation of humanoid robots.

## Key Concepts
### Links
Links are the rigid bodies of a robot. Each link has a name, and properties that define its mass, inertia (inertial element), visual representation (visual element), and collision properties (collision element). For a humanoid, examples include the torso, upper arm, forearm, hand, thigh, calf, and foot.

### Joints
Joints define the kinematic and dynamic properties of the connection between two links. They specify the type of motion allowed (e.g., `revolute`, `continuous`, `prismatic`, `fixed`) and their parent and child links. Joints are critical for representing the articulations of a humanoid robot's limbs.

### Coordinate Frames
URDF uses a right-handed coordinate system. Each link and joint implicitly defines its own local coordinate frame. Transformations between these frames are defined by the `origin` element within `joint`, `visual`, and `collision` tags.

### Visual and Collision Geometry
-   **Visual**: Describes the graphical appearance of a link, typically used for rendering in simulators and visualization tools (e.g., RViz).
-   **Collision**: Defines the simplified geometry used for collision detection in physics engines. Often, collision geometry is a simplified version of visual geometry to reduce computational load.

### Inertial Properties
The `inertial` element specifies the mass, center of mass (origin), and inertia tensor of a link. These properties are essential for accurate dynamic simulation of the robot's movement and balance.

## Technical Deep Dive

A URDF file is an XML document that defines the robot as a tree-like structure, starting from a base link. Each link is connected by a joint, leading to a hierarchy.

**Link-Joint Tree Structure (Text Description)**:
```
base_link (fixed)
  |-- torso_link (revolute joint: waist_pitch_joint)
      |-- left_upper_arm_link (revolute joint: left_shoulder_pitch_joint)
      |   |-- left_forearm_link (revolute joint: left_elbow_pitch_joint)
      |   |-- left_hand_link (fixed joint: left_wrist_fixed_joint)
      |-- right_upper_arm_link (revolute joint: right_shoulder_pitch_joint)
      |   |-- right_forearm_link (revolute joint: right_elbow_pitch_joint)
      |   |-- right_hand_link (fixed joint: right_wrist_fixed_joint)
      |-- left_thigh_link (revolute joint: left_hip_pitch_joint)
      |   |-- left_calf_link (revolute joint: left_knee_pitch_joint)
      |   |-- left_foot_link (revolute joint: left_ankle_pitch_joint)
      |-- right_thigh_link (revolute joint: right_hip_pitch_joint)
      |   |-- right_calf_link (revolute joint: right_knee_pitch_joint)
      |   |-- right_foot_link (revolute joint: right_ankle_pitch_joint)
```
This hierarchical structure, formed by parent-child relationships defined by joints, dictates the robot's kinematics.

### Essential URDF Tags
-   `<robot>`: The root element, containing the entire robot description.
-   `<link>`: Defines a rigid body with physical properties.
    -   `<inertial>`: Mass, center of mass, and inertia matrix.
    -   `<visual>`: Geometry, material, and origin for rendering.
    -   `<collision>`: Geometry and origin for collision detection.
-   `<joint>`: Defines a connection between two links.
    -   `type`: (e.g., `revolute`, `prismatic`, `fixed`, `continuous`).
    -   `<parent>`, `<child>`: Specify connected links.
    -   `<origin>`: Relative position and orientation of the joint frame with respect to the parent link.
    -   `<axis>`: Axis of rotation for rotational joints or translation for prismatic joints.
    -   `<limit>`: Joint limits (lower, upper, velocity, effort) for non-continuous joints.
    -   `<dynamics>`: Friction and damping coefficients.
    -   `<mimic>`: To mimic another joint's movement.
-   `<material>`: Defines colors and textures, often referenced by visual elements.
-   `<gazebo>`: Used for Gazebo-specific properties that are not part of standard URDF (e.g., plugins, materials).

## Code Examples

### Simple 2-Link Humanoid Arm Segment URDF Example
This example shows a simplified URDF for a 2-link robotic arm, representing a segment of a humanoid robot.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Upper Arm Link -->
  <link name="upper_arm_link">
    <inertial>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Forearm Link -->
  <link name="forearm_link">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="50" velocity="5"/>
  </joint>

</robot>
```
*Save this as `simple_humanoid_arm.urdf`*

### Fragments Demonstrating Specific Tags
`<material>` definition (often placed at the top of the URDF or in a separate file):
```xml
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
```
`<limit>` within a joint:
```xml
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
```
`<origin>` for relative positioning:
```xml
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
```

## Common Pitfalls
-   **Missing Inertial Properties**: Leads to incorrect physics simulation.
-   **Incorrect Joint Origins**: Robot appears disjointed or joints rotate unexpectedly.
-   **Collision vs. Visual Mismatch**: Using complex visual meshes for collision can significantly slow down simulations. Simplified collision geometries are usually preferred.
-   **URDF vs. SDF**: Confusing URDF (robot description) with SDF (full world description for Gazebo).

## Checkpoints / Mini-Exercises
1.  Extend the `simple_humanoid_arm.urdf` to include a hand link with a fixed wrist joint.
2.  Modify the visual material of the forearm to be yellow and change the elbow joint's movement limits.
3.  Describe the difference between a `revolute` joint and a `continuous` joint, and provide an example of where each might be used in a humanoid robot.

## References
-   [URDF Wiki Page](http://wiki.ros.org/urdf)
-   [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
-   [ROS 2 Tutorials - Using URDF with Gazebo](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-With-Robot-State-Publisher.html)
-   [URDF XML Schema Reference (implied by ROS Wiki)](http://wiki.ros.org/urdf/XML)