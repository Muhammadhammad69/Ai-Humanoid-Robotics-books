# Chapter 5 - URDF for Humanoid Robots

## Overview

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. In this chapter, you will learn the fundamental components of URDF, how to define the physical and kinematic properties of a humanoid robot, and how to prepare your URDF model for visualization and simulation. A well-constructed URDF is crucial for accurate robot representation and interaction within the ROS ecosystem.

---

## Learning Objectives
-   Understand the purpose and structure of URDF files.
-   Define links, joints, and transmissions for a basic humanoid robot model.
-   Incorporate visual and collision properties into URDF elements.
-   Prepare URDF models for use in simulation environments like Gazebo.

---

## What is URDF?

URDF (Unified Robot Description Format) is an XML file format that allows you to describe the physical characteristics of a robot. This description includes its kinematic and dynamic properties, visual appearance, and collision geometry. ROS tools like `rviz` (for visualization) and simulation environments like Gazebo rely heavily on URDF files to understand and interact with your robot.

For humanoid robots, URDF is essential because it defines:
*   **Body Structure**: Each segment of the robot (e.g., torso, upper arm, forearm, hand) is a `link`.
*   **Degrees of Freedom**: How these segments connect and move relative to each other are `joints`.
*   **Actuation**: How motors (`transmissions`) drive the joints.

## Links and Joints: The Kinematic Chain

### Links

A `<link>` element in URDF represents a rigid body segment of the robot. Each link has a name, and can define its `inertial` (mass properties), `visual` (appearance), and `collision` (physical interaction) properties.

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

### Joints

A `<joint>` element connects two links: a `parent` link and a `child` link. It defines the type of motion allowed between them (e.g., revolute, prismatic, fixed) and the axis of rotation/translation.

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Position of child wrt parent -->
  <axis xyz="0 0 1"/> <!-- Axis of rotation -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
</joint>
```

Common joint types:
*   **`revolute`**: Rotational joint with a limited range.
*   **`continuous`**: Rotational joint with unlimited range (e.g., spinning wheel).
*   **`prismatic`**: Linear joint with a limited range (e.g., a piston).
*   **`fixed`**: No motion allowed (child link is rigidly attached to parent).

## Transmissions: Connecting Joints to Actuators

The `<transmission>` element defines how a joint is actuated by a motor (actuator). It maps the motion of a joint to the effort of an actuator. This is particularly important for `ros2_control` which manages the hardware interfaces.

```xml
<transmission name="shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_motor">
    <mechanicalReduction>1</mechanicalReduction>
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </actuator>
</transmission>
```

## Building a Basic Humanoid URDF

Let's construct a very simple humanoid-like structure: a `base_link` (torso) connected to an `upper_arm_link` via a `shoulder_joint`.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>

  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0.1 0.25" rpy="0 0 0"/> <!-- Attaches to side of torso -->
    <axis xyz="1 0 0"/> <!-- Rotates around X-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Upper Arm Link -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1"/> <!-- Center cylinder along Y, extend down -->
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Transmission for Shoulder Joint -->
  <transmission name="shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

</robot>
```

### Visualizing URDF with `rviz`

To visualize this URDF, save it as `simple_humanoid.urdf` in your `urdf-examples` directory. You can then use the `display.launch.py` from `urdf_tutorial` package in ROS 2.

```bash
# In your ROS 2 workspace, after building urdf_tutorial
ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix my_ros2_pkg)/share/my_ros2_pkg/urdf-examples/simple_humanoid.urdf
```
(Note: You might need to install `urdf_tutorial` and adjust the path to your URDF file.)

## Visuals & Collisions Tags

*   **`<visual>`**: Defines the graphical representation of the link. This is what you see in visualization tools like `rviz`. It can specify geometry (box, cylinder, sphere, mesh), origin (offset), and material (color, texture).
*   **`<collision>`**: Defines the simplified geometry used for physics simulation and collision detection. This is usually a simpler shape than the visual model to reduce computational load. It also specifies geometry and origin.

It's common for `visual` and `collision` geometries to be identical for simple shapes or early development. However, for performance and accuracy in complex simulations, collision geometries are often simplified (e.g., a complex mesh replaced by a bounding box or sphere).

## Preparing URDF for Simulation in Gazebo

Gazebo is a powerful 3D robotics simulator. For a URDF to work effectively in Gazebo, it often requires additional tags or modifications:

*   **`<gazebo>` tags**: These are non-standard URDF tags that Gazebo understands. They are typically used to define:
    *   **Colors**: For links if not defined in `visual` material.
    *   **Sensors**: Attaching cameras, lidar, IMUs to links.
    *   **Plugins**: To provide functionality like joint controllers, custom physics, or interacting with ROS 2.
    *   **Material properties**: Like friction, damping.

*   **`xacro`**: Often, URDFs become very verbose and repetitive. `xacro` (XML Macros) is a preprocessor that allows you to use macros, properties, and includes to make URDF files more modular and readable. This is highly recommended for complex robots.

```xml
<!-- Example of a simple Gazebo tag for a link -->
<gazebo reference="base_link">
  <material>Gazebo/Red</material>
  <static>false</static> <!-- True if the link should not move -->
</gazebo>
```

---

## Key Takeaways

*   URDF uses XML to describe a robot's kinematic, dynamic, visual, and collision properties.
*   `<link>` elements represent rigid body parts, while `<joint>` elements define their connections and allowed motions.
*   `<transmission>` elements link joints to actuators, crucial for `ros2_control`.
*   `xacro` is a powerful tool for creating modular and readable URDF files.
*   `visual` tags define rendering, while `collision` tags define physical interaction in simulation.
*   `<gazebo>` tags extend URDF for specific simulation properties in Gazebo.

---

## Exercises

1.  **Extend Simple Humanoid**: Add a second joint (e.g., an `elbow_joint` of type `revolute`) and a new link (`forearm_link`) to the `simple_humanoid.urdf`. Visualize your updated robot in `rviz`.
2.  **Explore Joint Types**: Change the `shoulder_joint` type from `revolute` to `fixed` and then `prismatic`. Observe how the robot's structure changes in visualization.
3.  **Add Visual Properties**: For your `simple_humanoid.urdf`, add a `sphere` geometry for the `upper_arm_link` visual and change its material to `green`.
4.  **URDF and `xacro` Research**: Research the benefits of using `xacro` over plain URDF. Convert a small part of the `simple_humanoid.urdf` into `xacro` macros. Explain how this improves modularity.
5.  **Collision Geometry Simplification**: For a hypothetical complex mesh of a robot hand, describe how you would simplify its `<collision>` geometry compared to its `<visual>` geometry for optimal simulation performance.