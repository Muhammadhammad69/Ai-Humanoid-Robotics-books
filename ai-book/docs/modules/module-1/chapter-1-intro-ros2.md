# Chapter 1 - Introduction to ROS 2

## Overview

This chapter introduces the Robot Operating System 2 (ROS 2), explaining its role as the "nervous system" for modern robotics, especially in complex humanoid systems. We will delve into the motivations behind ROS 2, highlight key differences from its predecessor (ROS 1), and explore its fundamental concepts such as nodes, topics, services, and the Data Distribution Service (DDS) middleware.

---

## Learning Objectives
- Understand the necessity of ROS 2 in contemporary robotics development.
- Differentiate between ROS 1 and ROS 2 in terms of architecture and capabilities.
- Grasp the core components of a ROS 2 system, including nodes, topics, and services.
- Comprehend the role of Data Distribution Service (DDS) as the communication backbone in ROS 2.

---

## Why ROS 2? The Evolution of Robot Control

The complexity of modern robotic systems, particularly humanoid robots designed for intricate tasks in dynamic environments, demands a sophisticated and robust software framework. ROS (Robot Operating System) emerged as a de-facto standard in robotics, providing a flexible collection of tools, libraries, and conventions that simplify the task of creating complex and robust robot behavior.

However, the original ROS (now referred to as ROS 1) was not designed for certain critical applications:

*   **Real-time Capabilities**: ROS 1 lacked native real-time control, which is essential for safety-critical applications and precise robot movements.
*   **Multi-robot Systems**: Scaling ROS 1 to multiple robots was challenging due to its centralized master architecture.
*   **Security**: ROS 1 offered limited security features, making it unsuitable for commercial and industrial deployments where data integrity and privacy are paramount.
*   **Enterprise Adoption**: Its development model and lack of strong commercial backing hindered wider enterprise adoption.

**ROS 2** was re-architected from the ground up to address these limitations, bringing industrial-grade capabilities, improved real-time performance, enhanced security, and better support for multi-robot and embedded systems. For humanoid robotics, ROS 2 provides the necessary foundation for advanced perception, planning, and control algorithms to operate reliably and safely.

## ROS 1 vs. ROS 2: A Fundamental Shift

While both ROS 1 and ROS 2 aim to provide a common framework for robot application development, their underlying architectures are significantly different.

### Key Differences:

| Feature           | ROS 1                                   | ROS 2                                              |
| :---------------- | :-------------------------------------- | :------------------------------------------------- |
| **Middleware**    | Custom TCP/UDP (ROS TCP/UDP)            | DDS (Data Distribution Service) standard           |
| **Architecture**  | Centralized (ROS Master)                | Decentralized (peer-to-peer DDS)                   |
| **Real-time**     | Limited                                 | Improved, with real-time OS support                |
| **Security**      | Minimal to None                         | Enhanced (DDS-level authentication, encryption)    |
| **Multi-robot**   | Challenging                             | Natively supported with DDS                        |
| **Client Libraries** | `roscpp`, `rospy`                     | `rclcpp`, `rclpy`, `rclc` (C/C++), `rclnodejs`     |
| **Build System**  | `catkin`                                | `colcon`                                           |
| **Tooling**       | `roslaunch`, `rostopic`, `rosnode`      | `ros2 launch`, `ros2 topic`, `ros2 node` (unified CLI) |
| **OS Support**    | Linux                                   | Linux, Windows, macOS, RTOS                        |

The shift to **DDS as the middleware** is the most significant change. DDS is an international standard for real-time, scalable, and secure data exchange, enabling ROS 2 to overcome many of ROS 1's limitations.

## Key ROS 2 Concepts

Understanding these core concepts is crucial for building any ROS 2 application:

### 1. Nodes

A **node** is an executable process in a ROS 2 system. It represents a single functional block, such as a sensor driver, a motor controller, a navigation algorithm, or a high-level AI planner. ROS 2 applications are typically composed of many nodes working together.

### 2. Topics

**Topics** are named buses over which nodes exchange messages. This is a publish/subscribe communication model. A node can *publish* messages to a topic, and other nodes can *subscribe* to that topic to receive those messages. This is ideal for continuous data streams like sensor readings (e.g., camera feeds, lidar scans) or motor commands.

```ascii
+------------+        +------------+
| Publisher  |------->| Subscriber |
| (Node A)   | Topic  |<-----------|
+------------+        +------------+
```

### 3. Services

**Services** are a request/reply communication mechanism. A *service client* sends a request to a *service server*, which then processes the request and sends back a response. Services are suitable for tasks that require a single request and a corresponding response, such as querying a robot's current status, triggering a specific action, or performing a complex calculation.

```ascii
+------------+        +------------+
| Service    |--------| Service    |
| Client     | Request| Server     |
| (Node B)   |<-------| (Node C)   |
|            | Reply  |            |
+------------+        +------------+
```

### 4. Actions (High-level communication)

**Actions** are similar to services but provide feedback during execution and are typically used for long-running, goal-oriented tasks (e.g., "move to a specific location," "pick up an object"). An action client sends a goal, receives continuous feedback on the goal's progress, and eventually gets a result.

### 5. Parameters

**Parameters** are configurable values that nodes can load at startup or change during runtime. They allow for flexible configuration of node behavior without recompiling the code.

### 6. Data Distribution Service (DDS)

DDS is the underlying middleware that ROS 2 uses for all its communication. It handles:

*   **Discovery**: How nodes find each other on the network.
*   **Serialization**: How messages are converted into a format suitable for transmission.
*   **Transport**: How messages are sent across the network.
*   **Quality of Service (QoS)**: Configurable policies for reliability, latency, durability, and other communication aspects.

Unlike ROS 1's single ROS Master, DDS operates in a decentralized manner, meaning there's no single point of failure and communication can scale more effectively across distributed systems, which is crucial for complex humanoid robots.

---

## Key Takeaways

*   ROS 2 is a modern, robust robotics framework designed for real-time, secure, and multi-robot applications.
*   It fundamentally differs from ROS 1 by using DDS as its decentralized communication middleware.
*   Key ROS 2 concepts include Nodes (executable processes), Topics (publish/subscribe for continuous data), Services (request/reply for single interactions), Actions (goal-oriented tasks with feedback), and Parameters (configurable values).
*   DDS is critical for ROS 2's performance, scalability, and security.

---

## Exercises

1.  **ROS 1 vs. ROS 2 Comparison**: Research a specific feature (e.g., logging, visualization tools) in both ROS 1 and ROS 2. Write a short paragraph comparing their approaches and explain which you find more advantageous for humanoid robotics and why.
2.  **Identify Robot Components as Nodes**: Imagine a simple mobile robot with wheels, a lidar sensor, and a camera. List potential ROS 2 nodes for this robot and describe what each node would do.
3.  **Topic vs. Service Scenario**: For the mobile robot in Exercise 2, describe two scenarios: one where a ROS 2 topic would be the appropriate communication mechanism, and another where a ROS 2 service would be better. Explain your choices.
4.  **DDS Advantages**: Research one specific Quality of Service (QoS) policy in ROS 2 (e.g., `Reliability`, `Durability`, `Latency Budget`). Explain its purpose and how it could benefit a humanoid robot application.