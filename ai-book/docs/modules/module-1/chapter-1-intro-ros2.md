---
id: chapter-1-intro-ros2
title: Introduction to ROS 2
sidebar_position: 1
---

# Introduction to ROS 2

## Overview
This chapter provides a foundational understanding of the Robot Operating System 2 (ROS 2), a flexible framework for writing robot software. We will explore its core concepts, architecture, and the evolution from ROS 1, highlighting its relevance in developing complex robotic systems, especially for humanoid platforms. ROS 2 addresses the challenges of modern robotics, offering enhanced communication, security, and real-time capabilities crucial for sophisticated humanoid applications.

## Learning Objectives
- Understand the purpose and fundamental concepts of ROS 2.
- Identify the key architectural components of ROS 2.
- Explain the benefits of using ROS 2 for robotic development, particularly for humanoid robots.
- Differentiate between ROS 1 and ROS 2.

## Key Concepts
### What is ROS 2?
ROS 2 is an open-source, meta-operating system for robots. It provides libraries, tools, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms, including humanoid robots. Its distributed nature allows for modular and scalable robot applications.

### ROS 2 Architecture
The core architecture of ROS 2 is built around a decentralized network of nodes communicating through topics, services, actions, and parameters. This distributed design, powered by Data Distribution Service (DDS), enables robust and secure communication among various components, even across different machines or operating systems.

### Evolution from ROS 1
ROS 2 emerged to address critical limitations of ROS 1, such as lack of real-time support, limited multi-robot capabilities, and security vulnerabilities. Key improvements include a DDS-based communication layer, better support for embedded systems, and enhanced security features.

## Technical Deep Dive

ROS 2 leverages DDS (Data Distribution Service) as its middleware, providing discovery, serialization, and transport for inter-process communication. Unlike ROS 1's centralized master, ROS 2's DDS middleware enables a distributed architecture where nodes can discover each other dynamically without a single point of failure. This peer-to-peer communication is vital for robust and scalable multi-robot systems and distributed humanoid control.

**High-level ROS 2 Architectural Diagram (Text Description)**:
```
+----------------+       +----------------+       +----------------+
|     Node A     |-------|     Topic      |-------|     Node B     |
| (Publisher)    |       | (Data Stream)  |       | (Subscriber)   |
+----------------+       +----------------+       +----------------+
        |                                                 ^
        | Service Call                             Service Response
        v                                                 |
+----------------+       +----------------+       +----------------+
|     Node C     |<----->|     Service    |<----->|     Node D     |
| (Service Client)|       | (Request/Reply)|       | (Service Server) |
+----------------+       +----------------+       +----------------+
```
This diagram illustrates fundamental ROS 2 communication patterns. Nodes are executable processes. Topics facilitate asynchronous, many-to-many data streaming. Services provide synchronous request-reply interactions.

For humanoid robots, this architecture allows different computational units (e.g., vision processing, motion planning, balance control) to operate as independent nodes, communicating seamlessly. For instance, a vision node might publish sensor data to a topic, which a motion planning node subscribes to, and then uses a service to request a joint trajectory from a control node.

## Code Examples

Here are conceptual `ros2 run` commands typically used in ROS 2. Full code examples for nodes will be presented in subsequent chapters.

### Listing ROS 2 Nodes (Conceptual)
To see currently running ROS 2 nodes:
```bash
ros2 node list
```
*Expected Output (example)*:
```
/my_publisher
/my_subscriber
/another_node
```

### Listing ROS 2 Topics (Conceptual)
To see active ROS 2 topics:
```bash
ros2 topic list
```
*Expected Output (example)*:
```
/robot_chatter
/scan
/joint_states
```

### Listing ROS 2 Services (Conceptual)
To see available ROS 2 services:
```bash
ros2 service list
```
*Expected Output (example)*:
```
/add_two_ints
/get_current_state
```

## Common Pitfalls
- **ROS 1 vs ROS 2 mindset**: New users often try to apply ROS 1 concepts directly to ROS 2, leading to confusion with `ros1_bridge` or understanding DDS.
- **DDS Configuration**: Misconfiguring the underlying DDS implementation can lead to nodes not discovering each other.

## Checkpoints / Mini-Exercises
1.  Briefly describe the main problem that ROS 2 aims to solve compared to traditional monolithic robot software.
2.  Imagine a humanoid robot needing to pick up an object. List three potential nodes and how they might communicate using ROS 2 topics or services.

## References
-   [ROS 2 Documentation - Concepts](https://docs.ros.org/en/humble/Concepts.html)
-   [ROS 2 Documentation - About ROS 2](https://docs.ros.org/en/humble/The-ROS2-Project/About-ROS2.html)
-   [ROS 2 Documentation - Tutorials](https://docs.ros.org/en/humble/Tutorials.html)