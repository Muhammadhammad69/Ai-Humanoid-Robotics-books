# Physical AI & Humanoid Robotics – Course Introduction

Welcome to the **Physical AI & Humanoid Robotics** capstone quarter. This course bridges the gap between **digital intelligence** and the **physical world**, enabling students to build AI-driven humanoid systems capable of perception, reasoning, and action.

## Course Focus & Theme
**Physical AI** explores intelligent agents that operate in real environments, understand physics, and perform human-like interactions. Students apply their AI fundamentals to control humanoid robots using modern robotic frameworks such as **ROS 2**, **Gazebo**, **Unity**, and **NVIDIA Isaac**.

## Goal
To train students in designing, simulating, and deploying autonomous humanoid robots by integrating AI models with real-time robotic control systems.

## Quarter Overview
This quarter introduces the full ecosystem of embodied AI:
- Creating digital twins  
- Designing robot nervous systems  
- Generating advanced perception pipelines  
- Connecting LLM-based reasoning to robotic action  

You will learn to build humanoid behaviors in both **high-fidelity simulations** and **real-world environments**.

---

# 📘 Module Introductions

## **Module 1: The Robotic Nervous System (ROS 2)**
This module builds the foundational communication and control layer for a humanoid robot.

- Understanding how robots "think" using **ROS 2 nodes, topics, services, and actions**  
- Building communication between Python AI agents and ROS controllers using **rclpy**  
- Learning **URDF** to define a humanoid’s links, joints, sensors, and physical structure  
- Executing movement commands, sensor reading, and basic robot behaviors  

---

## **Module 2: The Digital Twin (Gazebo & Unity)**
This module focuses on creating a virtual environment where your humanoid robot can safely learn and interact.

- Building physically accurate simulations with **Gazebo**  
- Understanding gravity, collisions, joint constraints, and physics engines  
- Integrating Unity for **high-fidelity rendering** and interactive scenes  
- Simulating robot sensors: **LiDAR, depth cameras, IMUs, RGB-D vision**  

---

## **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**
This module introduces GPU-accelerated perception and navigation for advanced humanoid capabilities.

- Using **NVIDIA Isaac Sim** for photorealistic training and synthetic data generation  
- Implementing **Isaac ROS** for high-speed, hardware-accelerated perception  
- Using **VSLAM** for real-time localization and mapping  
- Designing bipedal path planning using **Nav2**  

---

## **Module 4: Vision–Language–Action (VLA)**
This module fuses LLMs, computer vision, and robotics to achieve natural human–robot interaction.

- Voice commands using **OpenAI Whisper**  
- Converting natural language tasks (“Clean the room”) into ROS 2 action sequences  
- Using LLMs for reasoning, decomposition, and robotic planning  
- **Capstone Project:**  
  Build **The Autonomous Humanoid** — a robot that:  
  - Listens to a voice command  
  - Plans navigation  
  - Identifies objects  
  - Avoids obstacles  
  - Manipulates the target object using vision-guided actions  
