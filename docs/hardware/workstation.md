# Digital Twin Workstation Requirements

## Overview
The workstation is the most critical component for this course. It handles three heavy computational loads simultaneously:
- Physics Simulation (Isaac Sim/Gazebo)
- Visual Perception (SLAM/Computer Vision)
- Generative AI (LLMs/VLA)

## Minimum Specifications

### GPU (Critical Bottleneck)
**Required:** NVIDIA RTX 4070 Ti (12GB VRAM) or higher

**Why RTX is Required:**
- NVIDIA Isaac Sim is an Omniverse application requiring Ray Tracing capabilities
- Standard laptops (MacBooks or non-RTX Windows machines) will NOT work
- High VRAM needed to load USD (Universal Scene Description) assets for robot and environment
- Must run VLA (Vision-Language-Action) models simultaneously

**Recommended:** RTX 3090 or 4090 (24GB VRAM)
- Allows smoother "Sim-to-Real" training
- Better performance for complex scene rendering

### CPU
**Required:** Intel Core i7 (13th Gen+) or AMD Ryzen 9

**Why:** Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive

### RAM
**Required:** 64 GB DDR5

**Warning:** 32 GB is the absolute minimum but will crash during complex scene rendering

### Operating System
**Required:** Ubuntu 22.04 LTS

**Important Note:**
- While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux
- Dual-booting or dedicated Linux machines are mandatory for friction-free experience

## Cloud Alternative

### AWS/Azure Cloud Workstations
**Best for:** Rapid deployment or students with weak laptops

**Instance Type:** AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge

**Software:** NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI)

**Cost Calculation:**
- Instance cost: ~$1.50/hour (spot/on-demand mix)
- Usage: 10 hours/week × 12 weeks = 120 hours
- Storage (EBS volumes): ~$25/quarter
- **Total Cloud Bill:** ~$205 per quarter

**Limitations:**
- High latency for real robot control
- Ongoing operational expenses (OpEx)
- Cannot eliminate hardware entirely for "Physical AI"

## Architecture Role
The workstation serves as the **Sim Rig** in the overall system:
- Runs Isaac Sim, Gazebo, Unity
- Trains LLM/VLA models
- Develops and tests code before edge deployment