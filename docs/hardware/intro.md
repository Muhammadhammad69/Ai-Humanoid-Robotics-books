# Hardware Requirements Introduction

## Course Computational Demands

This course is technically demanding and sits at the intersection of three heavy computational loads:

1. **Physics Simulation** - Isaac Sim/Gazebo
2. **Visual Perception** - SLAM/Computer Vision  
3. **Generative AI** - LLMs/VLA (Vision-Language-Action)

## The Physical AI Promise

Because the capstone involves a "Simulated Humanoid," the primary investment must be in **High-Performance Workstations**. However, to fulfill the "Physical AI" promise, you also need:

- **Edge Computing Kits** (brains without bodies)
- **Specific robot hardware** (physical platforms)

## Three-Tier Hardware Architecture

The course requires three distinct hardware components working together:

### 1. Digital Twin Workstation (Required per Student)
The simulation and training environment where students develop and test their AI models in virtual space.

**Primary Role:** Run Isaac Sim, train models, develop algorithms

**Cost Range:** $2,000-4,000 (on-premise) or ~$205/quarter (cloud)

[→ Full Workstation Specs](/docs/hardware/workstation)

### 2. Physical AI Edge Kit (Required per Student)
The "nervous system" that students set up on a desk to learn deployment before moving to robots.

**Primary Role:** Deploy trained models, run inference, connect sensors

**Cost Range:** ~$700 per kit

[→ Full Edge Kit Details](/docs/hardware/edge-kit)

### 3. Robot Platform (Shared Lab Resource)
The physical embodiment that receives commands from the edge brain.

**Primary Role:** Validate Sim-to-Real transfer, physical testing

**Cost Range:** $600-16,000+ depending on platform choice

[→ Robot Platform Options](/docs/hardware/robot-platform)

## Critical Hardware Note

**Standard laptops will NOT work for this course.**

- MacBooks: Not compatible with Isaac Sim
- Non-RTX Windows machines: Lack required Ray Tracing capabilities
- Chromebooks/tablets: Insufficient computing power

**You must have RTX-enabled workstations** or restructure the course to rely on cloud-based instances (AWS RoboMaker or NVIDIA Omniverse Cloud).

## Two Deployment Models

### Option 1: On-Premise Lab (High CapEx)

**Best for:** Long-term programs, institutional investment

**Advantages:**
- One-time purchase
- No ongoing costs after initial investment
- Full control and no latency
- Students learn on professional-grade equipment

**Typical Investment:**
- Workstations: $2,000-4,000 × students
- Edge Kits: $700 × students  
- Robots: $3,000-16,000 (shared, 1-2 per lab)

**Total Lab Setup:** $20,000-50,000 for 10-15 students

### Option 2: Cloud-Native Lab (High OpEx)

**Best for:** Rapid deployment, students with weak laptops

**Advantages:**
- Lower upfront cost
- Scalable on-demand
- No hardware maintenance
- Access from anywhere

**Typical Costs:**
- Cloud instances: ~$205/student/quarter
- Edge Kits: $700 × students (still required)
- Robots: $3,000+ (still required for physical testing)

**Important:** You cannot eliminate hardware entirely for "Physical AI" - edge devices and at least one robot are mandatory.

## The Latency Consideration

**Cloud simulation works well, but controlling real robots from cloud instances is dangerous due to latency.**

**Solution:**
1. Train in the Cloud (workstation/cloud instance)
2. Download model weights
3. Deploy to local Edge Kit (Jetson)
4. Control robot locally with low latency

## Minimum Viable Lab

For a budget-conscious start, the minimum viable lab includes:

1. **Cloud Workstations** for simulation (~$205/student/quarter)
2. **1-2 Edge Kits** as shared learning resources ($700 each)
3. **1 Proxy Robot** like Unitree Go2 ($3,000)

**Total First Quarter:** ~$6,000-8,000 for 10 students

Students can develop primarily in simulation, with scheduled access to edge kits and robot for physical validation.

## Investment Decision Framework

### Choose On-Premise If:
- You have 2+ year program timeline
- Budget for upfront capital expenditure
- Need consistent, high-performance access
- Want to build permanent lab infrastructure

### Choose Cloud-Native If:
- Starting with pilot program
- Students have varying hardware capabilities  
- Prefer operational expenses over capital
- Need rapid deployment (weeks not months)

### Hybrid Approach (Recommended):
- Cloud workstations for flexibility
- On-premise edge kits for hands-on learning
- 1-2 shared robots for physical validation

## Next Steps

Review each hardware component in detail:

1. [Workstation Requirements](/docs/hardware/workstation) - Understand GPU, CPU, RAM needs
2. [Edge Kit Specifications](/docs/hardware/edge-kit) - Learn about Jetson and sensors
3. [Robot Platform Options](/docs/hardware/robot-platform) - Choose your physical platform

## Budget Planning Summary

| Component | On-Premise | Cloud | Notes |
|-----------|-----------|-------|-------|
| **Workstation** | $2,000-4,000 | $205/quarter | Per student |
| **Edge Kit** | $700 | $700 | Per student or shared |
| **Robot** | $3,000-16,000 | $3,000-16,000 | Shared (1-2 per lab) |
| **Lab Setup** | High CapEx | High OpEx | Choose based on timeline |

Building a Physical AI lab is a significant investment, but the modular architecture allows you to scale based on budget and program maturity.