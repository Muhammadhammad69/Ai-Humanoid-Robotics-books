# Robot Platform Options

## Overview
For the "Physical" part of the course, there are three tiers of robot options depending on budget and learning objectives. The software principles (ROS 2, VSLAM, Isaac Sim) transfer effectively across platforms.

## Option A: The "Proxy" Approach (Recommended for Budget)

### Concept
Use a quadruped (dog) or robotic arm as a proxy for humanoid development. Software principles transfer 90% effectively to humanoids.

### Recommended Robot
**Unitree Go2 Edu**
- **Price:** $1,800 - $3,000
- **Type:** Quadruped (dog-like)

**Pros:**
- Highly durable
- Excellent ROS 2 support
- Affordable enough for multiple units
- Good for learning fundamentals

**Cons:**
- Not a biped (humanoid)
- Different kinematic model

### When to Choose
- Budget-constrained labs
- Focus on ROS 2, VSLAM, and AI principles
- Need multiple robots for student groups
- Durability is priority

## Option B: The "Miniature Humanoid" Approach

### Concept
Small, table-top humanoids for direct humanoid experience at lower cost.

### Mid-Range Options

**Unitree G1**
- **Price:** ~$16,000
- **Type:** Humanoid
- **Status:** Good balance of capability and cost
- **SDK:** Open enough for custom ROS 2 controllers

**Robotis OP3**
- **Price:** ~$12,000
- **Type:** Humanoid
- **Status:** Older but stable platform
- **Community:** Established support base

### Budget Alternative

**Hiwonder TonyPi Pro**
- **Price:** ~$600
- **Type:** Miniature humanoid
- **Processor:** Raspberry Pi

**Warning:**
- Cannot run NVIDIA Isaac ROS efficiently
- Use only for kinematics (walking patterns)
- Requires separate Jetson kit for AI workloads
- Limited performance

### When to Choose
- Direct humanoid experience needed
- Mid-range budget available
- Focus on bipedal locomotion
- Table-top development preferred

## Option C: The "Premium" Lab (Sim-to-Real Specific)

### Concept
Full-scale humanoid for actual Capstone deployment and advanced research.

### Recommended Robot

**Unitree G1 Humanoid**
- **Price:** ~$16,000
- **Type:** Full humanoid

**Why This Choice:**
- One of few commercially available humanoids
- Can walk dynamically (not just static poses)
- SDK open enough for student ROS 2 controllers
- Active development and support
- Suitable for Sim-to-Real transfer

**Not Recommended:**
- **Unitree H1:** Too expensive ($90k+)

### When to Choose
- Capstone deployment to real humanoid is goal
- Premium budget available
- Advanced research objectives
- Sim-to-Real validation critical

## Robot Lab Architecture

### Complete System Integration

| Component | Hardware | Function |
|-----------|----------|----------|
| **Sim Rig** | PC with RTX 4080 + Ubuntu 22.04 | Runs Isaac Sim, Gazebo, Unity, trains LLM/VLA models |
| **Edge Brain** | Jetson Orin Nano | Runs inference stack, students deploy code here |
| **Sensors** | RealSense Camera + Lidar | Connected to Jetson, feeds real-world data to AI |
| **Actuator** | Unitree Go2 or G1 (Shared) | Receives motor commands from Jetson |

## Investment Considerations

### On-Premise Lab (High CapEx)
**Advantages:**
- One-time purchase
- No ongoing costs
- Full control and ownership
- No latency issues

**Disadvantages:**
- High upfront cost
- Maintenance responsibility
- Space requirements

### Shared Robot Strategy
- Purchase 1-2 robots per lab
- Students develop on simulation
- Schedule robot time for physical testing
- Reduces per-student cost significantly

### Cost Comparison

| Option | Initial Cost | Ongoing Cost | Students Supported |
|--------|-------------|--------------|-------------------|
| **Proxy (Go2)** | $3,000 | Minimal | 10-15 |
| **Mini Humanoid (G1)** | $16,000 | Minimal | 10-15 |
| **Budget (TonyPi)** | $600 | Minimal | 5-8 (limited) |
| **Premium Lab (G1 Full)** | $16,000+ | Minimal | 8-12 |

## Deployment Workflow

### Typical Student Experience

1. **Simulation Phase** (Workstation)
   - Develop and test in Isaac Sim
   - Train models in virtual environment
   - Debug algorithms safely

2. **Edge Deployment** (Jetson Kit)
   - Deploy trained models
   - Test with real sensors
   - Validate inference performance

3. **Physical Testing** (Robot)
   - Schedule robot lab time
   - Deploy to actual hardware
   - Validate Sim-to-Real transfer

## Recommendation Summary

**For Most Labs:** Start with Option A (Unitree Go2)
- Best value for learning objectives
- Principles transfer to humanoids
- Can expand to humanoid later

**For Humanoid-Focused Programs:** Option B or C (Unitree G1)
- Direct humanoid experience
- Scalable based on budget
- Industry-relevant platform