# Physical AI Edge Kit

## Overview
The Edge Kit represents the "nervous system" of the Physical AI setup. Students learn to set up this brain on a desk before deploying to actual robots. This kit covers Module 3 (Isaac ROS) and Module 4 (VLA).

## The Economy Jetson Student Kit (~$700)

### The Brain
**Component:** NVIDIA Jetson Orin Nano Super Dev Kit (8GB)
- **Price:** $249 (new official MSRP, dropped from ~$499)
- **Performance:** 40 TOPS
- **Role:** Industry standard for embodied AI
- **Purpose:** Students deploy ROS 2 nodes here to understand resource constraints vs. powerful workstations

**Alternative:** Orin NX (16GB) for more demanding applications

### The Eyes (Vision)
**Component:** Intel RealSense D435i
- **Price:** $349
- **Features:** 
  - RGB (Color) data
  - Depth (Distance) data
  - Built-in IMU
- **Role:** Essential for VSLAM and Perception modules
- **Important:** Get the D435**i** model (includes IMU), not the D435

**Alternative:** Intel RealSense D455 for extended range

### The Inner Ear (Balance)
**Component:** Generic USB IMU (BNO055)
- **Note:** Often built into RealSense D435i or Jetson boards
- **Purpose:** Separate module helps teach IMU calibration
- **Use Case:** Inertial measurement for orientation and motion tracking

### The Ears (Voice Interface)
**Component:** ReSpeaker USB Mic Array v2.0
- **Price:** $69
- **Features:** Far-field microphone for voice commands
- **Purpose:** "Voice-to-Action" Whisper integration in Module 4

### Connectivity
**Component:** Wi-Fi Module
- **Price:** $0 (included in new "Super" kit)
- **Note:** Pre-installed in Jetson Orin Nano Super Dev Kit

### Storage & Miscellaneous
**Components:**
- High-endurance microSD card (128GB)
- Jumper wires
- **Price:** ~$30

## Total Kit Cost
**Complete Edge Kit:** ~$700

## Architecture Role
The Edge Kit serves as the **Edge Brain** in the overall system:
- Runs the "Inference" stack
- Students deploy trained models here
- Connects sensors to feed real-world data to AI
- Sends motor commands to actuators/robots

## Deployment Strategy

### The Latency Trap
Controlling real robots from cloud instances is dangerous due to latency.

**Solution:**
1. Train models in the Cloud (workstation/cloud instance)
2. Download trained model weights
3. Flash models to local Jetson kit
4. Run inference locally on edge device

### Why Edge Computing Matters
- Low latency for real-time control
- Understanding resource constraints
- Industry-standard deployment pipeline
- Sim-to-Real transfer validation

## Compatibility Notes

### Not Compatible
**Raspberry Pi-based robots (e.g., Hiwonder TonyPi Pro):**
- Cannot run NVIDIA Isaac ROS efficiently
- Use only for basic kinematics (walking)
- Jetson kit handles all AI workloads

### Integration Points
The Edge Kit connects to:
- Workstation (for model deployment)
- Sensors (RealSense, IMU, Microphone)
- Robot actuators (via ROS 2 messages)
- Cloud services (for model updates)