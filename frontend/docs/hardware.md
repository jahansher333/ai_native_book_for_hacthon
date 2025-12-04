---
id: hardware
title: Hardware Requirements
sidebar_label: Hardware
sidebar_position: 3
description: Choose your hardware setup - Economy Jetson Kit ($700), Digital Twin Workstation, Robot Lab, or Cloud-Native
keywords: [jetson-orin-nano, realsense-d435i, hardware-setup, edge-computing, aws-gpu]
---

import HardwareTable from '@site/src/components/HardwareTable';

# Hardware Requirements

Choose the hardware setup that fits your learning goals and budget. This page provides **4 options** ranging from **$205/quarter (cloud-only)** to **$700 (edge AI kit)** to **$16,000+ (research robots)**.

---

## 🌟 Recommended Path: Economy Jetson Kit ($700)

### Why This Setup?

The **Economy Jetson Kit** is our **recommended path for students** because it provides:

✅ **Complete Edge AI Stack**: Train models in the cloud/workstation, deploy to Jetson for real-time inference
✅ **Real-World Skills**: Learn the industry-standard sim-to-real workflow
✅ **Affordable**: ~$700 one-time cost (no recurring cloud fees)
✅ **Portable**: Jetson Orin Nano runs on 7-15W, powered by USB-C
✅ **Future-Proof**: Use it for capstone projects, research, and personal robots

### What's Included

<HardwareTable variant="economy" lastUpdated="2025-12-04" />

### When to Use This Setup

- ✅ **Deploying to real robots** (quadrupeds, humanoids, drones)
- ✅ **Edge inference** (&lt;10ms latency requirement)
- ✅ **Sim-to-real projects** (train in cloud → deploy to Jetson)
- ✅ **Capstone project** (autonomous humanoid with voice commands)
- ✅ **Post-course projects** (you own the hardware)

### Assembly & Setup

1. **Jetson Orin Nano**: Flash Ubuntu 22.04 + JetPack 6.0 (includes CUDA, TensorRT)
2. **RealSense D435i**: Connect via USB 3.0, install `librealsense2` drivers
3. **ReSpeaker**: Connect via USB 2.0, install Python SDK for voice commands
4. **Power**: 65W USB-C power supply (included in Jetson kit or purchase separately)

**Total Setup Time**: ~2 hours for first-time setup.

---

## 💻 Option 2: Digital Twin Workstation ($1,500-2,500)

### Why This Setup?

Perfect for **simulation-only learners** who want to:
- Train models locally without cloud costs
- Run NVIDIA Isaac Sim, Gazebo, and Unity simulations
- Generate synthetic training data with domain randomization

### Recommended Specifications

<HardwareTable variant="workstation" lastUpdated="2025-12-04" />

### When to Use This Setup

- ✅ **Isaac Sim simulation** (requires RTX GPU for ray tracing)
- ✅ **Training models locally** (no AWS costs)
- ✅ **Synthetic data generation** (domain randomization)
- ✅ **Gazebo/Unity development** (high FPS, no cloud lag)

### Operating System

- **Recommended**: Ubuntu 22.04 LTS (best ROS 2 support)
- **Alternative**: Windows 11 + WSL2 (Ubuntu 22.04) for dual-boot convenience

### Notes

- **One-time investment**: No recurring cloud costs
- **Desktop or laptop**: Desktop preferred for better cooling/upgradability
- **GPU VRAM**: 12GB minimum for Isaac Sim; 24GB recommended for large scenes

---

## 🤖 Option 3: Robot Lab Hardware (Optional, $1,800-$16,000)

### Why This Setup?

For **advanced learners and research labs** testing algorithms on real physical platforms.

### Available Platforms

<HardwareTable variant="robot-lab" lastUpdated="2025-12-04" />

### Unitree Go2 Quadruped ($1,800-3,000)

- **Use Cases**: Navigation, SLAM, outdoor robotics
- **Specifications**: 12 motors, 25kg payload, 2m/s max speed
- **SDK**: ROS 2 support, Python/C++ APIs
- **Best For**: Weeks 7-9 (Nav2, VSLAM), outdoor autonomous navigation

### Unitree G1 Humanoid ($16,000)

- **Use Cases**: Manipulation, human-robot interaction, VLA research
- **Specifications**: 43 degrees of freedom (DoF), force feedback, dexterous hands
- **SDK**: ROS 2 support, Python/C++ APIs, pre-trained VLA models
- **Best For**: Week 13 capstone (autonomous humanoid with voice commands)

### When to Use This Setup

- ✅ **Research labs** with funding for physical platforms
- ✅ **Advanced students** working on thesis projects
- ✅ **Industry teams** building commercial robot products
- ✅ **Testing sim-to-real transfer** on actual hardware

### Notes

- **⚠️ Not Required**: You can complete the entire course with simulation + Jetson kit
- **⚠️ High Cost**: $1,800-$16,000 per robot
- **⚠️ Maintenance**: Requires spare parts, repairs, and storage space

---

## ☁️ Option 4: Cloud-Native Alternative ($205/quarter)

### Why This Setup?

Perfect for learners who:
- Don't want upfront hardware costs
- Want to try Isaac Sim before committing to a workstation
- Have reliable high-speed internet (25+ Mbps)

### Cloud Instance Pricing

<HardwareTable variant="cloud-native" lastUpdated="2025-12-04" />

### Cost Breakdown

- **Hourly Rate**: $1.006/hour (AWS g5.xlarge with A10G GPU, 24GB VRAM)
- **Weekly Usage**: 5 hours/week (labs, assignments)
- **10-Week Quarter**: 5 hrs × 10 weeks = 50 hours
- **Total Cost**: **~$205 for the quarter**

### When to Use This Setup

- ✅ **Isaac Sim simulation** (A10G GPU supports ray tracing)
- ✅ **Training models in the cloud**
- ✅ **No local GPU available**
- ✅ **Temporary access** (testing before buying workstation)

### Setup Instructions

1. **Sign up for AWS**: Create free-tier account
2. **Launch g5.xlarge**: EC2 → Launch Instance → g5.xlarge (A10G GPU)
3. **Install software**:
   - Ubuntu 22.04 LTS AMI
   - NVIDIA drivers + CUDA toolkit
   - ROS 2 Humble
   - Isaac Sim (via Omniverse)
4. **Connect via SSH**: Use X11 forwarding or VNC for GUI access

### Notes

- **⚠️ Recurring Cost**: $1.006/hour when instance is running (stop instance when not in use)
- **⚠️ Network Latency**: 50-200ms+ latency for cloud-robot control (**UNSAFE for real robots**)
- **⚠️ Data Transfer**: Uploading/downloading large datasets costs extra (use S3 for storage)

### Cost Optimization Tips

- **Stop instances** when not in use (not just pause—fully stop)
- **Use spot instances** (up to 70% cheaper, may be interrupted)
- **Set billing alerts** ($50, $100, $150) to avoid overspending
- **Delete snapshots** and unused volumes after course ends

---

## 🆚 Comparison: Which Setup Is Right for You?

| Feature | Economy Jetson Kit | Workstation | Robot Lab | Cloud-Native |
|---------|-------------------|-------------|-----------|--------------|
| **Cost** | ~$700 (one-time) | $1,500-2,500 (one-time) | $1,800-16,000 (one-time) | ~$205/quarter (recurring) |
| **Edge Deployment** | ✅ Yes (Jetson Orin Nano) | ❌ No | ✅ Yes (robot-integrated) | ❌ No |
| **Isaac Sim** | ❌ No (use cloud for training) | ✅ Yes (RTX GPU required) | ⚠️ Optional | ✅ Yes (A10G GPU) |
| **Sim-to-Real** | ✅ Full workflow | ⚠️ Training only | ✅ Full workflow + robot | ⚠️ Training only |
| **Portability** | ✅ Very portable (7-15W) | ❌ Desktop-bound | ❌ Lab-bound | ✅ Access anywhere |
| **Real Robots** | ⚠️ DIY integration | ❌ Simulation only | ✅ Pre-integrated | ❌ Simulation only |
| **Best For** | Students, edge AI projects | Simulation researchers | Advanced research labs | Budget-conscious, cloud-first |

---

## 🎯 Recommended Combinations

### For Students (Most Popular)

**Economy Jetson Kit ($700) + AWS g5.xlarge (as needed)**

- Train models in cloud (Isaac Sim, large datasets)
- Deploy to Jetson for real-time inference
- Total cost: $700 + ~$50-100 cloud usage for training

### For Researchers

**Workstation ($1,500-2,500) + Economy Jetson Kit ($700)**

- Train locally (no cloud costs, faster iteration)
- Deploy to Jetson for field testing
- Total cost: $2,200-3,200 (one-time)

### For Institutions

**Workstation ($1,500-2,500) + Robot Lab ($1,800-16,000)**

- Full research stack (simulation + real robots)
- Suitable for multi-year projects and Ph.D. students
- Total cost: $3,300-18,500

---

## 🛒 Where to Buy

### Jetson Orin Nano Developer Kit ($249)
- **NVIDIA Store**: [store.nvidia.com](https://store.nvidia.com/)
- **Amazon**: Search "Jetson Orin Nano Developer Kit"
- **Seeed Studio**: [seeedstudio.com](https://www.seeedstudio.com/)

### Intel RealSense D435i ($349)
- **Intel Store**: [intel.com/realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)
- **Amazon**: Search "Intel RealSense D435i"

### ReSpeaker Mic Array v2.0 ($69)
- **Seeed Studio**: [seeedstudio.com](https://www.seeedstudio.com/)
- **Amazon**: Search "ReSpeaker Mic Array v2.0"

### Power Supply + Cables
- **Amazon**: 65W USB-C PD charger + USB 3.0 cables + 128GB microSD card

---

## 📦 What's Next?

Once you've chosen your hardware setup:

1. **Review [Course Overview](/docs/course-overview)** for 13-week breakdown
2. **Start [Module 1: ROS 2](/docs/ros2)** (no hardware required yet)
3. **Week 5**: Set up hardware for hands-on labs (Modules 2-4)

---

## 💡 Still Undecided?

**Start with simulation-only** (Modules 1-2) while you decide:
- Use your existing laptop/desktop
- Install ROS 2 Humble on Ubuntu 22.04 (or WSL2)
- Run Gazebo simulations (lightweight, no GPU required)

By **Week 5**, you'll understand your hardware needs better and can invest accordingly.

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>🚀 Ready to Get Started?</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    No hardware? No problem! Start learning ROS 2 with just your laptop.
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/course-overview"
      style={{marginRight: '1rem'}}
    >
      View Course Structure →
    </a>
    <a
      className="button button--secondary button--lg"
      href="/docs/ros2"
    >
      Start Module 1 (No Hardware Needed) →
    </a>
  </div>
</div>
