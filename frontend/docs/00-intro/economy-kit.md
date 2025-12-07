---
id: economy-jetson-kit
title: "Economy Jetson Kit: Complete Hardware Guide ($700)"
sidebar_label: "Economy Kit"
sidebar_position: 1
description: "Complete breakdown of the $700 Economy Jetson Kit - Jetson Orin Nano ($249), RealSense D435i ($349), and accessories. Includes cloud cost comparison and break-even analysis."
keywords: [jetson orin nano, realsense d435i, economy kit, hardware, edge ai, $700, pricing, aws comparison]
---

# Economy Jetson Kit: Complete Hardware Guide

The **Economy Jetson Kit** is the default hardware path for this course, providing everything you need to deploy Physical AI models to real robots for approximately **$700**.

---

## Kit Overview

| Component | Price | Purpose | Last Verified |
|-----------|-------|---------|---------------|
| **Jetson Orin Nano Dev Kit** | **$249** | Edge AI inference (&lt;10ms latency) | 2025-12-06 |
| **Intel RealSense D435i** | **$349** | RGB-D perception + IMU | 2025-12-06 |
| **65W USB-C Power Supply** | $30 | Power Jetson Orin Nano | 2025-12-06 |
| **USB-C Cable + Ethernet** | $20 | Connectivity (6ft cables) | 2025-12-06 |
| **128GB microSD Card (U3)** | $20 | OS storage + models | 2025-12-06 |
| **Protective Acrylic Case** | $30 | Jetson housing + cooling | 2025-12-06 |
| **TOTAL** | **$698** | *Rounded to $700* | - |

**Purchase Links**:
- Jetson Orin Nano: [NVIDIA Store](https://store.nvidia.com/en-us/jetson/store)
- RealSense D435i: [Intel RealSense Store](https://www.intelrealsense.com/depth-camera-d435i/)
- Accessories: [Amazon](https://amazon.com) (search: "65W USB-C power supply", "128GB microSD U3", "Jetson Orin case")

**Regional Pricing Note**: Prices verified for US market (2025-12-06). International buyers should check local vendors and import duties.

---

## Component Details

### 1. Jetson Orin Nano Developer Kit ($249)

**Why This Component?**

The Jetson Orin Nano is NVIDIA's entry-level edge AI platform that brings datacenter-class AI performance to a $249 device the size of a credit card. It's the **sweet spot** for Physical AI: powerful enough for VLA model inference, affordable enough for students, and power-efficient enough for battery-powered robots.

**Technical Specifications**:

| Spec | Value | Comparison |
|------|-------|------------|
| **AI Performance** | 10 TOPS (INT8) | ~10x Raspberry Pi 4 |
| **GPU** | 512-core NVIDIA Ampere | Same architecture as RTX 3000 series |
| **CPU** | 6-core Arm Cortex-A78AE | 64-bit ARMv8.2 |
| **Memory** | 8GB 128-bit LPDDR5 | 102.4 GB/s bandwidth |
| **Storage** | microSD slot | Supports up to 2TB cards |
| **Power** | 7W - 15W (configurable) | Battery-powered robots possible |
| **Connectivity** | Gigabit Ethernet, 4x USB 3.2, DisplayPort | Full dev kit I/O |
| **Size** | 100mm x 79mm x 31mm | Credit card footprint |

**What Can It Run?**:
- **VLA Models**: Octo (7B params), RT-1 architecture at &lt;50ms inference
- **Object Detection**: YOLOv8, Faster R-CNN real-time (30 FPS)
- **Navigation**: ROS 2 Nav2 stack with costmap updates at 10 Hz
- **Manipulation**: MoveIt2 trajectory planning for 7-DOF arms

**Power Modes**:
```bash
# View current power mode
sudo nvpmodel -q

# Set to maximum performance (15W)
sudo nvpmodel -m 0

# Set to power-efficient mode (7W, battery-powered)
sudo nvpmodel -m 1
```

**When to Upgrade**:
- **Jetson Orin NX** ($399, 70 TOPS): Need more performance for larger VLA models (>7B params)
- **Jetson AGX Orin** ($1,999, 275 TOPS): Production deployment, multi-robot systems
- Orin Nano is sufficient for 95% of this course's projects

---

### 2. Intel RealSense D435i ($349)

**Why This Component?**:

The RealSense D435i provides **RGB-D perception** (color + depth) plus an **IMU** (Inertial Measurement Unit) in a single device. This combination is essential for Physical AI:
- **RGB**: Visual input for VLA models ("pick up the red cup")
- **Depth**: 3D spatial understanding (object distances, obstacle avoidance)
- **IMU**: Motion tracking (robot orientation, acceleration)

**Technical Specifications**:

| Spec | Value | Use Case |
|------|-------|----------|
| **Depth Resolution** | 1280 × 720 | Up to 90 FPS |
| **RGB Resolution** | 1920 × 1080 | Up to 30 FPS |
| **Depth Range** | 0.3m - 3m | Indoor navigation/manipulation |
| **Depth Technology** | Active IR stereo | Works in low light |
| **IMU** | BMI055 6-axis | Accel + gyro at 200 Hz |
| **Field of View** | 87° × 58° (depth) | Wide-angle for navigation |
| **Interface** | USB 3.1 Type-C | Plug-and-play with Jetson |
| **Size** | 90mm × 25mm × 25mm | Compact for robot mounting |
| **Power** | &lt;1.5W | USB-powered, no external supply |

**What You Get**:
- **Point Clouds**: 3D representation of environment (for mapping, SLAM)
- **Aligned RGB-D**: Color and depth registered (same pixel corresponds to same 3D point)
- **IMU Data**: Orientation, angular velocity, linear acceleration (for odometry, stabilization)

**ROS 2 Integration**:
```bash
# Install RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera

# Launch camera node
ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    enable_imu:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```

**Alternatives (If D435i Unavailable)**:

| Camera | Price | Pros | Cons |
|--------|-------|------|------|
| **RealSense D455** | $329 | Longer range (6m vs 3m) | Slightly heavier |
| **OAK-D** (Luxonis) | $199 | Onboard AI processing | Smaller community |
| **Kinect Azure** (used) | ~$200 | Higher resolution (1024×1024 depth) | Discontinued (eBay only) |

**Recommendation**: If D435i out of stock, choose **RealSense D455** ($329, same SDK, longer range). Kit total becomes $679.

---

### 3. Accessories ($100)

**65W USB-C Power Supply ($30)**:
- **Specification**: 65W USB Power Delivery (PD) 3.0
- **Why needed**: Jetson Orin Nano requires 5V/6A (30W) minimum; 65W provides headroom for peripherals
- **Cable included**: 6ft USB-C cable
- **Search term**: "65W USB-C PD power supply"

**Additional Cables ($20)**:
- **USB-C to USB-C** (6ft): Connect RealSense to Jetson
- **Cat6 Ethernet cable** (6ft): Network connectivity (for SSH, ROS 2 networking)
- **Search terms**: "USB-C cable 6ft", "Cat6 Ethernet patch cable"

**128GB microSD Card ($20)**:
- **Specification**: Class 10, UHS-I (U3), 100MB/s read
- **Why needed**: Jetson OS (JetPack 6.0) + ROS 2 + models = ~50GB; 128GB provides comfortable headroom
- **Brands**: SanDisk Extreme, Samsung EVO Select
- **Search term**: "128GB microSD U3 Class 10"

**Protective Case ($30)**:
- **Material**: Acrylic or aluminum
- **Features**: Ventilation holes (Jetson runs warm), GPIO access, mounting points
- **Optional**: Add 40mm cooling fan ($10) for sustained workloads
- **Search term**: "Jetson Orin Nano acrylic case"

---

## Cloud Alternative: AWS g5.xlarge

**Specifications**:
- **GPU**: NVIDIA A10G (24GB VRAM, Ampere architecture)
- **CPU**: 4 vCPUs (AMD EPYC 7R32)
- **Memory**: 16GB RAM
- **Storage**: EBS volumes (pay-per-GB)
- **Cost**: $1.006/hour (us-east-1, on-demand pricing)

**Use Cases**:
- ✅ **Training VLA models**: 24GB VRAM handles larger models (>7B params)
- ✅ **Isaac Sim**: RTX-accelerated physics simulation
- ✅ **Parallel training**: Spin up multiple instances for hyperparameter sweeps
- ❌ **Real robot control**: 50-200ms+ latency (UNSAFE for physical robots)

**Cost Calculation** (5 hours/week for 10-week quarter):
```
Hourly rate: $1.006/hr
Weekly usage: 5 hours
Quarterly usage: 5 hrs/week × 10 weeks = 50 hours
Quarterly cost: $1.006 × 50 = $50.30/quarter
```

**Annual Cost** (3 quarters):
```
3 quarters × $50.30 = $150.90/year
```

**Break-Even Analysis**:
```
Jetson upfront: $700
Cloud per quarter: $50.30

Break-even: $700 ÷ $50.30 = 13.9 quarters ≈ 3.5 years
```

**Decision Matrix**:

| Scenario | Recommended Option | Rationale |
|----------|-------------------|-----------|
| **Single quarter course** | Cloud ($50) | Cheaper for short-term use |
| **Multi-year learning** | Jetson ($700) | Own hardware forever, break-even at 3.5 years |
| **Real robot deployment** | Jetson (required) | &lt;10ms inference, cloud has 50-200ms+ latency |
| **Large VLA training** | Cloud (24GB VRAM) | Orin Nano has 8GB (sufficient for inference, limited for training) |
| **Battery-powered robot** | Jetson (7-15W) | Cloud requires constant internet |

---

## Total Cost Comparison (1 Year)

| Option | Upfront | Q1 | Q2 | Q3 | **Total (1 Year)** |
|--------|---------|----|----|----|--------------------|
| **Jetson Orin Nano Kit** | $700 | $0 | $0 | $0 | **$700** |
| **AWS g5.xlarge** | $0 | $50 | $50 | $50 | **$151** |
| **Both (Hybrid)** | $700 | $50 | $50 | $50 | **$851** |

**Hybrid Approach** (Recommended for Serious Learners):
1. **Cloud for training**: Use g5.xlarge for VLA training (24GB VRAM, fast iterations)
2. **Jetson for deployment**: Deploy trained models to Jetson for edge inference (&lt;10ms)
3. **Total cost**: $700 (Jetson) + $151/year (cloud) = $851/year

**Why hybrid?**:
- Training on Jetson's 8GB RAM is possible but slow (limited batch size)
- Cloud training is 3-5x faster with 24GB VRAM
- Deployment to Jetson is **mandatory** for real robots (latency requirements)

---

## Setup Guide (Jetson Orin Nano)

### Step 1: Flash JetPack 6.0

1. Download **NVIDIA SDK Manager**: https://developer.nvidia.com/sdk-manager
2. Flash JetPack 6.0 to 128GB microSD card
3. Insert microSD into Jetson, power on
4. Follow on-screen setup (create user, connect WiFi)

**Or use pre-flashed image**:
```bash
# Download JetPack 6.0 image
wget https://developer.nvidia.com/downloads/jetpack-60-image

# Flash to microSD (on Linux host)
sudo dd if=jetpack-6.0.img of=/dev/sdX bs=4M status=progress
```

### Step 2: Install ROS 2 Humble

```bash
# Add ROS 2 repository (on Jetson)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y

# Source environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install RealSense SDK

```bash
# Install RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera

# Test camera (plug in RealSense D435i via USB-C)
ros2 launch realsense2_camera rs_launch.py
```

**Verify depth stream**:
```bash
# In another terminal
ros2 topic hz /camera/depth/image_rect_raw
# Should show ~30 Hz
```

### Step 4: Install Deep Learning Libraries

```bash
# ONNX Runtime (for VLA model inference)
pip3 install onnxruntime-gpu

# PyTorch (ARM64 build for Jetson)
wget https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.1.0-cp310-cp310-linux_aarch64.whl
pip3 install torch-2.1.0-cp310-cp310-linux_aarch64.whl

# Verify GPU access
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
# Should print: CUDA available: True
```

---

## FAQ

**Q: Can I use Raspberry Pi 4 instead of Jetson?**

A: **No**. Raspberry Pi 4 (2 TOPS) is 5x slower than Jetson Orin Nano (10 TOPS) and lacks GPU acceleration for deep learning. VLA model inference would be >500ms (too slow for real-time control).

**Q: Do I need the Developer Kit or can I use the module?**

A: **Developer Kit required** for this course. The module (Jetson Orin Nano without carrier board) requires custom PCB design. Dev Kit includes carrier board with USB, Ethernet, GPIO, display outputs.

**Q: Can I use older Jetson (Nano, Xavier NX)?**

A: **Not recommended**. Jetson Nano (original) has only 472 GFLOPS (vs. Orin Nano's 10 TOPS). Xavier NX (2019) is EOL (end-of-life). Orin Nano is the current-gen entry-level device.

**Q: What if I already have an RTX 3060+ GPU?**

A: **Great!** Use your RTX GPU for Modules 1-3 (training, Isaac Sim). Still recommend buying Jetson for Module 4 (edge deployment is core learning objective).

**Q: Can I use a different depth camera (Kinect, OAK-D)?**

A: **Yes, but RealSense recommended**. RealSense has the largest ROS 2 community and best documentation. Other cameras work but require more setup.

**Q: Will prices change?**

A: **Possibly**. Prices verified 2025-12-06. NVIDIA occasionally runs promotions. Check vendor sites for current deals. This guide will be updated quarterly.

---

## Summary

The **Economy Jetson Kit ($700)** provides:
- ✅ Edge AI inference (&lt;10ms latency for real-time control)
- ✅ RGB-D perception + IMU (complete sensor suite)
- ✅ One-time purchase (no recurring cloud costs)
- ✅ Portable (7-15W power, battery-powered robots possible)
- ✅ Sufficient for 95% of course projects

**Cloud alternative ($151/year)**: Use for training, not deployment (latency trap).

**Hybrid approach ($851/year)**: Train in cloud (faster), deploy to Jetson (safer).

---

## Next Steps

1. **Order hardware**: Purchase from links above (2-5 day shipping typically)
2. **Setup Jetson**: Flash JetPack 6.0, install ROS 2 Humble
3. **Test RealSense**: Verify depth streams at 30 Hz
4. **Start Module 1**: [ROS 2 Fundamentals](/docs/01-ros2/index) (works without Jetson)

---

<!-- Generated by @hardware-economist on December 6, 2025 -->
