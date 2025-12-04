---
id: isaac-setup
title: Isaac Sim Installation & Setup
sidebar_label: Isaac Sim Setup
sidebar_position: 2
description: Install NVIDIA Isaac Sim via Omniverse and run your first simulation
keywords: [isaac-sim-installation, nvidia-omniverse, rtx-gpu, ros2-bridge]
---

import PlatformNote from '@site/src/components/PlatformNote';

# Isaac Sim Installation & Setup

## System Requirements

<PlatformNote
  linux="Ubuntu 22.04 LTS recommended (native installation)"
  macos="macOS not supported. Use cloud (AWS g5.xlarge) or dual-boot Linux."
  wsl2="WSL2 not officially supported. Use native Ubuntu or cloud instance."
/>

### Hardware Requirements

- **GPU**: NVIDIA RTX 2060+ (6GB+ VRAM) — **RTX required for ray tracing**
- **CPU**: 8+ cores recommended
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free (SSD recommended)

---

## Installation Steps

### Step 1: Install NVIDIA Drivers

```bash
# Check if NVIDIA drivers installed
nvidia-smi

# If not installed:
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Expected output** from `nvidia-smi`:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
```

---

### Step 2: Install Omniverse Launcher

1. Visit [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Click **Download** → **Omniverse Launcher for Linux**
3. Extract and run:

```bash
cd ~/Downloads
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

---

### Step 3: Install Isaac Sim

1. **In Omniverse Launcher**, go to **Exchange** tab
2. Search for **Isaac Sim**
3. Click **Install** (version 2023.1.1 or later)
4. Wait ~20 minutes (8GB download)

---

### Step 4: Launch Isaac Sim

1. **In Omniverse Launcher**, go to **Library** tab
2. Find **Isaac Sim** → Click **Launch**
3. First launch takes 5-10 minutes (shader compilation)

**You should see**: Isaac Sim viewport with welcome screen.

---

## Your First Simulation

### Load Example Scene

1. **File** → **Open**
2. Navigate to: `Isaac/Samples/ROS2/Scenario/simple_room.usd`
3. Click **Open**

**You should see**: Warehouse environment with Carter robot.

---

### Run Simulation

1. Press **Play** button (toolbar, left side)
2. Robot should start moving
3. Press **Stop** to pause

---

## ROS 2 Integration

Isaac Sim communicates with ROS 2 via **ROS Bridge**.

### Enable ROS 2 Bridge

1. **Window** → **Extensions**
2. Search: `omni.isaac.ros2_bridge`
3. Enable extension

### Launch ROS 2 Nodes

**Terminal 1** (Isaac Sim ROS 2 bridge):
```bash
source /opt/ros/humble/setup.bash

# Bridge is auto-enabled when ROS 2 extension is active
# Check topics:
ros2 topic list
```

**Expected topics**:
```
/camera/color/image_raw
/camera/depth/image_raw
/scan
/cmd_vel
/tf
```

---

## Creating Your First Scene

### Step 1: New Stage

1. **File** → **New**
2. Save as: `my_first_scene.usd`

### Step 2: Add Ground Plane

1. **Create** → **Physics** → **Ground Plane**
2. Adjust size: Select ground plane → **Property** panel → **Size**: 50×50

### Step 3: Add Robot

1. **Create** → **Isaac** → **Robots** → **Carter**
2. Position at origin (0, 0, 0)

### Step 4: Add Camera

1. Select Carter robot
2. **Create** → **Camera**
3. Parent to robot: Drag camera onto robot in hierarchy

### Step 5: Add LiDAR

1. **Create** → **Isaac** → **Sensors** → **Rotating Lidar**
2. Parent to robot
3. Position: (0.3, 0, 0.2) relative to robot

---

## Domain Randomization (Preview)

Enable Replicator for synthetic data:

```python
import omni.replicator.core as rep

# Randomize lighting
with rep.trigger.on_frame():
    rep.randomizer.light_intensity(min=0.5, max=2.0)

# Randomize textures
materials = ['wood', 'concrete', 'marble']
rep.randomizer.materials(materials)
```

---

## Hands-On Lab: Warehouse Scene

**Goal**: Create a warehouse with obstacles and a mobile robot.

### Requirements

1. Ground plane (50×50m)
2. 4 walls (boxes)
3. 10 obstacles (random props from Isaac library)
4. Carter robot with LiDAR
5. Enable ROS 2 bridge

### Steps

1. Create new stage
2. Add ground plane
3. Add walls: **Create** → **Shape** → **Cube** (scale to 10×0.2×2)
4. Add obstacles: **Create** → **Isaac** → **Environments** → **Props**
5. Add robot: **Create** → **Isaac** → **Robots** → **Carter**
6. Enable ROS 2 extension
7. Press **Play**

---

## Troubleshooting

### Issue: GPU not detected

```bash
# Check GPU
nvidia-smi

# Update drivers
sudo ubuntu-drivers autoinstall
```

### Issue: Slow performance

1. Lower resolution: **Edit** → **Preferences** → **Rendering** → **Resolution**: 720p
2. Disable ray tracing: **Render Settings** → **Ray Tracing**: Off
3. Use lower-quality materials

### Issue: ROS 2 topics not visible

```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Ensure same domain ID in Isaac Sim and ROS 2
export ROS_DOMAIN_ID=0
```

---

## Key Takeaways

✅ **Isaac Sim** requires RTX GPU (ray tracing)
✅ **Omniverse Launcher** manages installation
✅ **ROS 2 Bridge** extension for ROS integration
✅ **USD format** for scene descriptions

---

## Next Steps

Learn synthetic data generation in **[Chapter 2: Synthetic Data](/docs/isaac/synthetic-data)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/isaac/synthetic-data">
      Next: Synthetic Data Generation →
    </a>
  </div>
</div>
