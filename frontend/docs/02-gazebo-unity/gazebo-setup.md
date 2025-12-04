---
id: gazebo-setup
title: Gazebo Setup & First Simulation
sidebar_label: Gazebo Setup
sidebar_position: 3
description: Install Gazebo, create worlds, and spawn your first robot
keywords: [gazebo-installation, sdf, world-files, robot-spawning]
---

import PlatformNote from '@site/src/components/PlatformNote';

# Gazebo Setup & First Simulation

## Installation

<PlatformNote
  linux="Recommended: Install via apt on Ubuntu 22.04"
  macos="Use Docker with Ubuntu 22.04 image"
  wsl2="Install in WSL2 Ubuntu environment. GPU acceleration supported via WSLg."
/>

### Install Gazebo Classic (ROS 2 Humble)

```bash
# Update package list
sudo apt update

# Install Gazebo with ROS 2 integration
sudo apt install ros-humble-gazebo-ros-pkgs -y

# Install Gazebo Classic 11
sudo apt install gazebo -y

# Verify installation
gazebo --version
```

**Expected output**: `Gazebo multi-robot simulator, version 11.14.0`

---

## Your First Simulation

### Launch Empty World

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch Gazebo
gazebo
```

**You should see**: 3D viewport with grid ground plane.

### Basic Controls

- **Rotate camera**: Left-click + drag
- **Pan camera**: Middle-click + drag
- **Zoom**: Scroll wheel
- **Move objects**: Click object → Translation/Rotation mode (toolbar)

---

## World Files (SDF Format)

Gazebo uses **SDF** (Simulation Description Format) for world descriptions.

### Minimal World Example

Create `empty_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

  </world>
</sdf>
```

### Launch Custom World

```bash
gazebo empty_world.sdf
```

---

## Adding Models

### Insert Models from GUI

1. Click **Insert** tab (left sidebar)
2. Browse models (Ambulance, BookShelf, etc.)
3. Click model → Click in world to place
4. Save world: **File → Save World As**

### Programmatically Add Models

```xml
<!-- Add a box obstacle -->
<model name="box">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

---

## Spawning Robots

### Method 1: ROS 2 Launch File

```python
# spawn_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),

        # Spawn robot from URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', 'robot.urdf'],
            output='screen'
        ),
    ])
```

### Method 2: Command Line

```bash
# Start Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot (in another terminal)
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf
```

---

## ROS 2 Integration

### Launch Gazebo with ROS 2

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### Check ROS 2 Topics

```bash
ros2 topic list
```

**Output**:
```
/clock
/gazebo/link_states
/gazebo/model_states
/tf
```

---

## Hands-On Lab: Warehouse World

**Goal**: Create a warehouse environment with obstacles.

### Requirements

1. Ground plane
2. 4 walls (boxes)
3. 10 obstacles (boxes, cylinders)
4. Lighting (sun)

### Starter Code

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse">

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- TODO: Add walls -->
    <!-- TODO: Add obstacles -->

  </world>
</sdf>
```

---

## Key Takeaways

✅ **Gazebo Classic 11** for ROS 2 Humble
✅ **SDF format** for world descriptions
✅ **spawn_entity.py** to load robots
✅ **ROS 2 integration** automatic via `gazebo_ros`

---

## Next Steps

Learn Unity as an alternative in **[Chapter 3: Unity Simulation](/docs/gazebo-unity/unity-sim)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/gazebo-unity/unity-sim">
      Next: Unity Simulation →
    </a>
  </div>
</div>
