---
id: index
title: Module 1 - ROS 2 Fundamentals
sidebar_label: Overview
sidebar_position: 1
description: Master the robotic nervous system - nodes, topics, services, actions, and URDF
keywords: [ros2, rclpy, nodes, topics, services, actions, urdf, tf2]
---

# Module 1: Robotic Nervous System (ROS 2)

## Introduction

**ROS 2** (Robot Operating System 2) is the industry-standard middleware for building distributed robot systems. Think of it as the "nervous system" that allows different robot components (sensors, motors, AI models) to communicate seamlessly.

In this module, you'll learn the fundamental patterns that power autonomous robots from warehouse AMRs (Autonomous Mobile Robots) to humanoid assistants.

---

## Why ROS 2?

### The Problem: Robot Complexity

Modern robots are **distributed systems** with dozens of independent processes:
- Camera drivers publishing images at 30 FPS
- LiDAR nodes publishing scans at 10 Hz
- Navigation algorithms requesting path planning
- Motor controllers executing velocity commands

Without a middleware layer, you'd need to:
- Write low-level socket code for inter-process communication
- Manually serialize/deserialize messages
- Handle discovery (which nodes are running?)
- Debug timing issues across multiple processes

**ROS 2 solves all of this** with a standardized API, built-in tools, and a vibrant ecosystem (10,000+ packages).

### ROS 2 vs Alternatives

| Feature | ROS 2 | ROS 1 | MQTT | Raw Sockets |
|---------|-------|-------|------|-------------|
| **Real-Time** | ✅ DDS middleware | ❌ Master bottleneck | ⚠️ QoS varies | ✅ Manual tuning |
| **Security** | ✅ DDS-Security | ❌ No encryption | ✅ TLS support | ⚠️ Manual implementation |
| **Tooling** | ✅ `ros2 topic`, RViz, rqt | ✅ Same tools (ROS 1) | ⚠️ Limited | ❌ Manual logging |
| **Ecosystem** | ✅ 10,000+ packages | ✅ Legacy support | ❌ IoT-focused | ❌ None |
| **Learning Curve** | Moderate | Moderate | Low | High (low-level) |

**Verdict**: ROS 2 is the best choice for production robotics due to real-time support, security, and ecosystem.

---

## Module Overview

### What You'll Build

By the end of this module, you'll create a **multi-node ROS 2 system** that:
1. Publishes simulated sensor data (temperature, humidity)
2. Subscribes to sensor data and logs it
3. Provides a calculator service (add/subtract)
4. Implements a countdown action with progress feedback
5. Defines a 2-link robot arm in URDF and visualizes in RViz

### Learning Objectives

- **Week 1**: Understand publisher-subscriber pattern, write `rclpy` nodes
- **Week 2**: Master services (request/response) and actions (long-running tasks)
- **Week 3**: Create robot descriptions with URDF and manage transforms (TF2)
- **Week 4**: Build complete ROS 2 packages with launch files and parameters

---

## Chapters

1. **[ROS 2 Nodes](/docs/ros2/nodes)** - The building blocks of robot systems
2. **[Topics](/docs/ros2/topics)** - Publisher-subscriber communication pattern
3. **[Services](/docs/ros2/services)** - Request/response pattern for one-off tasks
4. **[Actions](/docs/ros2/actions)** - Long-running tasks with feedback (navigation, manipulation)
5. **[rclpy](/docs/ros2/rclpy)** - Python client library for ROS 2
6. **[URDF](/docs/ros2/urdf)** - Robot descriptions and coordinate transforms

---

## Prerequisites

Before starting this module, ensure you have:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (native or WSL2 on Windows)
- **Python**: 3.10+ (included in Ubuntu 22.04)
- **Disk Space**: 10GB+ free space
- **RAM**: 4GB+ (8GB recommended for RViz)

### Software Installation

```bash
# Update package list
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble (LTS release)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep (dependency manager)
sudo rosdep init
rosdep update
```

### Verify Installation

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --help

# Expected output:
# usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...
```

---

## Recommended Development Environment

### Option 1: VS Code (Recommended)
```bash
# Install VS Code
sudo snap install code --classic

# Install ROS extension
code --install-extension ms-iot.vscode-ros
```

**Advantages**: Autocomplete, debugging, integrated terminal

### Option 2: Terminal + Text Editor
Use any text editor (Vim, Nano, Gedit) + terminal for running ROS 2 commands.

---

## Key Concepts Preview

### 1. Nodes
Independent processes that perform specific tasks (e.g., camera driver, motion planner).

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 2. Topics
Named channels for asynchronous communication (publish-subscribe).

```python
# Publisher
self.publisher = self.create_publisher(String, 'chatter', 10)
self.publisher.publish(msg)

# Subscriber
self.subscription = self.create_subscription(String, 'chatter', self.callback, 10)
```

### 3. Services
Synchronous request/response (like HTTP APIs).

```python
# Service definition (AddTwoInts.srv)
int64 a
int64 b
---
int64 sum
```

### 4. Actions
Long-running tasks with feedback (e.g., navigation).

```python
# Action definition (Countdown.action)
int32 duration
---
bool success
---
int32 remaining
```

---

## Time Commitment

| Week | Reading | Hands-On Lab | Total |
|------|---------|--------------|-------|
| Week 1 | 2 hours | 2 hours | 4 hours |
| Week 2 | 2 hours | 2 hours | 4 hours |
| Week 3 | 2 hours | 3 hours | 5 hours |
| Week 4 | 2 hours | 3 hours | 5 hours |
| **Total** | **8 hours** | **10 hours** | **18 hours** |

---

## Assessment Preview

**Assessment 1 (Week 4)**: Build a complete ROS 2 package with:
- Publisher node (simulated sensor data at 10 Hz)
- Subscriber node (logs data to terminal)
- Custom message type (`SensorData.msg`)
- Launch file to start both nodes
- Documentation (README + architecture diagram)

**Grading**: Code quality (30%), Functionality (40%), Documentation (20%), Testing (10%)

---

## Next Steps

Ready to get started? Begin with **[Chapter 1: ROS 2 Nodes](/docs/ros2/nodes)** to learn the building blocks of robot systems.

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>🚀 Let's Build Your First ROS 2 System</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    Master the nervous system that powers autonomous robots worldwide.
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/ros2/nodes"
    >
      Start Chapter 1: Nodes →
    </a>
  </div>
</div>
