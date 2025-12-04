---
id: unity-sim
title: Unity for Robotics Simulation
sidebar_label: Unity Simulation
sidebar_position: 4
description: Use Unity for photorealistic robot simulation and synthetic data generation
keywords: [unity, unity-robotics-hub, photorealistic-simulation, synthetic-data]
---

# Unity for Robotics Simulation

## Why Unity?

**Unity** is a professional game engine that offers **photorealistic rendering** and **GPU-accelerated physics** (PhysX). While Gazebo excels at robotics prototyping, Unity is ideal for:

- **Computer vision training**: High-fidelity synthetic images
- **Domain randomization**: Vary textures, lighting, object poses
- **Marketing demos**: Beautiful visualizations of robots

---

## Unity vs Gazebo

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Setup Time** | 10 minutes | 1-2 hours |
| **Rendering Quality** | Basic (OGRE) | Photorealistic (HDRP) |
| **ROS 2 Integration** | Native | Via Unity Robotics Hub |
| **Learning Curve** | Robotics-focused | Game development knowledge needed |
| **Best For** | Quick prototyping | Synthetic data generation |

---

## Installation

### Step 1: Install Unity Hub

```bash
# Download Unity Hub (Ubuntu)
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.AppImage
chmod +x UnityHubSetup.AppImage
./UnityHubSetup.AppImage
```

### Step 2: Install Unity Editor

1. Open Unity Hub
2. Click **Installs** → **Add**
3. Select **Unity 2022.3 LTS** (Long Term Support)
4. Add modules: **Linux Build Support**

### Step 3: Install Unity Robotics Hub

```bash
# Clone Unity Robotics Hub
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub
```

---

## ROS 2 Integration

Unity Robotics Hub provides **ROS-TCP-Connector** to bridge Unity and ROS 2.

### Architecture

```mermaid
graph LR
    A[Unity Scene] <-->|TCP| B[ROS-TCP-Endpoint]
    B <-->|ROS 2 Topics| C[ROS 2 Nodes]
```

### Setup ROS-TCP-Endpoint

```bash
# Install ROS-TCP-Endpoint package
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

### Configure Unity

1. In Unity Editor: **Window → Package Manager**
2. Add package by git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3. **Robotics → ROS Settings**
   - Protocol: ROS 2
   - ROS IP: 127.0.0.1
   - ROS Port: 10000

---

## Creating a Robot in Unity

### Import URDF

Unity Robotics Hub includes URDF importer:

1. **Assets → Import New Asset** → Select `robot.urdf`
2. Unity converts URDF → GameObjects automatically
3. Joints become **ArticulationBody** components

### Add Physics

```csharp
// Attach to robot GameObject
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", MoveRobot);
    }

    void MoveRobot(TwistMsg twist)
    {
        // Apply velocity commands
        float linearX = (float)twist.linear.x;
        float angularZ = (float)twist.angular.z;

        // Move robot using Unity physics
        GetComponent<Rigidbody>().velocity = new Vector3(linearX, 0, 0);
    }
}
```

---

## Synthetic Data Generation

### Domain Randomization Script

```csharp
using UnityEngine;

public class DomainRandomizer : MonoBehaviour
{
    public Material[] randomMaterials;
    public Light sceneLight;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            RandomizeScene();
        }
    }

    void RandomizeScene()
    {
        // Randomize object textures
        var objects = FindObjectsOfType<Renderer>();
        foreach (var obj in objects)
        {
            obj.material = randomMaterials[Random.Range(0, randomMaterials.Length)];
        }

        // Randomize lighting
        sceneLight.intensity = Random.Range(0.5f, 2.0f);
        sceneLight.color = new Color(
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f)
        );
    }
}
```

---

## Exporting Images to ROS 2

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    Camera cam;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("/camera/image");
        cam = GetComponent<Camera>();
    }

    void Update()
    {
        // Capture frame
        RenderTexture rt = cam.targetTexture;
        Texture2D image = new Texture2D(rt.width, rt.height);
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        image.Apply();

        // Convert to ROS message
        ImageMsg msg = new ImageMsg();
        msg.data = image.GetRawTextureData();
        msg.width = (uint)rt.width;
        msg.height = (uint)rt.height;

        ros.Publish("/camera/image", msg);
    }
}
```

---

## Hands-On Lab: Unity Pick-and-Place

**Goal**: Create a robot arm that picks objects in Unity.

### Requirements

1. Import robot arm URDF
2. Spawn 10 objects with random textures
3. Implement pick logic (collision detection)
4. Export camera feed to ROS 2 topic `/camera/image`

---

## Key Takeaways

✅ **Unity** for photorealistic rendering
✅ **Unity Robotics Hub** bridges Unity ↔ ROS 2
✅ **Domain randomization** for robust vision models
✅ **Use for synthetic data**, not real-time control

---

## Next Steps

Learn LiDAR simulation in **[Chapter 4: LiDAR Sensors](/docs/gazebo-unity/sensors-lidar)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/gazebo-unity/sensors-lidar">
      Next: LiDAR Sensors →
    </a>
  </div>
</div>
