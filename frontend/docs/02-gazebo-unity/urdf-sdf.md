---
id: urdf-sdf
title: URDF vs SDF Model Formats
sidebar_label: URDF/SDF Models
sidebar_position: 8
description: Understand robot description formats for ROS 2 and Gazebo
keywords: [urdf, sdf, xacro, robot-models, gazebo-models]
---

# URDF vs SDF Model Formats

## Format Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Purpose** | ROS robot descriptions | Gazebo world/robot descriptions |
| **Expressiveness** | Basic (single robot) | Advanced (multiple robots, plugins) |
| **Kinematics** | Yes | Yes |
| **Dynamics** | Limited | Full (collision, friction, damping) |
| **Sensors** | Requires Gazebo tags | Native support |
| **World Description** | No | Yes |
| **Version** | 1.0 (fixed) | 1.0-1.9 (evolving) |

**Summary**: Use **URDF for ROS 2**, convert to **SDF for Gazebo** simulation.

---

## URDF (Unified Robot Description Format)

### Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <!-- Joints (connections) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Gazebo-specific tags (sensors, plugins) -->
  <gazebo reference="link1">
    <sensor name="camera" type="camera">...</sensor>
  </gazebo>
</robot>
```

### Limitations

- Cannot describe multiple robots
- No world description (ground, obstacles, lighting)
- Gazebo features require `<gazebo>` tags

---

## SDF (Simulation Description Format)

### Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <!-- World -->
  <world name="default">
    <physics>...</physics>
    <light>...</light>

    <!-- Models (robots, objects) -->
    <model name="robot1">
      <link name="base_link">...</link>
      <joint name="joint1">...</joint>
      <plugin name="diff_drive">...</plugin>
    </model>

    <model name="robot2">...</model>
  </world>
</sdf>
```

### Advantages

- Native support for **multiple robots**
- Built-in **physics configuration**
- **Sensor plugins** without extra tags
- **World description** (lighting, gravity)

---

## Converting URDF to SDF

Gazebo automatically converts URDF → SDF at runtime, but you can do it manually:

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf
```

### Example Conversion

**URDF**:
```xml
<link name="wheel">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </visual>
</link>
```

**SDF** (converted):
```xml
<link name="wheel">
  <visual name="visual">
    <geometry>
      <cylinder>
        <radius>0.1</radius>
        <length>0.05</length>
      </cylinder>
    </geometry>
  </visual>
</link>
```

---

## Xacro (XML Macros)

**Xacro** extends URDF with macros, variables, and math—reducing repetition.

### Without Xacro (Repetitive)

```xml
<link name="wheel_left">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </visual>
</link>

<link name="wheel_right">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </visual>
</link>
```

### With Xacro (DRY)

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="side">
    <link name="wheel_${side}">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel side="left"/>
  <xacro:wheel side="right"/>

</robot>
```

### Convert Xacro → URDF

```bash
xacro robot.xacro > robot.urdf
```

---

## Best Practices

### For ROS 2

1. **Use URDF** (required for `robot_state_publisher`, RViz)
2. **Use Xacro** for modularity (separate files for sensors, wheels)
3. **Add Gazebo tags** for simulation (plugins, sensors)

### For Gazebo-Only

1. **Use SDF directly** if not using ROS 2
2. **Separate world and model files** (`world.sdf`, `model.sdf`)

### Hybrid Workflow (Recommended)

1. **Develop in URDF/Xacro** (ROS 2 compatibility)
2. **Test in RViz** (fast, no physics)
3. **Simulate in Gazebo** (auto-converts URDF → SDF)
4. **Export final SDF** for production (faster loading)

---

## Real-World Example: Mobile Robot

### robot.xacro

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_width" value="0.4"/>

  <!-- Include separate files -->
  <xacro:include filename="$(find my_robot_description)/urdf/base.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/wheels.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>

  <!-- Base link -->
  <xacro:base/>

  <!-- Wheels -->
  <xacro:wheel side="left" reflect="1"/>
  <xacro:wheel side="right" reflect="-1"/>

  <!-- Sensors -->
  <xacro:lidar/>
  <xacro:camera/>
  <xacro:imu/>

</robot>
```

---

## Hands-On Lab: Modular Robot

**Goal**: Create a robot using Xacro macros.

### Requirements

1. Separate files: `base.xacro`, `wheels.xacro`, `sensors.xacro`
2. Main file `robot.xacro` includes all
3. Use properties for dimensions
4. Generate URDF and test in Gazebo

### Starter Code (base.xacro)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="base">
    <link name="base_link">
      <visual>
        <geometry>
          <box size="0.6 0.4 0.2"/>
        </geometry>
      </visual>
      <!-- TODO: Add collision, inertial -->
    </link>
  </xacro:macro>

</robot>
```

---

## Key Takeaways

✅ **URDF** for ROS 2 (robot_state_publisher, RViz)
✅ **SDF** for Gazebo (worlds, multiple robots)
✅ **Xacro** for modularity (DRY principle)
✅ **Gazebo auto-converts** URDF → SDF

---

## Next Steps

You've completed Module 2! Move on to **[Module 3: NVIDIA Isaac Sim](/docs/isaac)** for advanced simulation.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>🎉 Module 2 Complete!</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    You've mastered digital twins with Gazebo and Unity. Ready for NVIDIA Isaac Sim?
  </p>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/isaac">
      Start Module 3: NVIDIA Isaac Sim →
    </a>
  </div>
</div>
