---
id: physics-sim
title: Physics Simulation Fundamentals
sidebar_label: Physics Simulation
sidebar_position: 2
description: Understand physics engines, collision detection, and dynamics simulation
keywords: [physics-engines, ode, bullet, dart, collision-detection, dynamics]
---

# Physics Simulation Fundamentals

## What Are Physics Engines?

**Physics engines** simulate real-world physics—gravity, friction, collisions, and forces—allowing robots to interact with virtual environments realistically.

### Core Responsibilities

1. **Rigid Body Dynamics**: How objects move under forces (Newton's laws)
2. **Collision Detection**: Detecting when objects touch/overlap
3. **Constraint Solving**: Enforcing joint limits, contact forces
4. **Integration**: Updating positions/velocities over time

---

## Popular Physics Engines

| Engine | Used By | Strengths | Weaknesses |
|--------|---------|-----------|------------|
| **ODE** (Open Dynamics Engine) | Gazebo Classic | Stable, mature | Slower, less accurate |
| **Bullet** | Gazebo, PyBullet | Fast, accurate | Complex API |
| **DART** | Gazebo | High accuracy, robotics-focused | Less documentation |
| **PhysX** | Unity, Unreal | Industry-standard, GPU-accelerated | Proprietary (NVIDIA) |

**Gazebo supports all three** (ODE, Bullet, DART)—choose based on your needs.

---

## Key Physics Concepts

### 1. Gravity

```xml
<physics type="ode">
  <gravity>0 0 -9.81</gravity>  <!-- Earth gravity (m/s²) -->
</physics>
```

**Effect**: All objects fall at 9.81 m/s² unless supported.

---

### 2. Friction

```xml
<collision>
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>   <!-- Coefficient of friction -->
        <mu2>0.8</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

**Effect**: Higher `mu` = less slipping (wheels, grasping).

---

### 3. Mass and Inertia

```xml
<inertial>
  <mass value="10.0"/>  <!-- kg -->
  <inertia ixx="0.1" ixy="0" ixz="0"
           iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

**Effect**: Heavier objects require more force to move.

---

### 4. Time Step

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

- **max_step_size**: Smaller = more accurate (but slower)
- **real_time_factor**: 1.0 = real-time, 0.5 = half speed

---

## Collision Detection

### Collision Shapes

Use **simple shapes** (box, cylinder, sphere) for performance:

```xml
<!-- Visual: complex mesh -->
<visual>
  <geometry>
    <mesh filename="robot_body.stl"/>
  </geometry>
</visual>

<!-- Collision: simplified box -->
<collision>
  <geometry>
    <box size="0.6 0.4 0.2"/>
  </geometry>
</collision>
```

**Rule**: Collision geometry should be 10-100× simpler than visual mesh.

---

## Choosing a Physics Engine

### ODE (Default)

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
    </solver>
  </ode>
</physics>
```

**Use when**: Stability > speed (mobile robots, slow-moving systems).

---

### Bullet

```xml
<physics type="bullet">
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
    </solver>
  </bullet>
</physics>
```

**Use when**: Fast simulation needed (multi-robot, large scenes).

---

### DART

```xml
<physics type="dart">
  <dart>
    <solver>
      <type>dantzig</type>
    </solver>
  </dart>
</physics>
```

**Use when**: High accuracy required (manipulation, contact-rich tasks).

---

## Real-World Example: Tuning Physics

```xml
<world name="warehouse">
  <physics type="bullet">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>

    <bullet>
      <solver>
        <type>sequential_impulse</type>
        <iters>100</iters>  <!-- More iterations = more stable -->
      </solver>
    </bullet>
  </physics>

  <gravity>0 0 -9.81</gravity>
</world>
```

---

## Hands-On Lab: Drop Test

**Goal**: Create a world and drop objects with different masses.

### Requirements

1. Create world with gravity
2. Spawn 3 spheres (1kg, 5kg, 10kg) at height 5m
3. Observe fall times (should be ~1.01s for all)

### Starter Code

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="drop_test">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- TODO: Add sphere models at z=5.0 -->

  </world>
</sdf>
```

**Expected**: All spheres hit ground at same time (Galileo's experiment!).

---

## Key Takeaways

✅ **Physics engines** simulate gravity, friction, collisions
✅ **ODE** (stable), **Bullet** (fast), **DART** (accurate)
✅ **Smaller time steps** = more accurate (but slower)
✅ **Simplified collision geometry** improves performance

---

## Next Steps

Learn to set up Gazebo in **[Chapter 2: Gazebo Setup](/docs/gazebo-unity/gazebo-setup)**.

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/gazebo-unity/gazebo-setup"
    >
      Next: Gazebo Setup →
    </a>
  </div>
</div>
