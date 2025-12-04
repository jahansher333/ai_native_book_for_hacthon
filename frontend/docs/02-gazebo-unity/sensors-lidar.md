---
id: sensors-lidar
title: LiDAR Sensor Simulation
sidebar_label: LiDAR Sensors
sidebar_position: 5
description: Simulate 2D/3D LiDAR sensors with realistic noise models in Gazebo
keywords: [lidar, laser-scanner, ray-casting, sensor-simulation, ros2-laserscan]
---

# LiDAR Sensor Simulation

## What is LiDAR?

**LiDAR** (Light Detection and Ranging) measures distances by emitting laser beams and timing their reflections. Robots use LiDAR for obstacle detection, mapping (SLAM), and navigation.

### LiDAR Types

- **2D LiDAR**: Single horizontal plane (360°) — e.g., SICK TiM, Hokuyo
- **3D LiDAR**: Multiple vertical layers — e.g., Velodyne VLP-16 (16 beams), Ouster OS1

---

## Adding LiDAR to Robot

### URDF with LiDAR Link

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
</joint>
```

---

## Gazebo LiDAR Plugin

Add to robot URDF (in `<robot>` tag, not inside `<link>`):

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>  <!-- 10 Hz -->
    <visualize>true</visualize>

    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>  <!-- Number of beams -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
      </scan>

      <range>
        <min>0.12</min>  <!-- Minimum range (m) -->
        <max>10.0</max>  <!-- Maximum range (m) -->
        <resolution>0.01</resolution>
      </range>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm noise -->
      </noise>
    </ray>

    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

---

## Testing LiDAR in Gazebo

### Launch Robot with LiDAR

```bash
ros2 launch my_robot robot.launch.py
```

### View LiDAR Data

```bash
# Echo scan data
ros2 topic echo /robot/scan

# Visualize in RViz
rviz2
```

**In RViz**:
1. Set **Fixed Frame** to `base_link` or `lidar_link`
2. Add **LaserScan** display
3. Set **Topic** to `/robot/scan`
4. You should see red dots representing obstacles

---

## LaserScan Message Format

```python
from sensor_msgs.msg import LaserScan

# LaserScan structure:
# - header: timestamp + frame_id
# - angle_min: start angle (radians)
# - angle_max: end angle (radians)
# - angle_increment: angular resolution
# - time_increment: time between measurements
# - scan_time: time for complete scan
# - range_min: minimum range (m)
# - range_max: maximum range (m)
# - ranges: distance measurements (array)
# - intensities: reflection intensities (optional)
```

### Processing LiDAR Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Find closest obstacle
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)

        # Calculate angle of closest obstacle
        angle = msg.angle_min + (min_index * msg.angle_increment)

        self.get_logger().info(
            f'Closest obstacle: {min_distance:.2f}m at {angle:.2f} rad'
        )

        # Check for obstacles in front (±15 degrees)
        front_ranges = msg.ranges[345:375]  # Assuming 720 samples
        front_min = min(front_ranges)

        if front_min < 0.5:
            self.get_logger().warn(f'Obstacle ahead: {front_min:.2f}m!')

def main():
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 3D LiDAR (Velodyne VLP-16)

For 3D LiDAR, use **PointCloud2** instead of **LaserScan**:

```xml
<gazebo reference="lidar_3d_link">
  <sensor name="velodyne" type="gpu_ray">
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>  <!-- 16 vertical layers -->
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle>  <!-- -15° -->
          <max_angle>0.2618</max_angle>   <!-- +15° -->
        </vertical>
      </scan>
      <range>
        <min>0.5</min>
        <max>100.0</max>
      </range>
    </ray>

    <plugin name="gazebo_ros_velodyne" filename="libgazebo_ros_velodyne_laser.so">
      <ros>
        <remapping>~/out:=velodyne_points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
    </plugin>
  </sensor>
</gazebo>
```

---

## Hands-On Lab: Obstacle Avoidance

**Goal**: Use LiDAR to avoid obstacles.

### Requirements

1. Subscribe to `/robot/scan`
2. If obstacle within 0.5m ahead, turn right
3. Otherwise, move forward
4. Publish velocity commands to `/cmd_vel`

### Starter Code

```python
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.scan_sub = self.create_subscription(
            LaserScan, '/robot/scan', self.scan_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_ahead = False

    def scan_callback(self, msg):
        # TODO: Check front ranges (±15°)
        # TODO: Set self.obstacle_ahead flag
        pass

    def control_loop(self):
        twist = Twist()

        if self.obstacle_ahead:
            # TODO: Turn right
            pass
        else:
            # TODO: Move forward
            pass

        self.cmd_vel_pub.publish(twist)
```

---

## Key Takeaways

✅ **LiDAR** measures distances using laser beams
✅ **2D LiDAR** (LaserScan), **3D LiDAR** (PointCloud2)
✅ **Gazebo plugin** `libgazebo_ros_ray_sensor.so`
✅ **Add noise** for realistic simulation

---

## Next Steps

Learn depth camera simulation in **[Chapter 5: Depth Cameras](/docs/gazebo-unity/sensors-depth)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/gazebo-unity/sensors-depth">
      Next: Depth Cameras →
    </a>
  </div>
</div>
