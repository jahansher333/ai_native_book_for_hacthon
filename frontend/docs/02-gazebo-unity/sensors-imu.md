---
id: sensors-imu
title: IMU Sensor Simulation
sidebar_label: IMU Sensors
sidebar_position: 7
description: Simulate inertial measurement units for odometry and state estimation
keywords: [imu, accelerometer, gyroscope, orientation, sensor-fusion]
---

# IMU Sensor Simulation

## What is an IMU?

**IMU** (Inertial Measurement Unit) measures:
- **Accelerometer**: Linear acceleration (m/s²) in 3 axes
- **Gyroscope**: Angular velocity (rad/s) in 3 axes
- **Magnetometer** (optional): Magnetic field for compass heading

Robots use IMUs for **odometry**, **orientation estimation**, and **state estimation** (via sensor fusion with other sensors).

---

## Adding IMU to Robot

### URDF with IMU Link

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="green">
      <color rgba="0 1 0 1"/>
    </material>
  </visual>

  <inertial>
    <mass value="0.001"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

---

## Gazebo IMU Plugin

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz -->
    <visualize>true</visualize>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.002</stddev>  <!-- Gyro noise -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.002</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- Accel noise -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

---

## Accessing IMU Data

### Subscribe to IMU Topic

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Orientation (quaternion)
        orientation_q = msg.orientation
        orientation = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        # Angular velocity (rad/s)
        angular_velocity = msg.angular_velocity
        gyro_x = angular_velocity.x
        gyro_y = angular_velocity.y
        gyro_z = angular_velocity.z

        # Linear acceleration (m/s²)
        linear_acceleration = msg.linear_acceleration
        accel_x = linear_acceleration.x
        accel_y = linear_acceleration.y
        accel_z = linear_acceleration.z

        self.get_logger().info(
            f'Orientation (RPY): ({orientation[0]:.2f}, {orientation[1]:.2f}, {orientation[2]:.2f}) rad'
        )
        self.get_logger().info(
            f'Angular velocity: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) rad/s'
        )
        self.get_logger().info(
            f'Linear acceleration: ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) m/s²'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rclpy.init()
    node = IMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## IMU Message Structure

```python
# sensor_msgs/Imu
orientation:         # Quaternion (x, y, z, w)
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance: [0.0] * 9  # 3x3 matrix (row-major)

angular_velocity:    # rad/s
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0] * 9

linear_acceleration:  # m/s²
  x: 0.0
  y: -9.81  # Gravity
  z: 0.0
linear_acceleration_covariance: [0.0] * 9
```

---

## Sensor Fusion (IMU + Odometry)

Combine IMU with wheel odometry for robust state estimation:

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/robot/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher (fused pose)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)

        self.imu_orientation = None
        self.odom_pose = None

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation

        if self.odom_pose is not None:
            self.publish_fused_pose()

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

        if self.imu_orientation is not None:
            self.publish_fused_pose()

    def publish_fused_pose(self):
        # Use odometry position, IMU orientation
        fused_msg = PoseWithCovarianceStamped()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = 'odom'

        fused_msg.pose.pose.position = self.odom_pose.position
        fused_msg.pose.pose.orientation = self.imu_orientation

        self.pose_pub.publish(fused_msg)
```

---

## Detecting Robot Motion

```python
class MotionDetector(Node):
    def __init__(self):
        super().__init__('motion_detector')

        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )

        self.motion_threshold = 0.1  # m/s²

    def imu_callback(self, msg):
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y

        # Calculate magnitude (ignore gravity on z-axis)
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2)

        if accel_magnitude > self.motion_threshold:
            self.get_logger().info('Robot is moving!')
        else:
            self.get_logger().info('Robot is stationary')
```

---

## Hands-On Lab: Tilt Detection

**Goal**: Detect when robot tilts beyond safe threshold (e.g., on ramp).

### Requirements

1. Subscribe to `/robot/imu`
2. Convert quaternion → Euler angles (roll, pitch, yaw)
3. If |roll| > 15° or |pitch| > 15°, log warning
4. Bonus: Publish emergency stop to `/cmd_vel` if tilt > 30°

### Starter Code

```python
class TiltDetector(Node):
    def __init__(self):
        super().__init__('tilt_detector')

        self.imu_sub = self.create_subscription(Imu, '/robot/imu', self.imu_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def imu_callback(self, msg):
        # TODO: Convert quaternion to Euler
        # TODO: Check roll/pitch thresholds
        # TODO: Publish stop command if unsafe
        pass
```

---

## Key Takeaways

✅ **IMU** measures acceleration, angular velocity, orientation
✅ **100 Hz update rate** typical for robotics
✅ **Add noise** for realistic simulation
✅ **Combine with odometry** for robust state estimation

---

## Next Steps

Understand URDF/SDF model formats in **[Chapter 7: URDF/SDF](/docs/gazebo-unity/urdf-sdf)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/gazebo-unity/urdf-sdf">
      Next: URDF/SDF Models →
    </a>
  </div>
</div>
