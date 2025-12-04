---
id: sensors-depth
title: Depth Camera Simulation
sidebar_label: Depth Cameras
sidebar_position: 6
description: Simulate Intel RealSense D435i depth cameras for computer vision
keywords: [depth-camera, realsense-d435i, rgbd, point-cloud, computer-vision]
---

# Depth Camera Simulation

## What are Depth Cameras?

**Depth cameras** measure distance to every pixel, creating 3D point clouds. The **Intel RealSense D435i** is industry-standard for robotics, providing:

- **RGB image** (1920×1080, 30 FPS)
- **Depth image** (1280×720, 30 FPS, up to 10m range)
- **IMU** (accelerometer + gyroscope)

---

## Adding Depth Camera to Robot

### URDF with Camera Link

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.025 0.09 0.025"/>  <!-- RealSense D435i dimensions -->
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.025 0.09 0.025"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.072"/>  <!-- RealSense D435i: 72g -->
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
</joint>
```

---

## Gazebo Depth Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <visualize>false</visualize>

    <camera>
      <horizontal_fov>1.51844</horizontal_fov>  <!-- 87° (RealSense spec) -->
      <image>
        <width>1280</width>
        <height>720</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.2</near>  <!-- Minimum range -->
        <far>10.0</far>   <!-- Maximum range -->
      </clip>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Depth noise -->
      </noise>
    </camera>

    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=camera/color/image_raw</remapping>
        <remapping>~/depth/image_raw:=camera/depth/image_raw</remapping>
        <remapping>~/depth/points:=camera/depth/points</remapping>
        <remapping>~/camera_info:=camera/color/camera_info</remapping>
      </ros>

      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- Stereo baseline -->
      <min_depth>0.2</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

---

## Accessing Camera Data

### RGB Image

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RGBViewer(Node):
    def __init__(self):
        super().__init__('rgb_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/color/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display image
        cv2.imshow('RGB Camera', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RGBViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Depth Image

```python
class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/depth/image_raw',
            self.depth_callback,
            10
        )

    def depth_callback(self, msg):
        # Convert to OpenCV (depth in meters)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Normalize for visualization (0-10m → 0-255)
        depth_normalized = (depth_image / 10.0 * 255).astype('uint8')

        cv2.imshow('Depth Camera', depth_normalized)
        cv2.waitKey(1)
```

### Point Cloud

```python
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/robot/camera/depth/points',
            self.pointcloud_callback,
            10
        )

    def pointcloud_callback(self, msg):
        # Extract XYZ points
        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        count = 0
        for point in points:
            x, y, z = point
            # Process point (e.g., check if obstacle within 1m)
            if z < 1.0:
                count += 1

        self.get_logger().info(f'Points within 1m: {count}')
```

---

## Visualizing in RViz

```bash
rviz2
```

**Add displays**:
1. **Image**: Topic `/robot/camera/color/image_raw`
2. **DepthCloud**: Topic `/robot/camera/depth/points`
3. **TF**: Show camera pose

---

## Object Detection with Depth

```python
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image, '/robot/camera/color/image_raw', self.rgb_callback, 10
        )

        self.depth_sub = self.create_subscription(
            Image, '/robot/camera/depth/image_raw', self.depth_callback, 10
        )

        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

        if self.rgb_image is not None:
            self.detect_objects()

    def detect_objects(self):
        # Simple color-based detection (red objects)
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Get depth at center
                center_depth = self.depth_image[y + h//2, x + w//2]

                self.get_logger().info(
                    f'Red object detected at ({x}, {y}), distance: {center_depth:.2f}m'
                )

                # Draw bounding box
                cv2.rectangle(self.rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(
                    self.rgb_image,
                    f'{center_depth:.2f}m',
                    (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

        cv2.imshow('Object Detection', self.rgb_image)
        cv2.waitKey(1)
```

---

## Hands-On Lab: Depth-Based Navigation

**Goal**: Use depth camera to detect obstacles and navigate around them.

### Requirements

1. Subscribe to `/robot/camera/depth/image_raw`
2. Check if any pixel in center region (±50px) is < 1m
3. If obstacle detected, turn; otherwise, move forward
4. Publish to `/cmd_vel`

---

## Key Takeaways

✅ **Depth cameras** provide RGB + depth + point cloud
✅ **RealSense D435i** standard for robotics
✅ **Use `cv_bridge`** to convert ROS Image ↔ OpenCV
✅ **Combine RGB + depth** for robust perception

---

## Next Steps

Learn IMU simulation in **[Chapter 6: IMU Sensors](/docs/gazebo-unity/sensors-imu)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/gazebo-unity/sensors-imu">
      Next: IMU Sensors →
    </a>
  </div>
</div>
