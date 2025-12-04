---
id: vslam
title: Visual SLAM (VSLAM)
sidebar_label: Visual SLAM
sidebar_position: 4
description: Implement ORB-SLAM3 for visual simultaneous localization and mapping
keywords: [vslam, orb-slam3, rtab-map, slam, visual-odometry, mapping]
---

# Visual SLAM (VSLAM)

## What is VSLAM?

**Visual SLAM** (Simultaneous Localization and Mapping) uses cameras to:
1. **Localize** robot in environment (where am I?)
2. **Map** environment (what does the world look like?)
3. **Do both simultaneously** (solve chicken-egg problem)

Unlike LiDAR SLAM, VSLAM works in GPS-denied environments (indoors, tunnels) and provides rich visual features.

---

## VSLAM Algorithms

| Algorithm | Type | Best For |
|-----------|------|----------|
| **ORB-SLAM3** | Feature-based | Real-time, monocular/stereo/RGB-D |
| **RTAB-Map** | Graph-based | Large-scale, loop closure |
| **LSD-SLAM** | Direct | Dense reconstruction |
| **SVO** (Semi-Direct) | Hybrid | High-frame-rate cameras |

**We'll use ORB-SLAM3**: Best balance of speed, accuracy, and ROS 2 support.

---

## Installing ORB-SLAM3

### Dependencies

```bash
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libssl-dev

# Pangolin (visualization)
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake ..
make -j4
sudo make install
```

### ORB-SLAM3

```bash
cd ~/ros2_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

---

## ROS 2 Integration

### Install ROS 2 Wrapper

```bash
cd ~/ros2_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git
cd ~/ros2_ws
colcon build --packages-select orb_slam3_ros2
source install/setup.bash
```

---

## Running VSLAM in Isaac Sim

### Step 1: Launch Isaac Sim with Camera

```python
# Isaac Sim: Add RGB-D camera to robot
from omni.isaac.sensor import Camera

camera = Camera(
    prim_path="/World/Carter/camera",
    position=np.array([0.3, 0, 0.2]),
    frequency=30,
    resolution=(640, 480)
)
```

### Step 2: Launch ORB-SLAM3

```bash
# Terminal 1: Isaac Sim (running)

# Terminal 2: ORB-SLAM3 ROS 2 node
cd ~/ros2_ws
source install/setup.bash

ros2 run orb_slam3_ros2 rgbd \
    --ros-args \
    -p vocabulary_file:=~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    -p settings_file:=~/ORB_SLAM3/Examples/RGB-D/RealSense.yaml \
    -r /camera/color/image_raw:=/robot/camera/color/image_raw \
    -r /camera/depth/image_raw:=/robot/camera/depth/image_raw
```

**You should see**: Pangolin visualization showing camera pose and 3D map points.

---

## Visualizing SLAM in RViz

### Launch RViz

```bash
rviz2
```

**Add displays**:
1. **TF**: See camera/robot pose
2. **PointCloud2**: SLAM map points (topic: `/orb_slam3/map_points`)
3. **Path**: Robot trajectory (topic: `/orb_slam3/camera_path`)

---

## Monocular SLAM

For single RGB camera (no depth):

```bash
ros2 run orb_slam3_ros2 mono \
    --ros-args \
    -p vocabulary_file:=~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    -p settings_file:=~/ORB_SLAM3/Examples/Monocular/TUM1.yaml \
    -r /camera/image_raw:=/robot/camera/color/image_raw
```

---

## Stereo SLAM

For stereo cameras (left + right):

```bash
ros2 run orb_slam3_ros2 stereo \
    --ros-args \
    -p vocabulary_file:=~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    -p settings_file:=~/ORB_SLAM3/Examples/Stereo/EuRoC.yaml \
    -r /camera/left/image_raw:=/robot/left_camera/image_raw \
    -r /camera/right/image_raw:=/robot/right_camera/image_raw
```

---

## Saving Maps

ORB-SLAM3 saves maps in KeyFrame format:

```bash
# After running SLAM, press 's' in Pangolin window to save
# Map saved to: ~/ORB_SLAM3/map.osa
```

### Load Existing Map

```bash
ros2 run orb_slam3_ros2 rgbd \
    --ros-args \
    -p vocabulary_file:=~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    -p settings_file:=~/ORB_SLAM3/Examples/RGB-D/RealSense.yaml \
    -p load_map:=true \
    -p map_file:=~/ORB_SLAM3/map.osa
```

---

## RTAB-Map Alternative

**RTAB-Map** (Real-Time Appearance-Based Mapping) for large-scale SLAM:

### Install RTAB-Map

```bash
sudo apt install ros-humble-rtabmap-ros
```

### Launch with Isaac Sim

```bash
ros2 launch rtabmap_ros rtabmap.launch.py \
    rgb_topic:=/robot/camera/color/image_raw \
    depth_topic:=/robot/camera/depth/image_raw \
    camera_info_topic:=/robot/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    queue_size:=30
```

---

## Hands-On Lab: Indoor Mapping

**Goal**: Create a 3D map of a warehouse using VSLAM.

### Requirements

1. Isaac Sim scene with warehouse (walls, obstacles)
2. Robot with RGB-D camera
3. Run ORB-SLAM3
4. Drive robot around (manual teleop)
5. Visualize map in RViz
6. Save map for later use

### Starter Steps

1. Launch Isaac Sim with warehouse scene
2. Enable ROS 2 bridge
3. Launch ORB-SLAM3 node
4. Launch teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
5. Drive robot to explore environment
6. Save map: Press 's' in Pangolin window

---

## Key Takeaways

✅ **VSLAM** localizes and maps using cameras
✅ **ORB-SLAM3** best for real-time robotics
✅ **RGB-D** more accurate than monocular
✅ **RTAB-Map** for large-scale, multi-session SLAM

---

## Next Steps

Learn autonomous navigation with **[Nav2](/docs/isaac/nav2)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/isaac/nav2">
      Next: Nav2 Navigation →
    </a>
  </div>
</div>
