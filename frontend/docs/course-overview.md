---
id: course-overview
title: Course Overview
sidebar_label: Course Overview
sidebar_position: 2
description: Complete 13-week breakdown with learning outcomes, weekly topics, and assessment criteria
keywords: [course-structure, syllabus, learning-outcomes, assessments, physical-ai-curriculum]
---

# Course Overview: Physical AI & Humanoid Robotics

## Course Summary

**Duration**: 13 weeks (10-13 weeks flexible)
**Time Commitment**: 3-5 hours per week
**Format**: Self-paced with hands-on labs and assessments
**Prerequisites**: Basic Python, command line familiarity, high school math
**Cost**: $700 (Economy Jetson Kit) or $205/quarter (cloud-native)

This course teaches you to build **autonomous humanoid robots** that understand natural language commands and interact with the real world. You'll master the complete Physical AI stack: **ROS 2** (robot middleware), **Gazebo/Isaac Sim** (digital twins), **Nav2/VSLAM** (navigation), and **Vision-Language-Action models** (AI planning).

By the end, you'll deploy a **voice-controlled humanoid** that can execute commands like *"Clean the room"* or *"Bring me water"* using edge AI (Jetson Orin Nano).

---

## 🎯 Learning Outcomes

By completing this course, you will be able to:

### 1. **Design & Implement ROS 2 Architectures**
- Build distributed robot systems using nodes, topics, services, and actions
- Write production-quality Python code with `rclpy` for real-time control
- Create URDF robot descriptions and manage coordinate transforms with TF2
- Debug ROS 2 systems using command-line tools (`ros2 topic`, `ros2 node`, `rqt`)

### 2. **Create High-Fidelity Digital Twins**
- Build realistic physics simulations in Gazebo and Unity
- Simulate sensors (LiDAR, depth cameras, IMUs) with accurate noise models
- Generate synthetic training data with domain randomization in Isaac Sim
- Optimize simulation performance for large-scale training (10,000+ episodes)

### 3. **Execute Sim-to-Real Transfer Workflows**
- Train models in cloud environments (AWS g5.xlarge, local RTX workstation)
- Export models to edge-optimized formats (ONNX, TensorRT)
- Deploy AI inference to Jetson Orin Nano for &lt;10ms latency
- Apply domain randomization to bridge the reality gap

### 4. **Integrate Vision-Language-Action Models**
- Use Whisper for real-time speech-to-text on edge devices
- Implement LLM-based planning (GPT-4, Claude) to translate natural language → robot actions
- Build ROS 2 action servers for long-running tasks (navigation, manipulation)
- Handle error cases (obstacles, speech recognition failures, unreachable goals)

### 5. **Deploy Production-Ready Robot Systems**
- Understand latency requirements for safe robot control (&lt;10ms for real-time)
- Optimize models for edge inference (quantization, pruning, TensorRT)
- Implement safety protocols (emergency stop, collision avoidance, timeout handling)
- Monitor system health (CPU/GPU usage, network latency, battery status)

### 6. **Evaluate & Iterate on Physical AI Systems**
- Measure sim-to-real transfer success (task completion rate, trajectory error)
- Collect deployment metrics (inference latency, power consumption, reliability)
- Debug failures using ROS 2 logs, bag files, and visualization tools (RViz)
- Iterate rapidly with cloud training → edge deployment pipelines

---

## 📅 Weekly Breakdown

### **Module 1: Robotic Nervous System (ROS 2)** - Weeks 1-4

#### Week 1: ROS 2 Fundamentals
**Topics**:
- Why ROS 2? (vs ROS 1, MQTT, raw sockets)
- Installation (Ubuntu 22.04, Humble Hawksbill)
- Nodes, topics, and the publisher-subscriber pattern
- Writing your first `rclpy` node

**Hands-On Lab**: Publish "Hello, ROS 2!" messages to a topic and visualize with `ros2 topic echo`.

**Reading**: 2 hours | **Lab**: 2 hours

---

#### Week 2: Services & Actions
**Topics**:
- Request/response pattern with ROS 2 services
- Long-running tasks with ROS 2 actions (goal, feedback, result)
- When to use topics vs services vs actions
- Custom message/service/action definitions

**Hands-On Lab**: Build a calculator service (add, subtract) and a timed countdown action.

**Reading**: 2 hours | **Lab**: 2 hours

---

#### Week 3: Robot Descriptions (URDF)
**Topics**:
- URDF syntax (links, joints, visual, collision)
- TF2 coordinate transforms (base_link → camera_link → map)
- Visualizing robots in RViz
- Joint states and robot state publisher

**Hands-On Lab**: Create a simple 2-link robot arm URDF and visualize in RViz.

**Reading**: 2 hours | **Lab**: 3 hours

---

#### Week 4: ROS 2 Packages & Launch Files
**Topics**:
- Creating ROS 2 packages with `colcon`
- Launch files (Python) for multi-node systems
- Parameters and dynamic reconfiguration
- **Assessment 1 Prep**: ROS 2 pub/sub package

**Hands-On Lab**: Build a complete ROS 2 package with launch file, params, and multiple nodes.

**Reading**: 2 hours | **Lab**: 3 hours

---

### **Module 2: Digital Twin (Gazebo & Unity)** - Weeks 5-6

#### Week 5: Physics Simulation & Gazebo
**Topics**:
- Physics engines (ODE, Bullet, DART)
- Installing Gazebo Classic/Gazebo Sim (Ignition)
- World files, model files, SDF format
- Spawning robots and sensors in simulation

**Hands-On Lab**: Create a custom Gazebo world with obstacles and spawn a mobile robot.

**Reading**: 2 hours | **Lab**: 3 hours

---

#### Week 6: Sensor Simulation
**Topics**:
- LiDAR simulation (ray casting, scan rates, noise)
- Depth cameras (Intel RealSense D435i, Kinect)
- IMU simulation (accelerometer, gyroscope, magnetometer)
- Camera simulation (RGB, distortion models)
- **Assessment 2 Prep**: Gazebo simulation with sensors

**Hands-On Lab**: Add LiDAR, depth camera, and IMU to your robot in Gazebo.

**Reading**: 2 hours | **Lab**: 4 hours

---

### **Module 3: AI-Robot Brain (NVIDIA Isaac)** - Weeks 7-9

#### Week 7: NVIDIA Isaac Sim Setup
**Topics**:
- Installing Isaac Sim (via Omniverse Launcher)
- Connecting Isaac Sim to ROS 2 (ROS Bridge)
- Synthetic data generation (RGB, depth, segmentation masks)
- Domain randomization (textures, lighting, object poses)

**Hands-On Lab**: Generate 1,000 synthetic images of objects on a table with randomized backgrounds.

**Reading**: 2 hours | **Lab**: 4 hours

---

#### Week 8: Navigation (VSLAM & Nav2)
**Topics**:
- Visual SLAM (ORB-SLAM3, RTAB-Map)
- Nav2 navigation stack (costmaps, planners, controllers)
- AMCL localization (particle filters)
- Behavior trees for autonomous navigation

**Hands-On Lab**: Navigate a simulated robot through a warehouse using Nav2.

**Reading**: 2 hours | **Lab**: 4 hours

---

#### Week 9: Sim-to-Real Transfer
**Topics**:
- Why cloud-controlled robots are unsafe (50-200ms latency)
- Proper workflow: Cloud training → Edge deployment
- ONNX/TensorRT model export
- **Assessment 3 Prep**: Isaac Sim-to-Real project

**Hands-On Lab**: Train a depth-based obstacle avoidance model in Isaac Sim, export to TensorRT, deploy to Jetson.

**Reading**: 2 hours | **Lab**: 5 hours

---

### **Module 4: Vision-Language-Action (VLA)** - Weeks 10-13

#### Week 10: Speech & Language Understanding
**Topics**:
- Speech-to-text with Whisper (Tiny, Base, Small models)
- Running Whisper on Jetson Orin Nano (INT8 quantization)
- Handling noisy environments (ReSpeaker mic array)
- Voice activity detection (VAD)

**Hands-On Lab**: Build a voice command node that transcribes "Go forward", "Turn left", "Stop".

**Reading**: 2 hours | **Lab**: 3 hours

---

#### Week 11: LLM-Based Planning
**Topics**:
- Translating natural language → robot actions
- Example: "Clean the room" → [Navigate to room, Detect objects, Pick, Place in bin, Return]
- Using GPT-4/Claude APIs (cloud) vs local models (Llama 3.1)
- Prompt engineering for robot tasks

**Hands-On Lab**: Build an LLM planner that converts "Bring me water" into a sequence of ROS 2 actions.

**Reading**: 2 hours | **Lab**: 4 hours

---

#### Week 12: ROS 2 Integration & Safety
**Topics**:
- Integrating VLA models with ROS 2 action servers
- Latency considerations (cloud APIs vs edge inference)
- Safety protocols (timeout handling, collision avoidance)
- Error recovery (replanning, human intervention)

**Hands-On Lab**: Connect LLM planner to Nav2 action server for autonomous navigation.

**Reading**: 2 hours | **Lab**: 4 hours

---

#### Week 13: Capstone Project
**Topics**:
- End-to-end autonomous humanoid system
- Voice command → LLM planning → ROS 2 actions → Edge inference
- Testing in Isaac Sim before real deployment
- **Assessment 4**: Final capstone presentation

**Hands-On Lab**: Deploy your autonomous humanoid that responds to voice commands and navigates autonomously.

**Reading**: 1 hour | **Lab**: 8 hours

---

## 📝 Assessments

### Assessment 1: ROS 2 Publisher-Subscriber Package (Week 4)
**Objective**: Demonstrate mastery of ROS 2 fundamentals by building a multi-node system.

**Deliverables**:
1. **Package Structure**:
   - `publisher_node.py`: Publishes sensor data (simulated temperature, humidity) at 10 Hz
   - `subscriber_node.py`: Subscribes and logs data to terminal
   - `launch/system.launch.py`: Launches both nodes with parameters
   - `package.xml` and `setup.py` (proper dependencies)

2. **Custom Message Type**:
   - Define `SensorData.msg` with fields: `temperature`, `humidity`, `timestamp`

3. **Documentation**:
   - `README.md` with setup instructions and architecture diagram

**Grading Criteria** (100 points):
- Code Quality (30 pts): PEP 8 compliance, comments, error handling
- Functionality (40 pts): Nodes communicate correctly, launch file works, custom message used
- Documentation (20 pts): Clear README, architecture diagram
- Testing (10 pts): Verify with `ros2 topic echo`, `ros2 node list`

**Submission**: GitHub repository link + 2-minute demo video

---

### Assessment 2: Gazebo Simulation with Sensors (Week 6)
**Objective**: Build a high-fidelity digital twin with multiple sensors in Gazebo.

**Deliverables**:
1. **Gazebo World**:
   - Custom world file with obstacles, walls, and target objects
   - At least 3 room-like structures

2. **Robot Model**:
   - Mobile robot with differential drive (2 wheels)
   - LiDAR sensor (360°, 10m range)
   - Depth camera (Intel RealSense D435i equivalent)
   - IMU sensor

3. **ROS 2 Integration**:
   - Robot publishes `/scan` (LaserScan), `/camera/depth/image` (Image), `/imu` (Imu)
   - Teleop node for manual control

4. **Visualization**:
   - RViz configuration showing all sensor data overlaid on robot model

**Grading Criteria** (100 points):
- World Design (20 pts): Realistic environment, varied obstacles
- Sensor Configuration (30 pts): All 3 sensors working, realistic noise models
- ROS 2 Topics (30 pts): Correct message types, proper coordinate frames (TF2)
- Visualization (20 pts): RViz shows robot + sensors correctly

**Submission**: GitHub repository + 3-minute demo video (screen recording)

---

### Assessment 3: Isaac Sim-to-Real Transfer (Week 9)
**Objective**: Train a model in Isaac Sim, export to TensorRT, and deploy to Jetson Orin Nano.

**Deliverables**:
1. **Synthetic Data Generation**:
   - Generate 5,000 images in Isaac Sim with domain randomization
   - Task: Depth-based obstacle avoidance (detect walls/objects within 1m)

2. **Model Training**:
   - Train a simple CNN (ResNet18) on synthetic data
   - Framework: PyTorch or TensorFlow

3. **Model Export**:
   - Export trained model to ONNX format
   - Convert ONNX → TensorRT (INT8 quantization)

4. **Edge Deployment**:
   - Deploy TensorRT model to Jetson Orin Nano
   - Measure inference latency (target: &lt;50ms per frame)

5. **Testing**:
   - Test in Gazebo simulation (not Isaac Sim) to validate sim-to-real transfer
   - Record success rate (% of episodes without collision)

**Grading Criteria** (100 points):
- Data Generation (20 pts): 5,000+ images, domain randomization applied
- Model Training (25 pts): Converges on validation set (>85% accuracy)
- Model Export (25 pts): ONNX + TensorRT conversion successful
- Edge Deployment (20 pts): Runs on Jetson with &lt;50ms latency
- Testing (10 pts): >70% success rate in Gazebo test environment

**Submission**: GitHub repository + technical report (2-3 pages, PDF) + 5-minute demo video

---

### Assessment 4: VLA Capstone Project (Week 13)
**Objective**: Build an end-to-end autonomous humanoid system with voice commands, LLM planning, and edge inference.

**Deliverables**:
1. **System Architecture**:
   - Voice input (Whisper on Jetson)
   - LLM planning (GPT-4 API or local Llama 3.1)
   - ROS 2 action servers (navigation, manipulation)
   - Edge inference (obstacle avoidance, object detection)

2. **Supported Commands** (minimum 3):
   - "Navigate to the kitchen"
   - "Pick up the red cup"
   - "Return to the charging station"

3. **Isaac Sim Testing**:
   - Test all commands in Isaac Sim before real deployment (if using real robot)
   - Record success rate per command

4. **Safety & Error Handling**:
   - Timeout handling (if action takes >60s, abort and replan)
   - Collision avoidance (emergency stop if obstacle detected)
   - Speech recognition fallback (ask user to repeat if confidence &lt;0.8)

5. **Documentation**:
   - Architecture diagram (Mermaid or draw.io)
   - User guide (how to run the system)
   - Technical report (4-5 pages): design decisions, challenges, results

**Grading Criteria** (100 points):
- System Integration (40 pts): All components work together (voice → LLM → actions → execution)
- Functionality (30 pts): At least 3 commands work with >70% success rate
- Safety (15 pts): Error handling, timeouts, collision avoidance implemented
- Documentation (15 pts): Clear architecture diagram, user guide, technical report

**Submission**: GitHub repository + 10-minute capstone presentation (video or live demo)

---

## 🎓 Grading Scale

| Assessment | Weight | Points |
|------------|--------|--------|
| Assessment 1: ROS 2 Package | 20% | 100 |
| Assessment 2: Gazebo Simulation | 20% | 100 |
| Assessment 3: Sim-to-Real Transfer | 25% | 100 |
| Assessment 4: VLA Capstone | 35% | 100 |
| **Total** | **100%** | **400** |

**Letter Grades**:
- A: 360-400 points (90-100%)
- B: 320-359 points (80-89%)
- C: 280-319 points (70-79%)
- D: 240-279 points (60-69%)
- F: &lt;240 points (&lt;60%)

---

## 🛠️ Required Tools & Software

### All Students
- **Operating System**: Ubuntu 22.04 LTS (native or WSL2)
- **ROS 2**: Humble Hawksbill (LTS release)
- **Python**: 3.10+ (included in Ubuntu 22.04)
- **Simulator**: Gazebo Classic 11 or Gazebo Sim (Ignition Fortress)

### For Isaac Sim (Weeks 7-9)
- **Option 1**: Local RTX workstation (RTX 4070+, 12GB+ VRAM)
- **Option 2**: AWS g5.xlarge ($1.006/hour, A10G GPU, 24GB VRAM)

### For Edge Deployment (Weeks 9-13)
- **Recommended**: Jetson Orin Nano Developer Kit ($249)
- **Alternative**: Jetson Nano/Xavier NX (older, lower performance)

---

## 📚 Recommended Resources

### Books
- *Programming Robots with ROS* by Morgan Quigley (O'Reilly)
- *Probabilistic Robotics* by Thrun, Burgard, Fox (MIT Press)
- *A Gentle Introduction to ROS 2* by Jason M. O'Kane (free PDF)

### Online Documentation
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](https://gazebosim.org/docs)
- [NVIDIA Isaac Sim Docs](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Nav2 Documentation](https://navigation.ros.org/)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [NVIDIA Isaac Sim Forums](https://forums.developer.nvidia.com/c/omniverse/isaac-sim/)

---

## 🚀 Ready to Start?

Begin with **[Module 1: ROS 2 Fundamentals](/docs/ros2)** and work through each week sequentially. No hardware required until Week 5!

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>🎯 Your Learning Path</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    13 weeks • 4 hands-on assessments • End-to-end Physical AI mastery
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/ros2"
      style={{marginRight: '1rem'}}
    >
      Start Module 1: ROS 2 →
    </a>
    <a
      className="button button--secondary button--lg"
      href="/docs/hardware"
    >
      Review Hardware Options →
    </a>
  </div>
</div>
