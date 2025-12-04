---
id: capstone
title: Capstone Project - Autonomous Humanoid
sidebar_label: Capstone Project
sidebar_position: 5
description: Build an end-to-end autonomous humanoid with voice commands, LLM planning, and edge inference
keywords: [capstone, autonomous-humanoid, vla, end-to-end, final-project]
---

# Capstone Project: Autonomous Humanoid

## Project Overview

Build a **complete autonomous humanoid system** that understands voice commands and executes complex tasks using Vision-Language-Action models.

### Required Features

1. **Voice Input**: Whisper speech-to-text (Jetson edge)
2. **LLM Planning**: GPT-4 or Llama 3.1 (cloud or local)
3. **Object Detection**: YOLO (Jetson edge)
4. **Navigation**: Nav2 autonomous navigation
5. **Safety**: Timeout handling, collision avoidance, error recovery

### Supported Commands (Minimum 3)

1. **"Navigate to the kitchen"** → Autonomous navigation to waypoint
2. **"Pick up the red cup"** → Detect, approach, pick object
3. **"Return to the charging station"** → Navigate home

**Bonus** (optional):
- "Bring me water" (multi-step: navigate, detect, pick, return, place)
- "Clean the room" (detect all trash, pick each, place in bin)

---

## System Architecture

```mermaid
graph TD
    A[Voice Command<br/>"Bring me water"] --> B[Whisper STT<br/>Jetson Edge<br/>5-10ms]
    B --> C{LLM Planner<br/>GPT-4 Cloud<br/>50-200ms OK}

    C --> D[Action Plan<br/>JSON Sequence]

    D --> E[Action Executor<br/>ROS 2 Edge]

    F[Camera Feed<br/>30 FPS] --> G[YOLO Detector<br/>Jetson Edge<br/>20ms]
    G --> E

    H[LiDAR Scan<br/>10 Hz] --> I[Safety Monitor<br/>Jetson Edge<br/>&lt;10ms]
    I --> E

    E --> J[Nav2<br/>Navigation]
    E --> K[MoveIt<br/>Manipulation]

    J --> L[Motor Commands<br/>&lt;10ms Latency]
    K --> L

    L --> M[Real Robot]

    style B fill:#00ff00
    style G fill:#00ff00
    style I fill:#00ff00
    style L fill:#00ff00
    style C fill:#87CEEB

    I -.->|Emergency Stop| L
```

**Color Key**:
- **Green**: Edge inference (Jetson, &lt;10ms)
- **Blue**: Cloud API (acceptable for planning only)

**Critical Safety**: All real-time perception and control runs on edge. LLM planning can use cloud (infrequent, high-level decisions).

---

## Implementation Steps

### Step 1: Voice Command Node

```python
# voice_command_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.command_pub = self.create_publisher(String, '/voice_command', 10)

        # Load Whisper Small model
        self.model = whisper.load_model("small")

        # Setup microphone
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        # Timer to check for voice activity
        self.create_timer(1.0, self.check_voice_activity)

        self.get_logger().info('Voice command node ready (say a command)')

    def check_voice_activity(self):
        # Simple VAD (voice activity detection)
        frames = []
        for _ in range(5):
            data = self.stream.read(1024, exception_on_overflow=False)
            frames.append(data)

        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        energy = np.abs(audio_data).mean()

        if energy > 500:  # Threshold
            self.get_logger().info('Voice detected! Recording...')
            full_audio = self.record_audio(duration=3)

            result = self.model.transcribe(full_audio, language='en')
            text = result["text"].strip()

            if text:
                self.get_logger().info(f'Transcribed: {text}')

                msg = String()
                msg.data = text
                self.command_pub.publish(msg)

    def record_audio(self, duration=3):
        frames = []
        for _ in range(0, int(16000 / 1024 * duration)):
            data = self.stream.read(1024, exception_on_overflow=False)
            frames.append(data)

        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        return audio_data.astype(np.float32) / 32768.0

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 2: LLM Planner Node

```python
# llm_planner_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.msg import ActionSequence, Action
import openai
import json

SYSTEM_PROMPT = """
You are an autonomous robot planner.

**Capabilities**:
- navigate: Move to location (params: target)
  Locations: kitchen, bedroom, living_room, charging_station, trash_bin
- detect_objects: Find objects (params: class)
  Classes: cup, bottle, water, trash, book
- pick: Pick object (params: object_id)
- place: Place object (params: location)

**Rules**:
1. Navigate before manipulating
2. Detect before picking
3. Return to charging_station when done
4. Return JSON array only

Example:
User: "Bring me water"
[
  {"action": "navigate", "target": "kitchen"},
  {"action": "detect_objects", "class": "water"},
  {"action": "pick", "object_id": "water_1"},
  {"action": "navigate", "target": "user"},
  {"action": "place", "location": "table"},
  {"action": "navigate", "target": "charging_station"}
]
"""

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        self.plan_pub = self.create_publisher(ActionSequence, '/action_plan', 10)

        openai.api_key = "sk-..."  # Your API key

        self.get_logger().info('LLM planner ready')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Planning for: {command}')

        actions = self.plan_task(command)

        plan_msg = ActionSequence()
        for action_dict in actions:
            action = Action()
            action.action_type = action_dict["action"]
            action.parameters = json.dumps(action_dict)
            plan_msg.actions.append(action)

        self.plan_pub.publish(plan_msg)
        self.get_logger().info(f'Published plan with {len(actions)} actions')

    def plan_task(self, command):
        response = openai.ChatCompletion.create(
            model="gpt-4-turbo",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": command}
            ],
            temperature=0.0
        )

        return json.loads(response.choices[0].message.content)

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

### Step 3: Action Executor Node

*(Use code from ROS Integration chapter)*

---

### Step 4: Safety Monitor Node

```python
# safety_monitor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.emergency_distance = 0.3  # meters

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)

        if min_distance < self.emergency_distance:
            self.get_logger().error(f'EMERGENCY STOP! Obstacle at {min_distance:.2f}m')

            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

def main():
    rclpy.init()
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

### Step 5: Launch File

```python
# autonomous_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Voice input
        Node(
            package='my_robot',
            executable='voice_command_node',
            output='screen'
        ),

        # LLM planner
        Node(
            package='my_robot',
            executable='llm_planner_node',
            output='screen'
        ),

        # Action executor
        Node(
            package='my_robot',
            executable='action_executor_node',
            output='screen'
        ),

        # Safety monitor
        Node(
            package='my_robot',
            executable='safety_monitor_node',
            output='screen'
        ),
    ])
```

---

## Testing Protocol

### Test Case 1: Simple Navigation

**Command**: "Navigate to the kitchen"

**Expected Behavior**:
1. Whisper transcribes command
2. LLM generates: `[{"action": "navigate", "target": "kitchen"}]`
3. Nav2 navigates to kitchen waypoint
4. Robot arrives within 0.5m of target

**Success Criteria**: Robot reaches kitchen within 30 seconds

---

### Test Case 2: Object Pick

**Command**: "Pick up the red cup"

**Expected Behavior**:
1. LLM generates: `[{"action": "detect_objects", "class": "cup"}, {"action": "pick", "object_id": "cup_1"}]`
2. YOLO detects red cup
3. Robot approaches and picks cup
4. Gripper closes successfully

**Success Criteria**: Cup is grasped securely

---

### Test Case 3: Emergency Stop

**Setup**: Place obstacle 0.2m in front of moving robot

**Expected Behavior**:
1. LiDAR detects obstacle
2. Safety monitor publishes emergency stop
3. Robot stops within 0.1m

**Success Criteria**: No collision, robot stops immediately

---

## Deliverables

### 1. Source Code

- `voice_command_node.py`
- `llm_planner_node.py`
- `action_executor_node.py`
- `safety_monitor_node.py`
- `autonomous_humanoid.launch.py`

### 2. Architecture Diagram

Mermaid diagram showing all components (see above for reference).

### 3. User Guide

**README.md** with:
- System requirements
- Installation instructions
- Launch commands
- Supported voice commands
- Troubleshooting

### 4. Demo Video (10 minutes)

Show:
- System startup (all nodes launching)
- 3 successful voice commands
- Error recovery (obstacle avoidance)
- System architecture explanation (2 min)

### 5. Technical Report (4-5 pages)

**Sections**:
1. **Introduction**: Problem statement, approach
2. **System Design**: Architecture, component descriptions
3. **Implementation**: Key algorithms, ROS 2 integration
4. **Results**: Success rates, latency measurements
   - Command recognition accuracy: X%
   - Navigation success rate: Y%
   - Average inference latency: Z ms
5. **Challenges & Solutions**: Problems encountered, how solved
6. **Conclusion**: Lessons learned, future work

---

## Grading Rubric (100 points)

### System Integration (40 points)

- All components work together (voice → LLM → actions) **(20 pts)**
- Proper ROS 2 topic/action usage **(10 pts)**
- Edge vs cloud separation (perception on Jetson, planning cloud OK) **(10 pts)**

### Functionality (30 points)

- At least 3 commands work with >70% success rate **(20 pts)**
- Graceful error handling (no crashes) **(10 pts)**

### Safety (15 points)

- Timeout handling implemented **(5 pts)**
- Collision avoidance works **(5 pts)**
- Emergency stop responds &lt;1 second **(5 pts)**

### Documentation (15 points)

- Clear architecture diagram **(5 pts)**
- Complete user guide **(5 pts)**
- Technical report (design, results, challenges) **(5 pts)**

---

## Submission

**Deadline**: End of Week 13

**Format**:
- GitHub repository (public or invite instructors)
- Demo video (YouTube/Vimeo link)
- Technical report (PDF)

**README.md must include**:
- System requirements
- Installation steps
- Launch commands
- Demo video link
- Report link

---

## Bonus Challenges (Optional)

1. **Multi-Robot Coordination** (+10 pts)
   - Command 2 robots simultaneously
   - Coordinate tasks (one fetches, one delivers)

2. **Advanced Manipulation** (+10 pts)
   - Pour water into cup
   - Stack objects

3. **Conversational Interface** (+5 pts)
   - Robot asks clarifying questions
   - User: "Bring me something to drink"
   - Robot: "Water or juice?"

---

## Key Takeaways

✅ **End-to-end VLA system** (voice, vision, language, action)
✅ **Proper edge/cloud separation** (perception edge, planning cloud OK)
✅ **Safety-first design** (timeouts, collision avoidance)
✅ **Real-world deployment** (Jetson, ROS 2, production-ready)

---

## Congratulations!

You've completed the **Physical AI & Humanoid Robotics** course! You now have the skills to:
- Build ROS 2 robot systems
- Create digital twins in Gazebo/Isaac Sim
- Generate synthetic training data
- Deploy AI models to edge devices
- Integrate LLMs with robots

**What's next?**
- Build your own robot projects
- Contribute to open-source robotics
- Join the Physical AI community
- Apply for robotics/AI positions

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>🎉 Course Complete!</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    You've mastered the complete Physical AI stack—from digital twins to autonomous humanoids.
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="https://github.com/jahansher333/Ai_Native_Books_Pyhsical_Ai"
      style={{marginRight: '1rem'}}
    >
      ⭐ Star the Repo
    </a>
    <a
      className="button button--secondary button--lg"
      href="/docs/intro"
    >
      Back to Introduction →
    </a>
  </div>
</div>
