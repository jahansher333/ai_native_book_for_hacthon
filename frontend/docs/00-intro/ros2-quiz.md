---
id: ros2-comprehensive-quiz
title: "ROS 2 Fundamentals: Comprehensive Module Quiz"
sidebar_label: "ROS 2 Quiz"
sidebar_position: 1
description: "Comprehensive assessment covering all ROS 2 topics: nodes, topics, services, actions, rclpy, and URDF. Includes 5 MCQs and 2 hands-on coding exercises."
keywords: [ros2, quiz, assessment, mcq, exercises, nodes, topics, services, actions, urdf, rclpy]
---

# ROS 2 Fundamentals: Comprehensive Module Quiz

Test your understanding of **ROS 2** (Robot Operating System 2) with this comprehensive assessment covering all key concepts from Module 1.

**Topics Covered**:
- Nodes (independent processes in robot systems)
- Topics (publish-subscribe communication)
- Services (request-response pattern)
- Actions (long-running tasks with feedback)
- rclpy (Python client library)
- URDF (robot descriptions)

**Format**:
- **5 Multiple-Choice Questions** (test conceptual understanding)
- **2 Hands-On Coding Exercises** (apply what you learned)

---

## Multiple-Choice Questions

### Question 1: ROS 2 Nodes [EASY]

**What is a ROS 2 node?**

A) A configuration file that defines robot parameters
B) An independent process that performs a specific task (e.g., camera driver, motion planner)
C) A server that manages all ROS 2 processes in the system
D) A message type used for communication between components

---

### Question 2: Topics vs. Services [MEDIUM]

**When should you use the publish-subscribe (topic) pattern instead of services?**

A) One-time calculations like adding two numbers
B) Continuous data streams like sensor readings (camera frames at 30 FPS, LiDAR scans at 10 Hz)
C) Long-running tasks that need progress feedback (like navigation)
D) Robot parameter configuration at startup

---

### Question 3: Service Definitions [MEDIUM]

**How do you define a ROS 2 service in a `.srv` file?**

A) Only specify the request fields (response is automatic)
B) Request fields, then `---`, then response fields
C) Response fields, then `---`, then request fields
D) All fields in a single block (no separator)

**Example**:
```
int64 a
int64 b
---
int64 sum
```

---

### Question 4: Actions vs. Services [HARD]

**Why would you use an action instead of a service for a robot navigation task?**

A) Actions are faster than services (&lt;1ms response time)
B) Actions provide progress feedback during execution and can be canceled mid-task
C) Actions don't require defining `.action` files (simpler to use)
D) Actions work over WiFi, services require Ethernet

**Context**: Navigation from Point A to Point B takes 30 seconds. You want to show progress (% complete) and allow user to cancel if needed.

---

### Question 5: URDF Purpose [EASY]

**What does URDF (Unified Robot Description Format) define?**

A) The communication protocol between ROS 2 nodes
B) Robot geometry (links, joints, coordinate frames) for visualization and kinematics
C) The Python API for creating ROS 2 nodes
D) Quality of Service (QoS) policies for topics

**Example use**: Defining a 2-link robot arm with shoulder and elbow joints for visualization in RViz.

---

## Hands-On Coding Exercises

### Exercise 1: Modified Sensor Publisher

**Task**: Modify the standard ROS 2 talker node to publish **sensor data** (temperature and humidity) at **10 Hz** instead of text strings.

**Requirements**:
1. Create a custom message type `SensorData.msg` with fields: `float32 temperature`, `float32 humidity`, `int64 timestamp`
2. Publish `SensorData` messages at 10 Hz (every 100ms)
3. Use random values: temperature 15-35°C, humidity 30-90%
4. Include timestamp (use `self.get_clock().now().nanoseconds`)

**Starter Code**:

```python
import rclpy
from rclpy.node import Node
import random

# TODO: Import your custom message type
# from your_package.msg import SensorData

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # TODO: Create publisher for SensorData on topic '/sensor/data'
        # Hint: self.create_publisher(SensorData, '/sensor/data', 10)

        # TODO: Create timer for 10 Hz publishing (0.1 second period)
        # Hint: self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Sensor publisher started')

    def timer_callback(self):
        # TODO: Create SensorData message
        msg = SensorData()

        # TODO: Fill message fields
        # msg.temperature = random.uniform(15.0, 35.0)
        # msg.humidity = random.uniform(30.0, 90.0)
        # msg.timestamp = self.get_clock().now().nanoseconds

        # TODO: Publish message
        # self.publisher.publish(msg)

        self.get_logger().info(f'Publishing: temp={msg.temperature:.1f}°C, humidity={msg.humidity:.1f}%')

def main():
    rclpy.init()
    node = SensorPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Success Criteria**:
- [ ] Publisher sends messages at 10 Hz (verify with `ros2 topic hz /sensor/data`)
- [ ] Messages contain temperature (15-35°C) and humidity (30-90%)
- [ ] Timestamp is included and increases monotonically
- [ ] Logger prints values every 100ms

**Hints**:
1. Define `SensorData.msg` in your package's `msg/` directory
2. Run `colcon build` after creating message file (rebuilds package)
3. Test with: `ros2 topic echo /sensor/data` (should show 10 messages/second)

**Estimated Time**: 25-35 minutes

---

### Exercise 2: Filtered Subscriber

**Task**: Implement a ROS 2 subscriber that **filters messages by topic name pattern**. Subscribe to all topics matching `/sensor/*` and log their data, but ignore messages from other topics.

**Requirements**:
1. Subscribe to topic `/sensor/data` (from Exercise 1)
2. Create a callback that only processes messages if topic name starts with `/sensor/`
3. Log message content: `[INFO] Received from /sensor/data: temp=25.3°C`
4. Ignore messages from topics like `/other/topic`

**Starter Code**:

```python
import rclpy
from rclpy.node import Node

# TODO: Import your custom message type
# from your_package.msg import SensorData

class FilteredSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')

        # TODO: Create subscriber for SensorData on topic '/sensor/data'
        # Hint: self.create_subscription(SensorData, '/sensor/data', self.callback, 10)

        self.get_logger().info('Filtered subscriber started (listening to /sensor/*)')

    def callback(self, msg):
        # TODO: Get topic name (hint: self.get_name() for node name, but topic name is fixed in this example)
        topic_name = '/sensor/data'  # In real scenario, you'd get this from subscription metadata

        # TODO: Check if topic starts with '/sensor/'
        if topic_name.startswith('/sensor/'):
            # TODO: Log message data
            self.get_logger().info(
                f'Received from {topic_name}: '
                f'temp={msg.temperature:.1f}°C, '
                f'humidity={msg.humidity:.1f}%'
            )
        else:
            # Ignore messages from non-/sensor/* topics
            pass

def main():
    rclpy.init()
    node = FilteredSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Success Criteria**:
- [ ] Subscriber receives messages from `/sensor/data` (should match 10 Hz rate)
- [ ] Logger prints: `Received from /sensor/data: temp=25.3°C, humidity=68.2%`
- [ ] Filter logic correctly identifies `/sensor/*` pattern
- [ ] (Advanced) Works with multiple sensor topics: `/sensor/temp`, `/sensor/humidity`

**Hints**:
1. Run Exercise 1's publisher first: `python3 sensor_publisher.py`
2. In another terminal, run this subscriber: `python3 filtered_subscriber.py`
3. You should see messages at 10 Hz matching the publisher's output
4. Test filter by publishing to `/other/topic` (subscriber should ignore)

**Estimated Time**: 20-30 minutes

---

## Answers

<details>
<summary>Click to reveal answers and explanations</summary>

### MCQ Answers

**Question 1: B - An independent process that performs a specific task**

**Rationale**: Nodes are the fundamental building blocks of ROS 2 systems. Each node is an independent process (separate OS process with its own PID) that performs a specific function like:
- Camera driver node (publishes image data)
- Motion planner node (computes navigation paths)
- Controller node (executes motor commands)

Nodes communicate via topics, services, and actions. A typical robot system has 10-50 nodes running simultaneously.

---

**Question 2: B - Continuous data streams like sensor readings**

**Rationale**: The publish-subscribe (topic) pattern is ideal for:
- **Continuous data**: Sensor streams (camera at 30 FPS, LiDAR at 10 Hz)
- **One-to-many**: Multiple subscribers can listen to same topic
- **Asynchronous**: Publisher doesn't wait for subscribers

**When NOT to use topics**:
- One-time calculations → Use services (request-response)
- Long-running tasks with feedback → Use actions
- Parameters at startup → Use ROS 2 parameters

**Example**: Camera publishes frames to `/camera/image` topic. Multiple nodes subscribe (object detector, visualizer, logger) without camera knowing who's listening.

---

**Question 3: B - Request fields, then `---`, then response fields**

**Rationale**: `.srv` file format:
```
# Request (above separator)
int64 a
int64 b
---
# Response (below separator)
int64 sum
```

The `---` separator divides request (client sends) from response (server returns). This is different from:
- **Messages** (`.msg`): No separator, just fields
- **Actions** (`.action`): Three sections (goal, result, feedback)

---

**Question 4: B - Actions provide progress feedback during execution and can be canceled mid-task**

**Rationale**: Actions are designed for long-running tasks:
- **Feedback**: Server sends progress updates ("50% complete, currently at x=5.2, y=3.1")
- **Cancellation**: Client can cancel mid-execution ("user pressed stop button")
- **Result**: Server returns final outcome ("navigation succeeded/failed")

**Comparison**:
- **Service**: Blocks until complete, no progress feedback, can't cancel
- **Action**: Non-blocking, progress updates, cancellable

**Example**: Navigation takes 30 seconds. Action provides:
- Feedback every 1 second: "Current position: (x, y), 60% complete"
- User can cancel if robot going wrong direction
- Final result: "Reached goal" or "Aborted (obstacle detected)"

---

**Question 5: B - Robot geometry (links, joints, coordinate frames)**

**Rationale**: URDF (Unified Robot Description Format) describes:
- **Links**: Rigid bodies (robot arm segments, wheels, base)
- **Joints**: Connections between links (revolute for rotation, prismatic for linear)
- **Coordinate frames**: TF tree (base_link → shoulder → elbow → wrist → gripper)
- **Visual/Collision geometry**: Meshes for visualization (RViz) and collision checking

**Example**: 2-link arm URDF:
```xml
<link name="base_link"/>
<link name="link1"/>
<link name="link2"/>

<joint name="shoulder" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <axis xyz="0 0 1"/>  <!-- Rotates around Z-axis -->
</joint>

<joint name="elbow" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
</joint>
```

URDF is NOT:
- Communication protocol (that's DDS)
- Python API (that's rclpy)
- QoS policies (that's topic-level configuration)

---

### Exercise Solutions

#### Exercise 1 Solution: Sensor Publisher

**Complete Implementation**:

```python
# First, create msg/SensorData.msg in your package:
# float32 temperature
# float32 humidity
# int64 timestamp

import rclpy
from rclpy.node import Node
import random
from your_package.msg import SensorData  # Replace with your package name

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            SensorData,
            '/sensor/data',
            10  # Queue size
        )

        # Create timer for 10 Hz (0.1 second period)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Sensor publisher started')

    def timer_callback(self):
        msg = SensorData()

        # Generate random sensor values
        msg.temperature = random.uniform(15.0, 35.0)
        msg.humidity = random.uniform(30.0, 90.0)
        msg.timestamp = self.get_clock().now().nanoseconds

        # Publish
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Publishing: temp={msg.temperature:.1f}°C, '
            f'humidity={msg.humidity:.1f}%'
        )

def main():
    rclpy.init()
    node = SensorPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Points**:
- `create_timer(0.1, ...)` → 10 Hz (1/0.1 = 10)
- `random.uniform(15.0, 35.0)` → Temperature in realistic range
- `self.get_clock().now().nanoseconds` → ROS 2 time (not system time)

**Verification**:
```bash
# Terminal 1: Run publisher
python3 sensor_publisher.py

# Terminal 2: Check topic rate
ros2 topic hz /sensor/data
# Should show: average rate: 10.000

# Terminal 3: Echo messages
ros2 topic echo /sensor/data
```

---

#### Exercise 2 Solution: Filtered Subscriber

**Complete Implementation**:

```python
import rclpy
from rclpy.node import Node
from your_package.msg import SensorData

class FilteredSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_subscriber')

        # Subscribe to /sensor/data
        self.subscription = self.create_subscription(
            SensorData,
            '/sensor/data',
            self.callback,
            10
        )

        self.get_logger().info('Filtered subscriber started (listening to /sensor/*)')

    def callback(self, msg):
        # In this simplified version, topic name is fixed
        # In advanced scenarios, you'd iterate over all subscriptions
        topic_name = '/sensor/data'

        if topic_name.startswith('/sensor/'):
            self.get_logger().info(
                f'Received from {topic_name}: '
                f'temp={msg.temperature:.1f}°C, '
                f'humidity={msg.humidity:.1f}%'
            )
        else:
            pass  # Ignore non-/sensor/* topics

def main():
    rclpy.init()
    node = FilteredSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Advanced: Subscribe to Multiple Topics Dynamically**:

```python
# For advanced learners: subscribe to ALL /sensor/* topics
import rclpy
from rclpy.node import Node
from your_package.msg import SensorData

class DynamicFilteredSubscriber(Node):
    def __init__(self):
        super().__init__('dynamic_filtered_subscriber')

        # Get list of all topics
        topic_names_and_types = self.get_topic_names_and_types()

        # Filter for /sensor/* topics with SensorData type
        for topic_name, topic_types in topic_names_and_types:
            if topic_name.startswith('/sensor/'):
                self.create_subscription(
                    SensorData,
                    topic_name,
                    lambda msg, tn=topic_name: self.callback(msg, tn),
                    10
                )
                self.get_logger().info(f'Subscribed to {topic_name}')

    def callback(self, msg, topic_name):
        self.get_logger().info(
            f'Received from {topic_name}: '
            f'temp={msg.temperature:.1f}°C, '
            f'humidity={msg.humidity:.1f}%'
        )
```

**Key Points**:
- `startswith('/sensor/')` → Pattern matching
- `lambda msg, tn=topic_name: ...` → Capture topic name in callback
- Dynamic subscription → Adapts to new topics added at runtime

---

</details>

---

## Additional Practice

Want more practice? Try these challenges:

1. **Service Exercise**: Create an `AddTwoInts` service (server adds two numbers, client calls it)
2. **Action Exercise**: Implement a `Countdown` action (goal: duration, feedback: remaining seconds)
3. **URDF Exercise**: Define a 3-DOF robot arm in URDF and visualize in RViz
4. **Integration**: Combine publisher, subscriber, and service in one package

**Resources**:
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Example Code: https://github.com/ros2/examples

---

## Grading Rubric (Self-Assessment)

| Criteria | Points | Your Score |
|----------|--------|------------|
| **MCQ Accuracy** (5 questions × 2 points) | 10 | /10 |
| **Exercise 1 Functionality** (publishes at 10 Hz, correct data) | 15 | /15 |
| **Exercise 1 Code Quality** (clean code, comments, error handling) | 10 | /10 |
| **Exercise 2 Functionality** (filters correctly, subscribes) | 15 | /15 |
| **Exercise 2 Code Quality** (clean code, logging) | 10 | /10 |
| **TOTAL** | **60** | **/60** |

**Passing**: 42/60 (70%)
**Excellent**: 54/60 (90%)

---

## Next Steps

1. **Review Incorrect Answers**: Re-read relevant chapter sections
2. **Complete Both Exercises**: Hands-on practice solidifies concepts
3. **Move to Module 2**: [Simulation Environments (Gazebo & Unity)](/docs/02-gazebo-unity/index)
4. **Join Discussions**: Ask questions in [GitHub Discussions](https://github.com/your-repo/discussions)

---

<!-- Generated by @quiz-generator on December 6, 2025 -->
