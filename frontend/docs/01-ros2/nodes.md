---
id: nodes
title: ROS 2 Nodes
sidebar_label: Nodes
sidebar_position: 2
description: Learn the building blocks of ROS 2 systems - nodes, executors, and lifecycle management
keywords: [ros2-nodes, rclpy, executors, lifecycle, distributed-systems]
---

import PlatformNote from '@site/src/components/PlatformNote';

# ROS 2 Nodes

## What Are Nodes?

**Nodes** are the fundamental building blocks of ROS 2 systems. Each node is an independent process that performs a specific task—like reading sensor data, planning paths, or controlling motors.

Think of nodes as **microservices** in a robot:
- **Camera Node**: Captures images and publishes them
- **Object Detection Node**: Subscribes to images, publishes detected objects
- **Navigation Node**: Plans paths and sends velocity commands
- **Motor Controller Node**: Subscribes to velocity commands, controls motors

### Why Separate Nodes?

```mermaid
graph LR
    A[Camera Node] -->|Image| B[Object Detector]
    B -->|Detections| C[Navigation Planner]
    C -->|Velocity| D[Motor Controller]
    D --> E[Physical Motors]
```

**Benefits of this architecture**:
1. **Modularity**: Replace camera node without touching navigation code
2. **Scalability**: Run nodes on different machines (edge devices, cloud)
3. **Debuggability**: Test each node independently
4. **Reusability**: Use existing nodes from ROS 2 ecosystem (camera drivers, SLAM, Nav2)

---

## Your First Node

### Minimal Python Node

Create a file `minimal_node.py`:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')  # Node name
        self.get_logger().info('Hello from ROS 2!')

        # Create a timer that fires every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer callback #{self.counter}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = MinimalNode()

    try:
        rclpy.spin(node)  # Keep node alive and process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Node

<PlatformNote
  linux="Run directly: python3 minimal_node.py"
  macos="macOS not officially supported by ROS 2. Use Docker or VM with Ubuntu 22.04."
  wsl2="Run in WSL2 Ubuntu terminal: python3 minimal_node.py"
/>

```bash
# Make executable
chmod +x minimal_node.py

# Run the node
python3 minimal_node.py
```

**Expected output**:
```
[INFO] [1701234567.123456789] [minimal_node]: Hello from ROS 2!
[INFO] [1701234568.123456789] [minimal_node]: Timer callback #1
[INFO] [1701234569.123456789] [minimal_node]: Timer callback #2
...
```

**Stop the node**: Press `Ctrl+C`

---

## Node Introspection

While your node is running, open a new terminal and use ROS 2 CLI tools:

### List All Running Nodes

```bash
ros2 node list
```

**Output**:
```
/minimal_node
```

### Node Information

```bash
ros2 node info /minimal_node
```

**Output**:
```
/minimal_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Service Servers:
    /minimal_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /minimal_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /minimal_node/get_parameters: rcl_interfaces/srv/GetParameters
    /minimal_node/list_parameters: rcl_interfaces/srv/ListParameters
    /minimal_node/set_parameters: rcl_interfaces/srv/SetParameters
    /minimal_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

**Key observations**:
- Every node automatically publishes to `/rosout` (logging)
- Every node provides parameter services (for dynamic configuration)

---

## Node Lifecycle

### Basic Lifecycle (Most Common)

```python
def main():
    rclpy.init()           # 1. Initialize ROS 2 context
    node = MinimalNode()   # 2. Create node instance
    rclpy.spin(node)       # 3. Process callbacks until shutdown
    node.destroy_node()    # 4. Cleanup
    rclpy.shutdown()       # 5. Shutdown ROS 2 context
```

### Managed Lifecycle (Advanced)

For **production systems** that need controlled startup/shutdown (e.g., robot arms):

```python
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn

class ManagedNode(LifecycleNode):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node...')
        # Load configuration files, initialize drivers
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node...')
        # Enable motors, start publishing data
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating node...')
        # Stop motors, pause publishing
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up node...')
        # Release resources
        return TransitionCallbackReturn.SUCCESS
```

**State transitions**:
```
Unconfigured → Inactive → Active → Inactive → Finalized
```

**Use cases**: Camera drivers, motor controllers, safety-critical systems.

---

## Executors

**Executors** control how callbacks are processed. By default, `rclpy.spin()` uses a **single-threaded executor**.

### Single-Threaded Executor (Default)

```python
rclpy.spin(node)  # Processes callbacks sequentially
```

**Behavior**: Callbacks run one at a time. If a callback takes 100ms, other callbacks wait.

**Use case**: Most robots (simple, predictable behavior).

---

### Multi-Threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)

try:
    executor.spin()
finally:
    executor.shutdown()
```

**Behavior**: Callbacks can run in parallel (on different threads).

**Use case**: High-throughput systems (e.g., processing 10 camera feeds simultaneously).

**⚠️ Warning**: Requires thread-safe code (locks, mutexes).

---

## Logging Best Practices

ROS 2 provides 5 log levels (similar to Python's `logging` module):

```python
self.get_logger().debug('Detailed diagnostic info')
self.get_logger().info('General informational messages')
self.get_logger().warn('Warnings (non-critical issues)')
self.get_logger().error('Errors (failures that don't crash the node)')
self.get_logger().fatal('Fatal errors (node cannot continue)')
```

### Viewing Logs

**Terminal output** (default):
```bash
[INFO] [timestamp] [node_name]: Message
```

**ROS 2 logging service**:
```bash
ros2 topic echo /rosout
```

**Filter logs by severity**:
```bash
ros2 run rqt_console rqt_console
```

---

## Real-World Example: Sensor Node

Let's build a node that simulates a temperature sensor:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')

        # Publisher: sends temperature readings
        self.publisher = self.create_publisher(Float32, 'temperature', 10)

        # Timer: publish at 1 Hz (once per second)
        self.timer = self.create_timer(1.0, self.publish_temperature)

        self.get_logger().info('Temperature sensor node started')

    def publish_temperature(self):
        # Simulate sensor reading (20-30°C with noise)
        temp = 25.0 + random.uniform(-5.0, 5.0)

        msg = Float32()
        msg.data = temp

        self.publisher.publish(msg)
        self.get_logger().info(f'Published temperature: {temp:.2f}°C')

def main():
    rclpy.init()
    node = TemperatureSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing the Sensor Node

**Terminal 1** (run the node):
```bash
python3 temperature_sensor.py
```

**Terminal 2** (subscribe to the topic):
```bash
ros2 topic echo /temperature
```

**Output**:
```
data: 27.35
---
data: 22.18
---
data: 29.64
---
```

---

## Hands-On Lab: Build a Heartbeat Node

**Goal**: Create a node that publishes a heartbeat message every 2 seconds.

### Requirements

1. Node name: `heartbeat_node`
2. Topic name: `/heartbeat`
3. Message type: `std_msgs/msg/String`
4. Message content: `"Heartbeat at <timestamp>"`
5. Rate: 0.5 Hz (every 2 seconds)

### Starter Code

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')

        # TODO: Create publisher for /heartbeat topic
        # TODO: Create timer (2 second period)

        self.get_logger().info('Heartbeat node started')

    def publish_heartbeat(self):
        # TODO: Get current timestamp
        # TODO: Create String message
        # TODO: Publish message
        # TODO: Log message
        pass

def main():
    rclpy.init()
    node = HeartbeatNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing

**Terminal 1**:
```bash
python3 heartbeat_node.py
```

**Terminal 2**:
```bash
ros2 topic echo /heartbeat
```

**Expected output**:
```
data: 'Heartbeat at 2025-12-04 10:30:15'
---
data: 'Heartbeat at 2025-12-04 10:30:17'
---
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')

        self.publisher = self.create_publisher(String, 'heartbeat', 10)
        self.timer = self.create_timer(2.0, self.publish_heartbeat)

        self.get_logger().info('Heartbeat node started')

    def publish_heartbeat(self):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        msg = String()
        msg.data = f'Heartbeat at {timestamp}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    node = HeartbeatNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

## Key Takeaways

✅ **Nodes are independent processes** that perform specific tasks
✅ **One node, one responsibility** (camera driver, path planner, motor controller)
✅ **Use `rclpy.spin()`** to keep nodes alive and process callbacks
✅ **Introspect with CLI tools**: `ros2 node list`, `ros2 node info`
✅ **Log effectively**: Use appropriate log levels (info, warn, error)

---

## Next Steps

Now that you understand nodes, learn how they communicate using **[Topics](/docs/ros2/topics)** in the next chapter.

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    Master publisher-subscriber communication patterns with ROS 2 topics.
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/ros2/topics"
    >
      Next: Topics →
    </a>
  </div>
</div>
