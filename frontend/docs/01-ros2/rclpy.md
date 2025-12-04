---
id: rclpy
title: rclpy - Python Client Library
sidebar_label: rclpy
sidebar_position: 6
description: Master the Python client library for ROS 2 - timers, parameters, and lifecycle management
keywords: [rclpy, ros2-python, timers, parameters, lifecycle, executor]
---

# rclpy - Python Client Library

## Introduction

**rclpy** is the official Python client library for ROS 2. It provides Pythonic APIs for all ROS 2 features: nodes, topics, services, actions, parameters, and more.

This chapter covers advanced `rclpy` patterns you'll use in production robot systems.

---

## Timers

Timers execute callbacks at fixed intervals—perfect for periodic tasks like sensor publishing or state updates.

### Basic Timer

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Timer fires every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer fired {self.counter} times')

def main():
    rclpy.init()
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### One-Shot Timer

```python
def delayed_task(self):
    self.get_logger().info('Delayed task executed!')

# Execute once after 5 seconds
self.one_shot_timer = self.create_timer(5.0, self.delayed_task)

def delayed_task(self):
    self.get_logger().info('Delayed task executed!')
    self.one_shot_timer.cancel()  # Prevent future executions
```

---

## Parameters

Parameters allow runtime configuration without code changes.

### Declaring Parameters

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('use_lidar', True)

        # Read parameters
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        use_lidar = self.get_parameter('use_lidar').value

        self.get_logger().info(f'Robot: {robot_name}')
        self.get_logger().info(f'Max speed: {max_speed} m/s')
        self.get_logger().info(f'Use LiDAR: {use_lidar}')
```

### Setting Parameters from CLI

```bash
# Set parameters when launching node
ros2 run my_package parameter_node --ros-args \
  -p robot_name:=atlas \
  -p max_speed:=2.5 \
  -p use_lidar:=false
```

### Dynamic Parameter Updates

```python
from rcl_interfaces.msg import ParameterDescriptor

class DynamicParameterNode(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare parameter with descriptor
        descriptor = ParameterDescriptor(
            description='Maximum robot velocity in m/s',
            read_only=False
        )
        self.declare_parameter('max_speed', 1.0, descriptor)

        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                if param.value > 3.0:
                    self.get_logger().warn('Max speed too high! Clamping to 3.0')
                    return False  # Reject change
                self.get_logger().info(f'Max speed updated to {param.value}')

        return True  # Accept changes
```

### Update Parameters at Runtime

```bash
ros2 param set /dynamic_parameter_node max_speed 2.5
```

---

## Clock and Time

ROS 2 provides simulation-compatible time APIs.

### Getting Current Time

```python
class TimeNode(Node):
    def __init__(self):
        super().__init__('time_node')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Get current ROS time (works in simulation and real time)
        current_time = self.get_clock().now()

        self.get_logger().info(f'Time: {current_time.seconds_nanoseconds()}')
```

### Duration and Rate

```python
from rclpy.duration import Duration

# Create duration (5 seconds)
duration = Duration(seconds=5)

# Check if enough time has passed
start_time = self.get_clock().now()
# ... do work ...
elapsed = self.get_clock().now() - start_time

if elapsed > duration:
    self.get_logger().info('Task took too long!')
```

---

## Callbacks and Executors

### Callback Groups

For parallel execution of callbacks:

```python
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class CallbackGroupNode(Node):
    def __init__(self):
        super().__init__('callback_group_node')

        # Reentrant: callbacks can run in parallel
        reentrant_group = ReentrantCallbackGroup()

        # Mutually exclusive: callbacks run one at a time
        exclusive_group = MutuallyExclusiveCallbackGroup()

        # Fast timer (can run in parallel with itself)
        self.create_timer(0.1, self.fast_callback, callback_group=reentrant_group)

        # Slow service (must run alone)
        self.create_service(
            AddTwoInts,
            'add',
            self.service_callback,
            callback_group=exclusive_group
        )

    def fast_callback(self):
        # Quick operation
        pass

    def service_callback(self, request, response):
        # Slow operation (database query, file I/O)
        time.sleep(2.0)
        return response
```

### Multi-Threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor

def main():
    rclpy.init()

    node = CallbackGroupNode()

    # Use 4 threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

---

## Context Managers

For cleaner resource management:

```python
import rclpy
from rclpy.node import Node

class ResourceNode(Node):
    def __init__(self):
        super().__init__('resource_node')

        # Open file, connect to database, etc.
        self.file = open('/tmp/data.log', 'w')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Cleanup
        self.file.close()
        self.get_logger().info('Resources cleaned up')

def main():
    rclpy.init()

    with ResourceNode() as node:
        rclpy.spin(node)

    rclpy.shutdown()
```

---

## Logging Levels

Configure logging verbosity:

```python
class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')

        # Set log level for this node
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().debug('Debug message (verbose)')
        self.get_logger().info('Info message (normal)')
        self.get_logger().warn('Warning message (important)')
        self.get_logger().error('Error message (failure)')
        self.get_logger().fatal('Fatal message (crash imminent)')
```

### Set Logging Level from CLI

```bash
ros2 run my_package logging_node --ros-args --log-level DEBUG
```

---

## Spinning Strategies

### spin() - Block Forever

```python
rclpy.spin(node)  # Blocks until Ctrl+C
```

### spin_once() - Process One Callback

```python
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    # Do other work here
```

### spin_until_future_complete() - Wait for Result

```python
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

if future.done():
    result = future.result()
```

---

## Real-World Example: Robot Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('obstacle_distance', 0.5)

        # Subscribers
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            lidar_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Check for obstacles within threshold
        min_distance = min(msg.ranges)
        threshold = self.get_parameter('obstacle_distance').value

        self.obstacle_detected = min_distance < threshold

    def control_loop(self):
        twist = Twist()

        if self.obstacle_detected:
            # Stop if obstacle detected
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn in place
            self.get_logger().warn('Obstacle detected! Turning...')
        else:
            # Move forward
            max_speed = self.get_parameter('max_linear_speed').value
            twist.linear.x = max_speed
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before exiting
        stop_twist = Twist()
        controller.cmd_vel_pub.publish(stop_twist)

        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Hands-On Lab: Configurable Publisher

**Goal**: Create a publisher with configurable topic name, message type, and rate.

### Requirements

1. Parameters: `topic_name`, `publish_rate`, `message_prefix`
2. Publish `String` messages with format: `"{prefix}: count={counter}"`
3. Update `message_prefix` dynamically via `ros2 param set`

### Starter Code

```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # TODO: Declare parameters with defaults
        # TODO: Read parameters
        # TODO: Create publisher
        # TODO: Create timer (rate from parameter)
        # TODO: Register parameter callback

        self.counter = 0

    def timer_callback(self):
        # TODO: Get current prefix from parameter
        # TODO: Create message
        # TODO: Publish
        # TODO: Increment counter
        pass

    def parameter_callback(self, params):
        # TODO: Log parameter changes
        return True
```

### Expected Behavior

```bash
# Terminal 1
python3 configurable_publisher.py

# Terminal 2
ros2 topic echo /my_topic

# Terminal 3
ros2 param set /configurable_publisher message_prefix "Robot"
```

**Output**:
```
data: 'Hello: count=1'
data: 'Hello: count=2'
data: 'Robot: count=3'  # After parameter change
data: 'Robot: count=4'
```

---

## Key Takeaways

✅ **Timers** for periodic tasks (sensor polling, control loops)
✅ **Parameters** for runtime configuration (no code changes)
✅ **QoS policies** for network behavior (reliability, durability)
✅ **Callback groups** for parallel execution (multi-threaded)
✅ **Logging levels** for debugging (debug, info, warn, error)

---

## Next Steps

Learn how to describe robots with **[URDF](/docs/ros2/urdf)**.

---

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <p style={{fontSize: '1.1rem', marginTop: '1rem'}}>
    Master robot descriptions and coordinate transforms with URDF.
  </p>
  <div style={{marginTop: '2rem'}}>
    <a
      className="button button--primary button--lg"
      href="/docs/ros2/urdf"
    >
      Next: URDF →
    </a>
  </div>
</div>
