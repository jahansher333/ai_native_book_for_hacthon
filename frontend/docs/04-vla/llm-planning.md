---
id: llm-planning
title: LLM-Based Action Planning
sidebar_label: LLM Planning
sidebar_position: 3
description: Use GPT-4 or Llama 3.1 to convert natural language commands into robot action sequences
keywords: [llm-planning, gpt4, llama, natural-language, task-planning]
---

# LLM-Based Action Planning

## What is LLM Planning?

**LLM planning** converts natural language commands into structured robot actions:

**Input**: *"Clean the room"*

**LLM Output**:
```json
[
  {"action": "navigate", "target": "room"},
  {"action": "detect_objects", "class": "trash"},
  {"action": "pick", "object_id": "trash_1"},
  {"action": "navigate", "target": "trash_bin"},
  {"action": "place", "location": "trash_bin"},
  {"action": "navigate", "target": "charging_station"}
]
```

---

## LLM Options

| Model | Type | Latency | Cost | Quality |
|-------|------|---------|------|---------|
| **GPT-4 Turbo** | Cloud API | 50-200ms | $0.01/1K tokens | Excellent |
| **Claude 3 Opus** | Cloud API | 50-150ms | $0.015/1K tokens | Excellent |
| **Llama 3.1 8B** | Local | &lt;10ms | Free (local) | Good |
| **Llama 3.1 70B** | Local | 50ms | Free (local) | Excellent |

**Recommendation**: Use **GPT-4 for planning** (latency acceptable for high-level decisions), **local models for real-time perception**.

---

## GPT-4 Integration

### Install OpenAI SDK

```bash
pip3 install openai
```

### Basic Planning Example

```python
import openai
import json

openai.api_key = "sk-..."  # Your API key

def plan_task(command):
    """Convert natural language → action sequence"""

    prompt = f"""
You are a robot task planner. Convert the user's command into a JSON action sequence.

Available actions:
- navigate: Move to a location (params: target)
- detect_objects: Find objects (params: class)
- pick: Pick up object (params: object_id)
- place: Place object (params: location)

User command: "{command}"

Return JSON array of actions:
"""

    response = openai.ChatCompletion.create(
        model="gpt-4-turbo",
        messages=[
            {"role": "system", "content": "You are a helpful robot task planner."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.0  # Deterministic output
    )

    # Parse JSON response
    action_sequence = json.loads(response.choices[0].message.content)

    return action_sequence

# Example usage
command = "Bring me water"
actions = plan_task(command)

print(actions)
# [
#   {"action": "navigate", "target": "kitchen"},
#   {"action": "detect_objects", "class": "water_bottle"},
#   {"action": "pick", "object_id": "water_bottle_1"},
#   {"action": "navigate", "target": "user"},
#   {"action": "place", "location": "table"}
# ]
```

---

## Prompt Engineering for Robotics

### System Prompt Template

```python
ROBOT_SYSTEM_PROMPT = """
You are an autonomous robot planner. Your capabilities:

**Navigation**:
- Known locations: kitchen, bedroom, living_room, charging_station, trash_bin
- Action: {"action": "navigate", "target": "location_name"}

**Perception**:
- Detect objects (classes: cup, bottle, trash, book)
- Action: {"action": "detect_objects", "class": "object_class"}

**Manipulation**:
- Pick object: {"action": "pick", "object_id": "id"}
- Place object: {"action": "place", "location": "location_name"}

**Rules**:
1. Always navigate before manipulating objects
2. Always detect objects before picking
3. Return to charging_station when task complete
4. Return JSON array only (no explanations)

Example:
User: "Throw away the trash"
Output:
[
  {"action": "navigate", "target": "living_room"},
  {"action": "detect_objects", "class": "trash"},
  {"action": "pick", "object_id": "trash_1"},
  {"action": "navigate", "target": "trash_bin"},
  {"action": "place", "location": "trash_bin"},
  {"action": "navigate", "target": "charging_station"}
]
"""
```

---

## Error Handling and Replanning

```python
def execute_plan_with_recovery(actions):
    """Execute actions with error recovery"""

    for i, action in enumerate(actions):
        success = execute_action(action)

        if not success:
            # Replan from current state
            print(f"Action {i} failed: {action}")
            print("Replanning...")

            remaining_goal = infer_remaining_goal(actions[i:])
            new_plan = plan_task(f"Continue task: {remaining_goal}")

            # Execute new plan
            return execute_plan_with_recovery(new_plan)

    return True  # All actions succeeded

def execute_action(action):
    """Execute single action via ROS 2"""
    if action["action"] == "navigate":
        return navigate_to(action["target"])
    elif action["action"] == "pick":
        return pick_object(action["object_id"])
    # ... handle other actions

    return False
```

---

## Local LLM Alternative (Llama 3.1)

For offline operation:

### Install Llama.cpp

```bash
git clone https://github.com/ggerganov/llama.cpp.git
cd llama.cpp && make

# Download Llama 3.1 8B model (GGUF format)
wget https://huggingface.co/meta-llama/Llama-3.1-8B-Instruct/resolve/main/llama-3.1-8b-instruct-q4_k_m.gguf
```

### Python Integration

```bash
pip3 install llama-cpp-python
```

```python
from llama_cpp import Llama

# Load model
llm = Llama(model_path="llama-3.1-8b-instruct-q4_k_m.gguf", n_ctx=4096)

def plan_task_local(command):
    prompt = f"{ROBOT_SYSTEM_PROMPT}\n\nUser command: {command}\n\nAction sequence:"

    response = llm(
        prompt,
        max_tokens=512,
        temperature=0.0,
        stop=["User:", "\n\n"]
    )

    # Parse JSON
    action_sequence = json.loads(response["choices"][0]["text"])

    return action_sequence
```

---

## Context-Aware Planning

Include environment state in prompt:

```python
def plan_with_context(command, current_location, detected_objects):
    context = f"""
Current robot state:
- Location: {current_location}
- Detected objects: {', '.join(detected_objects)}

User command: "{command}"
"""

    response = openai.ChatCompletion.create(
        model="gpt-4-turbo",
        messages=[
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": context}
        ]
    )

    return json.loads(response.choices[0].message.content)
```

---

## ROS 2 Planning Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.msg import ActionSequence, Action
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Subscriber for voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        # Publisher for action sequences
        self.plan_pub = self.create_publisher(ActionSequence, '/action_plan', 10)

        openai.api_key = self.declare_parameter('openai_api_key', '').value

        self.get_logger().info('LLM planner ready')

    def command_callback(self, msg):
        command = msg.data

        self.get_logger().info(f'Planning for: {command}')

        # Generate plan
        actions = self.plan_task(command)

        # Convert to ROS message
        plan_msg = ActionSequence()

        for action_dict in actions:
            action = Action()
            action.action_type = action_dict["action"]
            action.parameters = json.dumps(action_dict)
            plan_msg.actions.append(action)

        # Publish plan
        self.plan_pub.publish(plan_msg)

        self.get_logger().info(f'Published plan with {len(actions)} actions')

    def plan_task(self, command):
        response = openai.ChatCompletion.create(
            model="gpt-4-turbo",
            messages=[
                {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
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

## Hands-On Lab: Multi-Step Task Planning

**Goal**: Implement "Clean the room" task.

### Requirements

1. Detect trash objects in room
2. Pick each trash item
3. Navigate to trash bin
4. Place items
5. Return to charging station

### Expected Plan

```json
[
  {"action": "navigate", "target": "living_room"},
  {"action": "detect_objects", "class": "trash"},
  {"action": "pick", "object_id": "trash_1"},
  {"action": "navigate", "target": "trash_bin"},
  {"action": "place", "location": "trash_bin"},
  {"action": "pick", "object_id": "trash_2"},
  {"action": "navigate", "target": "trash_bin"},
  {"action": "place", "location": "trash_bin"},
  {"action": "navigate", "target": "charging_station"}
]
```

### Test Cases

- "Clean the room" → Detect and pick all trash
- "Bring me water" → Navigate kitchen, pick water, bring to user
- "Organize books" → Detect books, move to bookshelf

---

## Key Takeaways

✅ **LLMs convert** natural language → action sequences
✅ **Prompt engineering** critical for robot domains
✅ **GPT-4 for planning** (50-200ms acceptable)
✅ **Local Llama 3.1** for offline operation

---

## Next Steps

Learn ROS 2 integration in **[Chapter 3: ROS 2 Integration](/docs/vla/ros-integration)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/vla/ros-integration">
      Next: ROS 2 Integration →
    </a>
  </div>
</div>
