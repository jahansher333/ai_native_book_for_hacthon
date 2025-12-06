"""
Personalizer Agent Integration

This module integrates with the @personalizer subagent from the 8-agent system
to personalize educational content based on user profiles.
"""

import anthropic
import os
import logging

logger = logging.getLogger(__name__)

# Initialize Claude API client
client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

async def personalize_chapter_content(
    original_content: str,
    user_profile: dict,
    chapter_id: str
) -> str:
    """
    Call @personalizer subagent to rewrite chapter content based on user profile.

    Args:
        original_content: The original chapter text (MDX format)
        user_profile: Dict with experience, hasRTX, hasJetson, hasRobot
        chapter_id: Identifier for the chapter

    Returns:
        Personalized MDX content as string
    """

    # Build hardware description
    hardware_list = []
    if user_profile['hasRTX']:
        hardware_list.append('RTX GPU')
    if user_profile['hasJetson']:
        hardware_list.append('Jetson Orin Nano')
    if user_profile['hasRobot']:
        hardware_list.append('Real Robot')

    hardware_text = ', '.join(hardware_list) if hardware_list else 'No hardware (simulation only)'

    # Build comprehensive personalization prompt
    prompt = f"""You are an expert technical content personalizer for a Physical AI & Robotics textbook.

USER PROFILE:
- Experience Level: {user_profile['experience']}
- Hardware: {hardware_text}

ORIGINAL CHAPTER CONTENT:
{original_content}

TASK: Rewrite this chapter to perfectly match the user's profile.

EXPERIENCE LEVEL ADJUSTMENTS:

For BEGINNER users:
- Start with "Let me explain this like you're learning for the first time..."
- Add detailed explanations with real-world analogies
- Include heavily commented code with line-by-line explanations
- Break down complex concepts into simple steps
- Add encouraging notes: "💡 Don't worry if this seems complex at first..."
- Explain jargon and acronyms
- Add "What This Means" callouts for technical terms

For INTERMEDIATE users:
- Use standard technical language
- Provide balanced explanations (not too basic, not too advanced)
- Moderate code comments
- Include some best practices and tips
- Reference related advanced topics briefly

For ADVANCED users:
- Start with "Quick overview for advanced users..."
- Use concise, technical language
- Minimal code comments (assume expertise)
- Focus on optimizations, performance tuning, edge cases
- Include low-level implementation details
- Add sections on "Advanced Techniques" and "Performance Considerations"
- Reference research papers and cutting-edge methods

HARDWARE-SPECIFIC ADJUSTMENTS:

If user HAS RTX GPU:
- Mention: "Since you have an RTX GPU, you can run Isaac Sim locally"
- Add GPU-specific optimization tips
- Include CUDA-accelerated examples
- Suggest local training workflows
- Add callout: "🎮 GPU Tip: You can leverage your RTX for..."

If user HAS Jetson:
- Emphasize: "Perfect for deployment on your Jetson Orin Nano"
- Add Jetson-specific inference optimization examples
- Include edge deployment code with JetPack SDK
- Mention power efficiency considerations
- Add callout: "🤖 Jetson Deployment: Here's how to optimize for edge..."
- Include specific CUDA flags for Jetson architecture

If user HAS NO Jetson:
- Suggest cloud training options (AWS SageMaker, Google Colab, Azure ML)
- Mention remote robot control via cloud
- Emphasize simulation-first workflows
- Add callout: "☁️ Cloud Alternative: Since you don't have a Jetson, try..."

If user HAS Real Robot:
- Include real-world deployment considerations
- Add safety protocols and hardware precautions
- Mention sensor calibration steps
- Include physical setup instructions
- Add callout: "⚠️ Hardware Safety: When working with your real robot..."

FORMATTING REQUIREMENTS:
1. Maintain valid MDX syntax (headings, code blocks, lists)
2. Keep all code examples functional and correct
3. Preserve chapter structure (same sections, same order)
4. Add personalized callout boxes where relevant
5. Use appropriate tone for experience level
6. Include hardware-specific examples inline

EXAMPLE TRANSFORMATIONS:

Original (Generic):
"ROS 2 nodes communicate via topics. Here's a basic publisher:
```python
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('publisher')
pub = node.create_publisher(String, 'topic', 10)
```"

For BEGINNER with RTX:
"Let me explain ROS 2 nodes like this: imagine nodes as separate programs that talk to each other by sending messages (called topics). Think of it like different apps on your phone sharing data.

Since you have an RTX GPU, you'll be able to run these examples with full visualization in RViz2!

Here's your first publisher node (with detailed comments):
```python
import rclpy  # ROS 2 Python library
from std_msgs.msg import String  # Message type for text

# Step 1: Initialize ROS 2
rclpy.init()

# Step 2: Create a node (think of it as starting your program)
node = rclpy.create_node('publisher')

# Step 3: Create a publisher (this will send messages)
# - String: type of message
# - 'topic': name of the channel
# - 10: queue size (how many messages to buffer)
pub = node.create_publisher(String, 'topic', 10)
```

💡 **What This Means**: Your node can now broadcast messages to any other node listening on the 'topic' channel!"

For ADVANCED with Jetson:
"Quick overview: Standard ROS 2 pub/sub. Here's an optimized publisher for Jetson deployment:
```python
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node('publisher', enable_rosout=False)  # Disable logging for performance

# Jetson optimization: Use best-effort QoS for lower latency
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
pub = node.create_publisher(String, 'topic', qos)
```

🤖 **Jetson Deployment**: Use `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` and set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` for zero-copy transport. With your Jetson's limited memory, prefer DDS tuning over large queues."

Return ONLY the personalized MDX content. Do NOT include explanations or meta-commentary about the changes.
"""

    logger.info(f"Calling Claude API for personalization (experience={user_profile['experience']})")

    try:
        # Call Claude API with personalization prompt
        message = client.messages.create(
            model="claude-opus-4",
            max_tokens=8000,
            messages=[
                {"role": "user", "content": prompt}
            ]
        )

        personalized_content = message.content[0].text
        logger.info(f"Successfully personalized chapter {chapter_id}")

        return personalized_content

    except Exception as e:
        logger.error(f"Claude API error: {str(e)}")
        raise Exception(f"AI personalization failed: {str(e)}")
