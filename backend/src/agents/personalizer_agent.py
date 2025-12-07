"""
Personalizer Agent Integration

This module integrates with the @personalizer subagent from the 8-agent system
to personalize educational content based on user profiles.

Uses OpenAI Agents SDK with Gemini 2.0 Flash for fast, cost-effective personalization.
"""

from agents import Agent, Runner
import os
import logging

logger = logging.getLogger(__name__)

# Define the personalizer agent
personalizer_agent = Agent(
    name="personalizer",
    instructions="""You are an expert technical content personalizer for a Physical AI & Robotics textbook.

TASK: Rewrite educational content to perfectly match the user's profile.

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

Return ONLY the personalized MDX content. Do NOT include explanations or meta-commentary about the changes.
"""
)

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
    if user_profile.get('hasRTX'):
        hardware_list.append('RTX GPU')
    if user_profile.get('hasJetson'):
        hardware_list.append('Jetson Orin Nano')
    if user_profile.get('hasRobot'):
        hardware_list.append('Real Robot')

    hardware_text = ', '.join(hardware_list) if hardware_list else 'No hardware (simulation only)'

    # Build user query for the agent
    user_message = f"""USER PROFILE:
- Experience Level: {user_profile.get('experience', 'intermediate')}
- Hardware: {hardware_text}

ORIGINAL CHAPTER CONTENT:
{original_content}

Please rewrite this chapter to match the user's profile according to your instructions."""

    logger.info(f"Calling Gemini 2.0 Flash via Agents SDK for personalization (experience={user_profile.get('experience')})")

    try:
        # Create a runner with Gemini configuration
        runner = Runner(
            agent=personalizer_agent,
            api_key=os.getenv("GEMINI_API_KEY"),
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            model="gemini-2.0-flash-exp"
        )

        # Run the agent with the user message
        result = await runner.run(user_message)

        personalized_content = result.messages[-1].content
        logger.info(f"Successfully personalized chapter {chapter_id}")

        return personalized_content

    except Exception as e:
        logger.error(f"Gemini API error via Agents SDK: {str(e)}")
        raise Exception(f"AI personalization failed: {str(e)}")
