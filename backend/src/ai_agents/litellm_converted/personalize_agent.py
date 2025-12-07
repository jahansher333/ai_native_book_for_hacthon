"""
Personalize Agent using LiteLLM with Groq backend
Adapts chapter content based on user experience level and hardware
"""
import asyncio
import os
import re
from typing import Dict, Any

# Import openai-agents BEFORE local imports to avoid name collision
import sys
_original_path = sys.path[:]
sys.path = [p for p in sys.path if 'backend' not in p and 'src' not in p.lower()]
try:
    from agents import Agent, Runner  # openai-agents package
    from agents.extensions.models.litellm_model import LitellmModel  # openai-agents extension
finally:
    sys.path = _original_path

from dotenv import load_dotenv

load_dotenv(override=True)
groq_api_key = os.getenv("GROQ_API_KEY")


async def personalize_chapter_content(
    original_content: str,
    user_profile: Dict[str, Any],
    chapter_id: str
) -> str:
    """
    Personalize chapter content for a specific user (T013, T014)

    Args:
        original_content: The original chapter content to personalize
        user_profile: User's profile containing experience level and hardware info
            Example: {"experience": "advanced", "hasJetson": True}
        chapter_id: Unique identifier for the chapter

    Returns:
        Personalized chapter content as markdown string

    Raises:
        asyncio.TimeoutError: If personalization takes longer than 60 seconds
        ValueError: If content validation fails
        Exception: If personalization fails for any other reason
    """
    try:
        # Extract profile information
        experience = user_profile.get("experience", "intermediate")
        has_jetson = user_profile.get("hasJetson", False)

        # Build dynamic instructions based on profile (T014)
        instructions = f"""You are an expert technical writer personalizing educational content.

    USER PROFILE:
    - Experience Level: {experience}
    - Hardware: {"Owns Jetson Orin Nano ($249)" if has_jetson else "No Jetson hardware"}

    PERSONALIZATION RULES:
    1. **Adapt Depth**:
    - Beginner: Explain concepts from first principles, use analogies
    - Intermediate: Assume basic knowledge, focus on practical steps
    - Advanced: Skip basics, show optimizations and edge cases

    2. **Hardware-Specific Examples**:
    - With Jetson: Add Jetson Orin Nano deployment code, TensorRT conversion steps
    - Without Jetson: Emphasize simulation (Gazebo, Isaac Sim), mention Jetson as $249 upgrade

    3. **Preserve Accuracy**:
    - Do NOT change hardware prices (keep "$249" exactly as written)
    - Do NOT add false claims or unsupported examples
    - Cite textbook content when referencing specific facts

    4. **Maintain Structure**:
    - Keep markdown headers (#, ##, ###)
    - Preserve code blocks with proper language tags
    - Maintain ordered/unordered lists

    5. **Sim-to-Real Workflow**:
    - Always teach: Train (cloud/workstation) → Export (ONNX/TensorRT) → Deploy (Jetson)
    - Warn against cloud-controlled robots (50-200ms latency)

    OUTPUT FORMAT:
    - Return ONLY the personalized markdown content
    - Do NOT add meta-commentary like "Here's the personalized version..."
    - Preserve code blocks exactly (no modifications to code)
    """

        # Initialize Groq model (T013)
        groq_model = LitellmModel(
           model="groq/llama-3.3-70b-versatile",
            api_key=groq_api_key
        )

        # Create agent
        agent = Agent(
            name="ChapterPersonalizer",
            instructions=instructions,
            model=groq_model
        )

        # Run agent with 60-second timeout
        result = await asyncio.wait_for(
            Runner.run(agent, f"Personalize this chapter:\\n\\n{original_content}"),
            timeout=60.0
        )

        personalized_content = result.final_output if hasattr(result, 'final_output') else str(result)

        # Validate price preservation (T015)
        # Check if original content had "$249" and verify it's preserved
        if "$249" in original_content:
            if "$249" not in personalized_content:
                # Log warning but don't block
                print(f"⚠️  Warning: Price $249 may have been modified in chapter {chapter_id}")
                # Attempt to restore price mentions
                personalized_content = re.sub(r'\\$?249\\s*(?:USD|dollars)?', '$249', personalized_content)

        return personalized_content

    except asyncio.TimeoutError:
        raise asyncio.TimeoutError(
            f"Personalization took too long (>60s) for chapter '{chapter_id}'. "
            "Try a shorter chapter or try again later."
        )

    except ValueError as e:
        raise ValueError(f"Personalization validation failed for chapter '{chapter_id}': {str(e)}")

    except Exception as e:
        error_message = str(e)

        # Check for rate limit errors (429)
        if "429" in error_message or "rate" in error_message.lower():
            raise Exception("⚠️ API rate limit exceeded. Please try again in a minute.")

        raise Exception(f"Personalization failed for chapter '{chapter_id}': {str(e)}")


__all__ = ['personalize_chapter_content']
