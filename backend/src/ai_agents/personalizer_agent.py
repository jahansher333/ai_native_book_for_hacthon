"""
Personalizer Agent - Orchestrates chapter personalization using OpenAI Agents SDK

This module provides the main entry point for chapter personalization, coordinating
the PersonalizeChapterSkill with Gemini API to deliver personalized content.
"""
import asyncio
import logging
import time
from typing import Dict, Any

from .skills.personalize_chapter_skill import personalize_chapter_skill

logger = logging.getLogger(__name__)


async def personalize_chapter_content(
    original_content: str,
    user_profile: Dict[str, Any],
    chapter_id: str
) -> str:
    """
    Personalize chapter content for a specific user.

    This function orchestrates the personalization process by calling the
    PersonalizeChapterSkill with the appropriate context and handling timeouts.

    Args:
        original_content: The original chapter content to personalize
        user_profile: User's profile containing experience level and hardware info
        chapter_id: Unique identifier for the chapter

    Returns:
        Personalized chapter content as markdown string

    Raises:
        asyncio.TimeoutError: If personalization takes longer than 60 seconds
        ValueError: If content validation fails
        Exception: If personalization fails for any other reason
    """
    start_time = time.time()

    logger.info(
        f"Starting personalization for chapter '{chapter_id}' - "
        f"User: {user_profile.get('experience', 'unknown')} level"
    )

    try:
        # Execute personalization with 60-second timeout
        personalized_content = await asyncio.wait_for(
            personalize_chapter_skill.execute(
                chapter_content=original_content,
                user_profile=user_profile,
                chapter_id=chapter_id
            ),
            timeout=60.0
        )

        duration = time.time() - start_time
        logger.info(
            f"Personalization completed for chapter '{chapter_id}' in {duration:.2f}s"
        )

        return personalized_content

    except asyncio.TimeoutError:
        duration = time.time() - start_time
        logger.error(
            f"Personalization timeout for chapter '{chapter_id}' after {duration:.2f}s"
        )
        raise asyncio.TimeoutError(
            f"Personalization took too long (>{duration:.0f}s). "
            "Try a shorter chapter or try again later."
        )

    except ValueError as e:
        duration = time.time() - start_time
        logger.error(
            f"Personalization validation failed for chapter '{chapter_id}' "
            f"after {duration:.2f}s: {e}"
        )
        raise

    except Exception as e:
        duration = time.time() - start_time
        logger.error(
            f"Personalization failed for chapter '{chapter_id}' "
            f"after {duration:.2f}s: {e}",
            exc_info=True
        )
        raise Exception(f"Personalization failed: {str(e)}")


# For backward compatibility with Agent/Runner pattern (if needed in future)
# Currently using direct skill execution for simplicity
class PersonalizerAgent:
    """
    Agent wrapper for chapter personalization.

    This class provides a structured interface for the personalization agent,
    making it easier to extend with additional skills or features in the future.
    """

    def __init__(self):
        """Initialize the personalizer agent with its skill"""
        self.name = "Chapter Personalizer"
        self.description = "Personalizes educational content based on user profile"
        self.skill = personalize_chapter_skill

    async def personalize(
        self,
        chapter_content: str,
        user_profile: Dict[str, Any],
        chapter_id: str
    ) -> str:
        """
        Personalize chapter using the agent's skill.

        Args:
            chapter_content: Original chapter content
            user_profile: User's profile information
            chapter_id: Chapter identifier

        Returns:
            Personalized chapter content
        """
        return await personalize_chapter_content(
            chapter_content,
            user_profile,
            chapter_id
        )


# Export main function and agent class
__all__ = ['personalize_chapter_content', 'PersonalizerAgent']
