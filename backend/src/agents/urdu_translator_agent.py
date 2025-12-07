"""
Urdu Translator Agent - Orchestrates chapter translation using @urdu-translator subagent

This module provides the main entry point for chapter translation to Urdu, coordinating
the TranslateToUrduSkill with Gemini API to deliver natural, technical Urdu content.
"""
import asyncio
import logging
import time

from .skills.translate_to_urdu_skill import translate_to_urdu_skill

logger = logging.getLogger(__name__)


async def translate_chapter_to_urdu(
    original_content: str,
    chapter_id: str
) -> str:
    """
    Translate chapter content from English to Urdu.

    This function orchestrates the translation process by calling the
    TranslateToUrduSkill with the appropriate context and handling timeouts.

    Args:
        original_content: The original English chapter content to translate
        chapter_id: Unique identifier for the chapter

    Returns:
        Translated Urdu chapter content as markdown string

    Raises:
        asyncio.TimeoutError: If translation takes longer than 60 seconds
        ValueError: If markdown structure validation fails
        Exception: If translation fails for any other reason
    """
    start_time = time.time()

    logger.info(
        f"Starting Urdu translation for chapter '{chapter_id}' - "
        f"Content length: {len(original_content)} chars"
    )

    try:
        # Execute translation with 60-second timeout
        translated_content = await asyncio.wait_for(
            translate_to_urdu_skill.execute(
                chapter_content=original_content,
                chapter_id=chapter_id
            ),
            timeout=60.0
        )

        duration = time.time() - start_time
        logger.info(
            f"Translation to Urdu completed for chapter '{chapter_id}' in {duration:.2f}s"
        )

        return translated_content

    except asyncio.TimeoutError:
        duration = time.time() - start_time
        logger.error(
            f"Translation timeout for chapter '{chapter_id}' after {duration:.2f}s"
        )
        raise asyncio.TimeoutError(
            f"Translation took too long (>{duration:.0f}s). "
            "Try a shorter chapter or try again later."
        )

    except ValueError as e:
        duration = time.time() - start_time
        logger.error(
            f"Translation validation failed for chapter '{chapter_id}' "
            f"after {duration:.2f}s: {e}"
        )
        raise

    except Exception as e:
        duration = time.time() - start_time
        logger.error(
            f"Translation failed for chapter '{chapter_id}' "
            f"after {duration:.2f}s: {e}",
            exc_info=True
        )
        raise Exception(f"Translation failed: {str(e)}")


# For backward compatibility with Agent/Runner pattern (if needed in future)
class UrduTranslatorAgent:
    """
    Agent wrapper for Urdu translation.

    This class provides a structured interface for the translation agent,
    making it easier to extend with additional skills or features in the future.
    """

    def __init__(self):
        """Initialize the Urdu translator agent with its skill"""
        self.name = "Urdu Translator"
        self.description = "Translates technical robotics content from English to Urdu"
        self.skill = translate_to_urdu_skill

    async def translate(
        self,
        chapter_content: str,
        chapter_id: str
    ) -> str:
        """
        Translate chapter using the agent's skill.

        Args:
            chapter_content: Original English chapter content
            chapter_id: Chapter identifier

        Returns:
            Translated Urdu chapter content
        """
        return await translate_chapter_to_urdu(
            chapter_content,
            chapter_id
        )


# Export main function and agent class
__all__ = ['translate_chapter_to_urdu', 'UrduTranslatorAgent']
