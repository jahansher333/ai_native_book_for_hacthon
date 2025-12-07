"""
PersonalizeChapterSkill - Personalizes educational chapter content based on user profile

This skill uses Gemini API via OpenAI-compatible endpoint to rewrite educational
chapters tailored to individual users' experience levels and hardware availability.
"""
import re
import logging
from typing import Dict, Any
from openai import AsyncOpenAI

from config import settings

logger = logging.getLogger(__name__)


class PersonalizeChapterSkill:
    """
    Personalizes chapter content based on user profile.

    Adapts educational content to match user's:
    - Experience level (beginner/intermediate/advanced)
    - Hardware availability (RTX GPU, Jetson, Real Robot)
    - Learning preferences (implicit from profile)
    """

    name = "personalizeChapterSkill"
    description = "Rewrite educational chapter content for specific user"

    def __init__(self):
        """Initialize the skill with Gemini API client"""
        self.client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url=settings.base_url
        )

    async def execute(
        self,
        chapter_content: str,
        user_profile: Dict[str, Any],
        chapter_id: str
    ) -> str:
        """
        Execute personalization logic.

        Args:
            chapter_content: Original chapter content to personalize
            user_profile: User's profile with experience level and hardware
            chapter_id: Chapter identifier for logging

        Returns:
            Personalized chapter content maintaining all original information

        Raises:
            ValueError: If content validation fails
            Exception: If API call fails
        """
        logger.info(f"Executing PersonalizeChapterSkill for chapter {chapter_id}")

        # Build personalization prompt
        prompt = self._build_prompt(chapter_content, user_profile)

        try:
            # Call Gemini via OpenAI-compatible endpoint
            response = await self.client.chat.completions.create(
                model=settings.model_name,  # gemini-2.0-flash-exp
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=8192
            )

            personalized_content = response.choices[0].message.content

            # Validate output preserves key information
            self._validate_personalization(chapter_content, personalized_content)

            logger.info(f"Successfully personalized chapter {chapter_id}")
            return personalized_content

        except Exception as e:
            logger.error(f"Personalization failed for chapter {chapter_id}: {e}", exc_info=True)
            raise

    def _build_prompt(self, content: str, profile: Dict[str, Any]) -> str:
        """Build personalization prompt based on user profile"""
        experience = profile.get('experience', 'intermediate')
        hardware = []

        if profile.get('hasRTX'):
            hardware.append("RTX GPU")
        if profile.get('hasJetson'):
            hardware.append("Jetson")
        if profile.get('hasRobot'):
            hardware.append("Real Robot")

        hardware_str = ", ".join(hardware) if hardware else "no specialized hardware"

        # Experience-specific guidelines
        experience_guidelines = {
            'beginner': """
   - Add more explanations and analogies
   - Avoid technical jargon or explain it when necessary
   - Provide step-by-step instructions
   - Include more examples and use cases
   - Add encouragement and context""",
            'intermediate': """
   - Balance theory and practice
   - Reference prerequisites when needed
   - Assume basic understanding of concepts
   - Focus on practical applications
   - Mention common pitfalls""",
            'advanced': """
   - Focus on edge cases and optimizations
   - Reference research papers and advanced topics
   - Assume strong technical foundation
   - Discuss trade-offs and design decisions
   - Include performance considerations"""
        }

        # Hardware-specific guidelines
        hardware_guidelines = []
        if profile.get('hasRTX'):
            hardware_guidelines.append("- Mention cloud training alternatives and compare with local GPU training")
        if profile.get('hasJetson'):
            hardware_guidelines.append("- Emphasize edge deployment strategies and power efficiency")
        if profile.get('hasRobot'):
            hardware_guidelines.append("- Include safety warnings and hardware-specific troubleshooting tips")
        if not hardware:
            hardware_guidelines.append("- Focus on simulation environments and cloud-based resources")
            hardware_guidelines.append("- Explain how to learn without physical hardware")

        hardware_section = "\n".join(hardware_guidelines) if hardware_guidelines else "- Focus on theoretical understanding"

        return f"""Personalize the following educational chapter for a **{experience}** student with **{hardware_str}**.

**Personalization Guidelines**:

1. **Experience Level** ({experience}):
{experience_guidelines.get(experience, experience_guidelines['intermediate'])}

2. **Hardware Availability**:
{hardware_section}

3. **Critical Constraints** (MUST FOLLOW):
   - Preserve ALL original information (no content loss)
   - Maintain exact markdown formatting (headings, lists, code blocks, tables)
   - Keep all code examples accurate and functional
   - Maintain all technical correctness
   - Do NOT add implementation details not in original
   - Do NOT remove any important concepts

4. **Personalization Approach**:
   - Adapt the **tone and explanations** to match user level
   - Add **context relevant** to their hardware situation
   - Keep the **same structure and information density**
   - Make content **more engaging and relevant** for this specific user

**Original Chapter**:
```markdown
{content}
```

**Your Task**:
Rewrite the chapter above with personalized tone, explanations, and hardware-specific guidance while maintaining 100% of the original information and structure. Output ONLY the personalized markdown content, no preamble or explanation."""

    def _get_system_prompt(self) -> str:
        """Get system prompt for the AI model"""
        return """You are an expert technical writer and educator specializing in robotics and AI.

Your task is to personalize educational content for individual learners while maintaining complete accuracy and information integrity.

Key principles:
- Adapt tone and explanations to learner level (beginner/intermediate/advanced)
- Add relevant context based on available hardware
- Preserve all technical accuracy and completeness
- Maintain exact markdown structure
- Never lose or change factual information
- Make learning more engaging and accessible for the specific user

You are personalizing Physical AI and Humanoid Robotics textbook chapters covering ROS 2, NVIDIA Isaac Sim, Jetson edge deployment, and vision-language-action models."""

    def _validate_personalization(self, original: str, personalized: str):
        """
        Validate personalized content meets quality standards

        Args:
            original: Original chapter content
            personalized: Personalized chapter content

        Raises:
            ValueError: If validation fails
        """
        # Check length isn't drastically different (±50%)
        orig_len = len(original)
        pers_len = len(personalized)

        if pers_len < orig_len * 0.5:
            raise ValueError(
                f"Personalized content too short: {pers_len} chars vs original {orig_len} chars "
                f"(less than 50% of original)"
            )

        if pers_len > orig_len * 2.0:
            raise ValueError(
                f"Personalized content too long: {pers_len} chars vs original {orig_len} chars "
                f"(more than 200% of original)"
            )

        # Check markdown structure preserved (heading count)
        orig_headers = len(re.findall(r'^#{1,6}\s', original, re.MULTILINE))
        pers_headers = len(re.findall(r'^#{1,6}\s', personalized, re.MULTILINE))

        if abs(orig_headers - pers_headers) > 2:
            logger.warning(
                f"Markdown structure changed significantly: {pers_headers} headers vs "
                f"original {orig_headers} headers"
            )
            # Don't raise error, just warn - structure changes can be acceptable

        # Basic content check - ensure it's not empty
        if len(personalized.strip()) < 100:
            raise ValueError("Personalized content is too short or empty")

        logger.debug(
            f"Validation passed: length={pers_len} chars ({pers_len/orig_len*100:.1f}% of original), "
            f"headers={pers_headers} (original: {orig_headers})"
        )


# Export skill instance for use by agent
personalize_chapter_skill = PersonalizeChapterSkill()
