"""
Urdu Translator Agent using LiteLLM with Groq Mixtral
Translates technical content from English to natural technical Urdu
"""
import asyncio
import os

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


async def translate_chapter_to_urdu(
    original_content: str,
    chapter_id: str
) -> str:
    """
    Translate chapter content from English to Urdu (T017, T018)

    Args:
        original_content: The original English chapter content to translate
        chapter_id: Unique identifier for the chapter

    Returns:
        Translated Urdu chapter content as markdown string

    Raises:
        asyncio.TimeoutError: If translation takes longer than 60 seconds
        ValueError: If markdown structure validation fails or content exceeds context window
        Exception: If translation fails for any other reason
    """
    try:
        # Context window validation (T019)
        # Rough token estimation: ~4 chars per token for English
        estimated_tokens = len(original_content) // 4
        if estimated_tokens > 30000:
            raise ValueError(
                f"Chapter exceeds context window (estimated {estimated_tokens} tokens, limit 30k). "
                f"Try a shorter chapter. Mixtral-8x7b has 32k context window with 2k buffer for output."
            )

        # Translation instructions (T018)
        instructions = """You are an expert translator specializing in technical content from English to Urdu.

TRANSLATION RULES:
1. **Natural Technical Urdu**:
   - Use proper technical vocabulary (not awkward word-by-word translation)
   - Example: "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2" (not "روبوٹ آپریٹنگ نظام 2")
   - Transliterate widely-known English terms: "middleware" → "مڈل ویئر"

2. **Preserve Code Completely**:
   - All code blocks (```python, ```bash, etc.) MUST remain in English
   - Inline code (`code`) MUST remain in English
   - Only translate surrounding explanatory text

3. **Preserve Markdown Structure**:
   - Keep header levels (#, ##, ###)
   - Keep list formatting (-, 1., *)
   - Keep link format [text](URL) with URL unchanged
   - Keep code fence delimiters (```)

4. **Preserve Prices and Technical Units**:
   - "$249" stays "$249" (do NOT convert to PKR or "ڈالر")
   - "10 TOPS", "8GB RAM", "100Hz" stay in English
   - Context: "The Jetson costs $249" → "جیٹسن کی قیمت $249 ہے"

5. **Technical Accuracy**:
   - "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2"
   - "Jetson Orin Nano" → "جیٹسن اورن نینو"
   - "DDS middleware" → "ڈی ڈی ایس مڈل ویئر"
   - "Latency" → "تاخیر" (delay)
   - "Edge device" → "ایج ڈیوائس" (transliteration)

OUTPUT FORMAT:
- Return ONLY the translated Urdu markdown
- Do NOT add meta-commentary
- Preserve all whitespace and line breaks
"""

        # Initialize Groq model for Urdu translation (T017)
        # Using Mixtral-8x7b-32768 for superior multilingual (Urdu) support
        groq_model = LitellmModel(
            model="groq/mixtral-8x7b-32768",
            api_key=groq_api_key
        )

        # Create agent
        agent = Agent(
            name="UrduTranslator",
            instructions=instructions,
            model=groq_model
        )

        # Run agent with 60-second timeout
        result = await asyncio.wait_for(
            Runner.run(agent, f"Translate to Urdu:\\n\\n{original_content}"),
            timeout=60.0
        )

        translated_content = result.final_output if hasattr(result, 'final_output') else str(result)

        # Validate markdown structure preserved
        # Check for basic markdown elements
        if "#" in original_content and "#" not in translated_content:
            print(f"⚠️  Warning: Markdown headers may have been corrupted in chapter {chapter_id}")

        if "```" in original_content and "```" not in translated_content:
            print(f"⚠️  Warning: Code blocks may have been corrupted in chapter {chapter_id}")

        return translated_content

    except asyncio.TimeoutError:
        raise asyncio.TimeoutError(
            f"Translation took too long (>60s) for chapter '{chapter_id}'. "
            "Try a shorter chapter or try again later."
        )

    except ValueError as e:
        raise ValueError(f"Translation validation failed for chapter '{chapter_id}': {str(e)}")

    except Exception as e:
        error_message = str(e)

        # Check for rate limit errors (429)
        if "429" in error_message or "rate" in error_message.lower():
            raise Exception("⚠️ API rate limit exceeded. Please try again in a minute.")

        raise Exception(f"Translation failed for chapter '{chapter_id}': {str(e)}")


__all__ = ['translate_chapter_to_urdu']
