"""
TranslateToUrduSkill - Translates technical robotics content to natural Urdu

This skill uses Gemini API via OpenAI-compatible endpoint to translate English
educational chapters into natural, readable, technical Urdu for Pakistani students.
"""
import re
import logging
from openai import AsyncOpenAI

from config import settings

logger = logging.getLogger(__name__)


class TranslateToUrduSkill:
    """
    Translates chapter content from English to technical Urdu.

    Uses 3-tier translation strategy:
    - Tier 1 (Full Translation): Established terms → روبوٹک آپریٹنگ سسٹم 2
    - Tier 2 (Transliteration): Proper nouns → جیٹسن
    - Tier 3 (Hybrid): Jargon → لیٹنسی ٹریپ (خطرناک تاخیر)
    """

    name = "translateToUrduSkill"
    description = "Translate English technical content to natural Urdu"

    def __init__(self):
        """Initialize the skill with Gemini API client"""
        self.client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url=settings.base_url
        )

    async def execute(
        self,
        chapter_content: str,
        chapter_id: str
    ) -> str:
        """
        Execute translation logic.

        Args:
            chapter_content: Original English chapter content
            chapter_id: Chapter identifier for logging

        Returns:
            Urdu translated content maintaining all markdown structure

        Raises:
            ValueError: If markdown structure validation fails
            Exception: If API call fails
        """
        logger.info(f"Executing TranslateToUrduSkill for chapter {chapter_id}")

        # Build translation prompt with glossary
        prompt = self._build_translation_prompt(chapter_content)

        try:
            # Call Gemini via OpenAI-compatible endpoint
            response = await self.client.chat.completions.create(
                model=settings.model_name,  # gemini-2.0-flash
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=8192
            )

            translated_content = response.choices[0].message.content

            # Validate markdown structure preservation
            self._validate_markdown_structure(chapter_content, translated_content)

            logger.info(f"Successfully translated chapter {chapter_id} to Urdu")
            return translated_content

        except Exception as e:
            logger.error(f"Translation failed for chapter {chapter_id}: {e}", exc_info=True)
            raise

    def _get_system_prompt(self) -> str:
        """Get system prompt for translation"""
        return """You are an expert technical translator specializing in robotics and AI content for Pakistani students.

Your task is to translate English technical documentation into natural, readable Urdu that maintains technical accuracy.

**Critical Rules**:
1. Preserve ALL markdown structure (headings, lists, code blocks, links)
2. Keep ALL code blocks completely in English - do not translate code
3. Keep ALL URLs and links unchanged
4. Use natural Urdu sentence flow
5. Follow the technical term glossary provided
6. Maintain the same level of technical depth

**Translation Quality**:
- Use formal Urdu suitable for educational content
- Maintain technical precision while being readable
- Keep acronyms and their Urdu expansions
- Use contextual explanations for complex terms"""

    def _build_translation_prompt(self, content: str) -> str:
        """Build translation prompt with 3-tier glossary"""
        glossary = self._get_technical_glossary()

        return f"""Translate the following English robotics chapter to Urdu for Pakistani students.

**Technical Term Translation Rules** (MUST FOLLOW):
{glossary}

**Important Instructions**:
- Preserve exact markdown structure (headings, lists, code blocks, links)
- Keep code blocks in English
- Keep URLs unchanged
- Use natural Urdu prose
- Apply glossary rules consistently
- Add contextual explanations for difficult concepts

**Content to translate**:

{content}"""

    def _get_technical_glossary(self) -> str:
        """3-tier technical term glossary"""
        return """
**Tier 1 - Full Translation** (Established robotics/tech concepts):
- ROS 2 → روبوٹک آپریٹنگ سسٹم 2 (Robotic Operating System 2)
- URDF → یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ (Unified Robot Description Format)
- Publisher → پبلشر (ناشر)
- Subscriber → سبسکرائبر (وصول کنندہ)
- Node → نوڈ (اکائی)
- Service → سروس (خدمت)
- Action → ایکشن (عمل)
- Topic → ٹاپک (موضوع)
- Package → پیکیج (بنڈل)
- Message → پیغام
- Sensor → سینسر (حساس آلہ)
- Actuator → ایکچویٹر (محرک)
- Robot → روبوٹ (خودکار مشین)
- Simulation → سمولیشن (نقلی ماحول)
- Camera → کیمرہ
- Lidar → لائیڈار (لیزر فاصلہ پیمائش)

**Tier 2 - Transliteration** (Proper nouns, brand names, products):
- Jetson → جیٹسن
- Jetson Orin Nano → جیٹسن اورین نینو
- NVIDIA → اینویڈیا
- Isaac Sim → آئزک سم
- Unitree → یونٹری
- RealSense → ریئل سینس
- RealSense D435i → ریئل سینس ڈی 435 آئی
- Gazebo → گازیبو
- Ubuntu → اُبنٹو
- Docker → ڈاکر
- Python → پائیتھن
- C++ → سی پلس پلس
- CUDA → کوڈا
- TensorRT → ٹینسر آر ٹی
- PyTorch → پائی ٹارچ
- AWS → اے ڈبلیو ایس

**Tier 3 - Hybrid** (Technical jargon needing context):
- Latency trap → لیٹنسی ٹریپ (خطرناک تاخیر)
- Sim-to-real → سم ٹو ریئل (نقلی سے حقیقی منتقلی)
- Edge deployment → ایج ڈیپلائمنٹ (مقامی آلات پر تعیناتی)
- Real-time → ریئل ٹائم (حقیقی وقت)
- Inference → انفرنس (نتیجہ اخذ کرنا)
- Training → ٹریننگ (تربیت)
- Dataset → ڈیٹاسیٹ (ڈیٹا کا مجموعہ)
- Pipeline → پائپ لائن (عمل کا سلسلہ)
- Workflow → ورک فلو (کام کا طریقہ)
- Framework → فریم ورک (بنیادی ڈھانچہ)
- API → اے پی آئی (ایپلیکیشن پروگرامنگ انٹرفیس)
- GPU → جی پی یو (گرافکس پروسیسنگ یونٹ)
- CPU → سی پی یو (سنٹرل پروسیسنگ یونٹ)
- TOPS → ٹی او پی ایس (ٹرلین آپریشنز فی سیکنڈ)

**Currency and Numbers**:
- Keep $ symbols: $700 → $700
- Use Urdu digits for text: "costs 700 dollars" → "قیمت 700 ڈالر ہے"
- Example: "The Economy Jetson Kit costs ~$700" → "ایکانومی جیٹسن کٹ کی قیمت تقریباً $700 ہے"
"""

    def _validate_markdown_structure(self, original: str, translated: str) -> None:
        """
        Validate that translated content preserves markdown structure.

        Args:
            original: Original English content
            translated: Translated Urdu content

        Raises:
            ValueError: If structure validation fails
        """
        # Check heading count
        original_headings = len(re.findall(r'^#{1,6}\s', original, re.MULTILINE))
        translated_headings = len(re.findall(r'^#{1,6}\s', translated, re.MULTILINE))

        if original_headings != translated_headings:
            raise ValueError(
                f"Heading count mismatch: original has {original_headings}, "
                f"translated has {translated_headings}"
            )

        # Check code block count
        original_code_blocks = original.count('```')
        translated_code_blocks = translated.count('```')

        if original_code_blocks != translated_code_blocks:
            raise ValueError(
                f"Code block mismatch: original has {original_code_blocks}, "
                f"translated has {translated_code_blocks}"
            )

        # Check link count (±10% tolerance)
        original_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', original))
        translated_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', translated))

        if abs(original_links - translated_links) > max(1, original_links * 0.1):
            raise ValueError(
                f"Link count mismatch: original has {original_links}, "
                f"translated has {translated_links}"
            )

        logger.info("Markdown structure validation passed")


# Create singleton instance
translate_to_urdu_skill = TranslateToUrduSkill()
