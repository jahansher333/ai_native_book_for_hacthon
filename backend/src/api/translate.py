from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
import logging
import time

router = APIRouter()
logger = logging.getLogger(__name__)


class TranslationRequest(BaseModel):
    chapterId: str = Field(..., pattern=r'^[a-z0-9-]+$', min_length=3, max_length=100)
    originalContent: str = Field(..., min_length=100, max_length=50000)


class TranslationResponse(BaseModel):
    chapterId: str
    translatedContent: str
    timestamp: int
    cached: bool = False


@router.post("/api/translate/chapter", response_model=TranslationResponse)
async def translate_chapter(request: TranslationRequest):
    """
    Translate chapter content from English to Urdu.

    This endpoint calls the @urdu-translator subagent to translate educational content
    into natural, readable, technical Urdu for Pakistani students.

    **No authentication required** - Urdu translation is accessible to all users.
    """
    start_time = time.time()

    try:
        logger.info(f"Translation request for chapter {request.chapterId}")

        # Import translator agent function
        from ..agents.urdu_translator_agent import translate_chapter_to_urdu

        # Call @urdu-translator subagent
        translated_content = await translate_chapter_to_urdu(
            original_content=request.originalContent,
            chapter_id=request.chapterId
        )

        duration = time.time() - start_time
        logger.info(
            f"Translation successful for chapter {request.chapterId} in {duration:.2f}s"
        )

        return TranslationResponse(
            chapterId=request.chapterId,
            translatedContent=translated_content,
            timestamp=int(time.time() * 1000),
            cached=False
        )

    except Exception as e:
        duration = time.time() - start_time
        logger.error(
            f"Translation failed for chapter {request.chapterId} "
            f"after {duration:.2f}s: {str(e)}",
            exc_info=True
        )

        # Return appropriate HTTP status code based on error type
        if "timeout" in str(e).lower():
            raise HTTPException(
                status_code=504,
                detail="Translation took too long. Please try a shorter chapter."
            )
        elif "quota" in str(e).lower() or "429" in str(e):
            raise HTTPException(
                status_code=429,
                detail="Translation service temporarily unavailable. Try again in 24 hours."
            )
        elif "validation" in str(e).lower():
            raise HTTPException(
                status_code=500,
                detail=f"Translation validation failed: {str(e)}"
            )
        else:
            raise HTTPException(
                status_code=500,
                detail="Translation failed. Please try again."
            )


@router.get("/api/translate/health")
async def health_check():
    """Health check endpoint for translation service"""
    from config import settings

    return {
        "status": "healthy",
        "gemini_api_available": bool(settings.gemini_api_key),
        "timestamp": int(time.time() * 1000)
    }
