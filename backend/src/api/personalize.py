from fastapi import APIRouter, Depends, HTTPException, Header
from pydantic import BaseModel
from typing import Literal, Optional
import logging

# Import auth service
from ..services.auth_service import AuthService

router = APIRouter()
logger = logging.getLogger(__name__)

# Auth dependency
async def get_current_user(authorization: Optional[str] = Header(None)) -> str:
    """Extract and verify JWT token from Authorization header"""
    if not authorization:
        raise HTTPException(status_code=401, detail="Not authenticated")

    try:
        # Extract token from "Bearer <token>"
        token = authorization.replace("Bearer ", "")
        payload = AuthService.verify_token(token)

        if not payload:
            raise HTTPException(status_code=401, detail="Invalid token")

        return payload.get("email")
    except Exception as e:
        logger.error(f"Auth error: {str(e)}")
        raise HTTPException(status_code=401, detail="Invalid authentication")

class UserProfile(BaseModel):
    experience: Literal['beginner', 'intermediate', 'advanced']
    hasRTX: bool
    hasJetson: bool
    hasRobot: bool

class PersonalizeChapterRequest(BaseModel):
    chapterId: str
    originalContent: str
    userProfile: UserProfile

class PersonalizeChapterResponse(BaseModel):
    success: bool
    personalizedContent: str
    metadata: dict

@router.post("/api/personalize/chapter", response_model=PersonalizeChapterResponse)
async def personalize_chapter(
    request: PersonalizeChapterRequest,
    user_email: str = Depends(get_current_user)
):
    """
    Personalize chapter content based on user profile.

    This endpoint calls the @personalizer subagent to rewrite educational content
    based on the user's experience level and hardware setup.
    """
    try:
        logger.info(f"Personalization request from {user_email} for chapter {request.chapterId}")

        # Import personalizer agent function (LiteLLM/Groq)
        from ..ai_agents.litellm_converted.personalize_agent import personalize_chapter_content

        # Call @personalizer subagent
        personalized_content = await personalize_chapter_content(
            original_content=request.originalContent,
            user_profile=request.userProfile.dict(),
            chapter_id=request.chapterId
        )

        logger.info(f"Personalization successful for {user_email}, chapter {request.chapterId}")

        return PersonalizeChapterResponse(
            success=True,
            personalizedContent=personalized_content,
            metadata={
                "generationTime": 0,  # TODO: Track actual time
                "cached": False
            }
        )

    except Exception as e:
        logger.error(f"Personalization failed for {user_email}: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")
