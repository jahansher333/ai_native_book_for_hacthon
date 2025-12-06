"""
Authentication API endpoints
Handles signup, signin, and session validation with profile questions
"""
from fastapi import APIRouter, HTTPException, Header
from pydantic import BaseModel, EmailStr, Field
from typing import Optional
from datetime import datetime

from ..services.auth_service import auth_service

router = APIRouter(prefix="/api/auth", tags=["authentication"])

# Request/Response Models
class ProfileData(BaseModel):
    hasRTX: bool = Field(..., description="User owns NVIDIA RTX GPU")
    hasJetson: bool = Field(..., description="User owns Jetson Orin Nano")
    hasRobot: bool = Field(..., description="User has access to real robot")
    experience: str = Field(..., description="beginner | intermediate | advanced")

class SignupRequest(BaseModel):
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="Password (min 8 characters)")
    profile: ProfileData = Field(..., description="User profile (4 questions)")

class SigninRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    success: bool
    user: dict
    token: str
    expires_at: str

class SessionResponse(BaseModel):
    authenticated: bool
    user: Optional[dict] = None

# Endpoints
@router.post("/signup", response_model=AuthResponse, status_code=201)
async def signup(request: SignupRequest):
    """
    Create new user account with profile
    Stores 4-question profile (hardware + experience) in Neon Postgres

    Profile questions:
    1. Do you have an RTX GPU? (hasRTX)
    2. Do you own a Jetson Orin Nano? (hasJetson)
    3. Do you have access to a real robot? (hasRobot)
    4. Your programming experience? (experience: beginner/intermediate/advanced)
    """
    try:
        print(f"Signup request received: email={request.email}, profile={request.profile.dict()}")

        user, token, expires_at = auth_service.create_user_with_profile(
            email=request.email,
            password=request.password,
            profile=request.profile.dict()
        )

        return {
            "success": True,
            "user": user.to_dict(),
            "token": token,
            "expires_at": expires_at.isoformat()
        }

    except ValueError as e:
        # Validation errors (duplicate email, invalid profile)
        print(f"Signup ValueError: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Database errors or unexpected issues
        print(f"Signup unexpected error: {type(e).__name__}: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """
    Authenticate user with email + password
    Returns user profile + JWT token on success
    """
    result = auth_service.authenticate_user(request.email, request.password)

    if not result:
        # Generic error (prevents email enumeration attacks)
        raise HTTPException(
            status_code=401,
            detail="Invalid email or password"
        )

    user, token, expires_at = result

    return {
        "success": True,
        "user": user.to_dict(),
        "token": token,
        "expires_at": expires_at.isoformat()
    }


@router.get("/session", response_model=SessionResponse)
async def get_session(authorization: Optional[str] = Header(None)):
    """
    Validate session token and return user data
    Expects: Authorization: Bearer <token>

    Returns authenticated=false for invalid tokens (graceful degradation)
    Supports optional login per Constitution Principle X
    """
    if not authorization:
        return {"authenticated": False, "user": None}

    # Extract token from "Bearer <token>"
    try:
        scheme, token = authorization.split()
        if scheme.lower() != "bearer":
            return {"authenticated": False, "user": None}
    except ValueError:
        return {"authenticated": False, "user": None}

    # Verify token and get user
    user = auth_service.get_user_by_token(token)

    if not user:
        return {"authenticated": False, "user": None}

    return {
        "authenticated": True,
        "user": user.to_dict()
    }
