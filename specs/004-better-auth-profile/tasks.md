# Tasks: Better-Auth Signup/Signin with Profile Questions

**Input**: Design documents from `/specs/004-better-auth-profile/`
**Prerequisites**: plan.md ✅, spec.md ✅
**Branch**: `004-better-auth-profile`
**Feature**: 50 Bonus Points - Authentication with Personalization

---

## Format: `[ID] [P?] [Phase] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Phase]**: Which implementation phase this task belongs to
- Include exact file paths and acceptance criteria in descriptions

---

## Phase 1: Backend Auth Setup (Tasks 1-5) 🎯 Priority: P1

**Purpose**: Install authentication libraries and create database infrastructure for user management with profile storage

**⚠️ CRITICAL**: This phase MUST complete before frontend forms can be implemented (frontend needs working API endpoints to test against)

---

### T001 [Setup] Install Python authentication libraries

**Description**: Install `python-jose` (JWT token generation) and `passlib` (bcrypt password hashing) in backend.

**Note**: Plan made architectural decision to use Python-native auth instead of Better-Auth Node.js for simpler FastAPI integration (40% faster implementation, same security).

**Commands**:
```bash
cd backend
pip install python-jose[cryptography]==3.3.0
pip install passlib[bcrypt]==1.7.4
pip freeze | grep -E "python-jose|passlib" >> requirements.txt
```

**Acceptance Criteria**:
- [ ] `python-jose[cryptography]` installed and added to `backend/requirements.txt`
- [ ] `passlib[bcrypt]` installed and added to `backend/requirements.txt`
- [ ] Import test succeeds:
  ```python
  from jose import jwt
  from passlib.context import CryptContext
  print("Auth libraries loaded successfully")
  ```
- [ ] No dependency conflicts (run `pip check`)

**Validation Steps**:
1. Run `pip install` commands
2. Test imports in Python REPL
3. Verify `requirements.txt` updated

**Commit Message**: `feat(backend): install python-jose and passlib for auth (T001)`

---

### T002 [Migration] Create database migration for profile column

**File**: `backend/src/migrations/001_add_profile.sql`

**Description**: Create SQL migration script to add `profile` JSONB column to existing `users` table (if it exists) or create `users` table with profile column.

**Migration Script**:
```sql
-- Migration: Add profile column to users table
-- Date: 2025-12-06
-- Feature: 004-better-auth-profile

-- Create users table if not exists (Better-Auth equivalent structure)
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    profile JSONB DEFAULT '{}'::JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Add profile column if users table already exists but lacks it
ALTER TABLE users ADD COLUMN IF NOT EXISTS profile JSONB DEFAULT '{}'::JSONB;

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_users_profile ON users USING GIN (profile);

-- Create sessions table for JWT token management
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token TEXT NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_sessions_token ON sessions(token);
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);

-- Insert sample profile structure for documentation
COMMENT ON COLUMN users.profile IS 'JSONB structure: {"has_rtx_gpu": boolean, "has_jetson": boolean, "has_real_robot": boolean, "programming_experience": "beginner"|"intermediate"|"advanced", "profile_version": 1}';
```

**Acceptance Criteria**:
- [ ] File exists at `backend/src/migrations/001_add_profile.sql`
- [ ] SQL is idempotent (can run multiple times safely via `IF NOT EXISTS`)
- [ ] Creates `users` table with profile JSONB column
- [ ] Creates `sessions` table for session management
- [ ] Creates 4 indexes for performance (email, profile GIN, token, user_id)
- [ ] No SQL syntax errors (test with `psql` or online SQL validator)

**Validation Steps**:
1. Create migration file with exact SQL above
2. Validate SQL syntax (copy to SQL editor or use `sqlfluff lint`)
3. Document migration in `backend/src/migrations/README.md` (if exists)

**Commit Message**: `feat(db): add profile JSONB column migration (T002)

Migration creates:
- users table (id, email, password_hash, profile JSONB)
- sessions table (id, user_id, token, expires_at)
- Indexes for performance (email, profile GIN, token)
- Idempotent (IF NOT EXISTS for safe re-runs)

Profile JSONB structure:
{
  "has_rtx_gpu": boolean,
  "has_jetson": boolean,
  "has_real_robot": boolean,
  "programming_experience": "beginner|intermediate|advanced",
  "profile_version": 1
}`

---

### T003 [P] [Config] Add JWT configuration to backend

**File**: `backend/src/config.py`

**Description**: Add JWT secret, algorithm, and expiration settings to backend configuration.

**Changes to `.env`**:
```bash
# Add to backend/.env (DO NOT COMMIT - add to .env.example only)
JWT_SECRET=<generate-with-openssl-rand-hex-32>
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7
```

**Changes to `backend/src/config.py`**:
```python
# Add JWT settings to Settings class
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Existing settings...
    gemini_api_key: str
    neon_database_url: str
    # ... other existing fields ...

    # NEW: JWT Authentication Settings
    jwt_secret: str = Field(..., env="JWT_SECRET")
    jwt_algorithm: str = Field(default="HS256", env="JWT_ALGORITHM")
    jwt_expiration_days: int = Field(default=7, env="JWT_EXPIRATION_DAYS")

    class Config:
        env_file = ".env"
        case_sensitive = False

# Global settings instance
settings = Settings()
```

**Changes to `.env.example`**:
```bash
# Add to backend/.env.example
JWT_SECRET=your-secret-key-generate-with-openssl-rand-hex-32
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7
```

**Acceptance Criteria**:
- [ ] `JWT_SECRET`, `JWT_ALGORITHM`, `JWT_EXPIRATION_DAYS` added to `config.py` Settings class
- [ ] `.env.example` updated with placeholder values (not actual secret)
- [ ] `.env` has actual `JWT_SECRET` (generate with `openssl rand -hex 32`)
- [ ] Configuration loads successfully (test with `python -c "from src.config import settings; print(settings.jwt_secret)"`)

**Validation Steps**:
1. Generate secret: `openssl rand -hex 32` → Add to `.env` (NOT .env.example)
2. Update `config.py` with JWT fields
3. Test configuration loads: `python -c "from src.config import settings; print(settings.jwt_secret[:10])"`

**Commit Message**: `feat(config): add JWT configuration for authentication (T003)

- Add jwt_secret, jwt_algorithm, jwt_expiration_days to Settings
- Update .env.example with placeholders
- JWT_SECRET generated with openssl rand -hex 32 (256-bit security)
- Expiration: 7 days (balances security and UX)`

---

### T004 [P] [Model] Create User SQLAlchemy model

**File**: `backend/src/models/user.py`

**Description**: Create SQLAlchemy model for `users` table with profile JSONB column.

**Implementation**:
```python
"""
User model for authentication
Stores user credentials and profile (hardware + experience)
"""
from sqlalchemy import Column, String, Boolean, DateTime
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    email_verified = Column(Boolean, default=False)
    profile = Column(JSONB, nullable=False, default={})
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    def __repr__(self):
        return f"<User(email='{self.email}', verified={self.email_verified})>"

    def to_dict(self):
        """Convert User to dict (for API responses)"""
        return {
            "id": str(self.id),
            "email": self.email,
            "email_verified": self.email_verified,
            "profile": self.profile,
            "created_at": self.created_at.isoformat()
        }

    def get_profile_summary(self):
        """Human-readable profile summary for welcome message"""
        profile = self.profile or {}
        hardware = []
        if profile.get("has_rtx_gpu"):
            hardware.append("RTX GPU")
        if profile.get("has_jetson"):
            hardware.append("Jetson")
        if profile.get("has_real_robot"):
            hardware.append("Real Robot")

        hardware_str = ", ".join(hardware) if hardware else "No hardware yet"
        experience = profile.get("programming_experience", "beginner").capitalize()

        return f"{experience} User with {hardware_str}"
```

**Acceptance Criteria**:
- [ ] File exists at `backend/src/models/user.py`
- [ ] User model has all required columns (id, email, password_hash, profile JSONB, timestamps)
- [ ] `to_dict()` method converts User to JSON-serializable dict
- [ ] `get_profile_summary()` returns formatted string (e.g., "Advanced User with Jetson, RTX GPU")
- [ ] Model imports successfully: `from src.models.user import User`

**Validation Steps**:
1. Create file with code above
2. Test import: `python -c "from backend.src.models.user import User; print(User.__tablename__)"`
3. Verify `to_dict()` method works

**Commit Message**: `feat(models): create User model with profile JSONB (T004)

- SQLAlchemy model for users table
- Columns: id (UUID), email, password_hash, profile (JSONB), timestamps
- Methods: to_dict() for API responses, get_profile_summary() for welcome messages
- Profile structure: {has_rtx_gpu, has_jetson, has_real_robot, programming_experience}`

---

### T005 [Service] Create auth service with JWT and password hashing

**File**: `backend/src/services/auth_service.py`

**Description**: Create authentication service with functions for password hashing (bcrypt), JWT token creation/validation, and user creation with profile storage.

**Implementation** (complete service):
```python
"""
Authentication service for user signup, signin, and session management
Uses python-jose for JWT tokens and passlib for bcrypt password hashing
"""
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy import create_engine, select
from sqlalchemy.orm import sessionmaker
import uuid

from ..config import settings
from ..models.user import User, Base

# Password hashing context (bcrypt with cost factor 12)
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Database connection (reuse existing Neon URL)
engine = create_engine(settings.neon_database_url)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

class AuthService:
    """Authentication service for signup, signin, session management"""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash password using bcrypt (cost factor 12)"""
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify password against bcrypt hash"""
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def create_access_token(user_id: str, email: str) -> tuple[str, datetime]:
        """
        Create JWT access token
        Returns: (token, expiration_datetime)
        """
        expires_delta = timedelta(days=settings.jwt_expiration_days)
        expire = datetime.utcnow() + expires_delta

        to_encode = {
            "user_id": user_id,
            "email": email,
            "exp": expire,
            "iat": datetime.utcnow(),
            "sub": user_id
        }

        token = jwt.encode(to_encode, settings.jwt_secret, algorithm=settings.jwt_algorithm)
        return token, expire

    @staticmethod
    def verify_token(token: str) -> Optional[Dict[str, Any]]:
        """
        Verify JWT token and return payload
        Returns: payload dict if valid, None if invalid/expired
        """
        try:
            payload = jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
            return payload
        except JWTError:
            return None

    @staticmethod
    def create_user_with_profile(
        email: str,
        password: str,
        profile: dict
    ) -> tuple[User, str, datetime]:
        """
        Create user with profile in database
        Returns: (user, token, expiration)
        Raises: ValueError if email exists or profile invalid
        """
        # Validate profile structure
        required_keys = ["has_rtx_gpu", "has_jetson", "has_real_robot", "programming_experience"]
        for key in required_keys:
            if key not in profile:
                raise ValueError(f"Missing required profile field: {key}")

        # Validate programming_experience enum
        valid_experience = ["beginner", "intermediate", "advanced"]
        if profile["programming_experience"] not in valid_experience:
            raise ValueError(f"Invalid programming_experience. Must be one of: {valid_experience}")

        # Add metadata to profile
        profile["profile_version"] = 1
        profile["created_at"] = datetime.utcnow().isoformat()

        db = SessionLocal()
        try:
            # Check if email already exists
            existing_user = db.query(User).filter(User.email == email).first()
            if existing_user:
                raise ValueError("Email already exists")

            # Hash password
            password_hash = AuthService.hash_password(password)

            # Create user
            user = User(
                id=uuid.uuid4(),
                email=email,
                password_hash=password_hash,
                profile=profile,
                email_verified=False
            )

            db.add(user)
            db.commit()
            db.refresh(user)

            # Create access token
            token, expires_at = AuthService.create_access_token(str(user.id), user.email)

            return user, token, expires_at

        finally:
            db.close()

    @staticmethod
    def authenticate_user(email: str, password: str) -> Optional[tuple[User, str, datetime]]:
        """
        Authenticate user with email + password
        Returns: (user, token, expiration) if valid, None if invalid
        """
        db = SessionLocal()
        try:
            # Find user by email
            user = db.query(User).filter(User.email == email).first()

            if not user:
                return None  # Email not found

            # Verify password
            if not AuthService.verify_password(password, user.password_hash):
                return None  # Wrong password

            # Create new access token
            token, expires_at = AuthService.create_access_token(str(user.id), user.email)

            return user, token, expires_at

        finally:
            db.close()

    @staticmethod
    def get_user_by_token(token: str) -> Optional[User]:
        """
        Get user by JWT token (for session validation)
        Returns: User if token valid, None if invalid/expired
        """
        payload = AuthService.verify_token(token)
        if not payload:
            return None

        user_id = payload.get("user_id")
        if not user_id:
            return None

        db = SessionLocal()
        try:
            user = db.query(User).filter(User.id == uuid.UUID(user_id)).first()
            return user
        finally:
            db.close()


# Global auth service instance
auth_service = AuthService()
```

**Acceptance Criteria**:
- [ ] File exists at `backend/src/services/auth_service.py`
- [ ] `hash_password()` produces different hashes for same password (bcrypt salt randomization)
- [ ] `verify_password()` correctly validates passwords
- [ ] `create_access_token()` generates valid JWT with 7-day expiration
- [ ] `verify_token()` decodes valid tokens, rejects invalid/expired
- [ ] `create_user_with_profile()` creates user in database with profile JSONB
- [ ] `authenticate_user()` validates credentials and returns token
- [ ] `get_user_by_token()` retrieves user from valid JWT

**Validation Steps**:
1. Test password hashing:
   ```python
   from backend.src.services.auth_service import auth_service
   hash1 = auth_service.hash_password("test123")
   hash2 = auth_service.hash_password("test123")
   assert hash1 != hash2  # Different salts
   assert auth_service.verify_password("test123", hash1)  # Verify works
   ```
2. Test JWT creation and validation
3. Test user creation (requires database connection)

**Commit Message**: `feat(auth): create auth service with JWT and bcrypt (T005)

Complete authentication service:
- hash_password(): bcrypt with cost factor 12
- verify_password(): validate against hash
- create_access_token(): JWT with 7-day expiration
- verify_token(): decode and validate JWT
- create_user_with_profile(): signup with 4-question profile
- authenticate_user(): signin with credentials
- get_user_by_token(): session validation

Profile validation ensures all 4 required fields present.`

---

**Checkpoint Phase 1**: Auth libraries installed, migration created, auth service implemented. Can proceed to Phase 2 (API endpoints).

---

## Phase 2: Backend API Endpoints (Tasks 6-8) 🎯 Priority: P1

**Purpose**: Create FastAPI endpoints for signup, signin, and session validation

**All tasks in this phase can run in PARALLEL** (different endpoints, no dependencies)

---

### T006 [P] [API] Create signup endpoint with profile storage

**File**: `backend/src/api/auth.py`

**Description**: Create FastAPI router with signup endpoint that captures email + password + 4 profile questions.

**Implementation**:
```python
"""
Authentication API endpoints
Handles signup, signin, and session validation
"""
from fastapi import APIRouter, HTTPException, Header
from pydantic import BaseModel, EmailStr, Field
from typing import Optional
from datetime import datetime

from ..services.auth_service import auth_service

router = APIRouter(prefix="/api/auth", tags=["authentication"])

# Request/Response Models
class ProfileData(BaseModel):
    has_rtx_gpu: bool = Field(..., description="User owns NVIDIA RTX GPU")
    has_jetson: bool = Field(..., description="User owns Jetson Orin Nano")
    has_real_robot: bool = Field(..., description="User has access to real robot")
    programming_experience: str = Field(..., description="beginner | intermediate | advanced")

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
    """
    try:
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
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Database errors or unexpected issues
        raise HTTPException(status_code=500, detail="Signup failed. Please try again.")
```

**Acceptance Criteria**:
- [ ] File exists at `backend/src/api/auth.py`
- [ ] Signup endpoint defined: `POST /api/auth/signup`
- [ ] Pydantic models validate request structure (email format, password length, profile fields)
- [ ] Endpoint calls `auth_service.create_user_with_profile()`
- [ ] Returns 201 with user + token on success
- [ ] Returns 400 with error message on validation failure (duplicate email, short password)
- [ ] Profile stored in database as JSONB

**Validation Steps**:
1. Test with curl:
   ```bash
   curl -X POST http://localhost:8000/api/auth/signup \
     -H "Content-Type: application/json" \
     -d '{
       "email": "test@example.com",
       "password": "SecurePass123",
       "profile": {
         "has_rtx_gpu": false,
         "has_jetson": false,
         "has_real_robot": false,
         "programming_experience": "beginner"
       }
     }'
   ```
2. Verify 201 response with token
3. Check database: `SELECT email, profile FROM users WHERE email='test@example.com'`

**Commit Message**: `feat(api): add signup endpoint with profile questions (T006)

POST /api/auth/signup endpoint:
- Accepts email + password + profile (4 questions)
- Validates email format, password length (min 8), profile structure
- Creates user in Neon Postgres with profile JSONB
- Returns JWT token (7-day expiration)
- Error handling: 400 for validation, 500 for server errors

Profile questions:
1. has_rtx_gpu (boolean)
2. has_jetson (boolean)
3. has_real_robot (boolean)
4. programming_experience (beginner|intermediate|advanced)`

---

### T007 [P] [API] Create signin endpoint

**File**: `backend/src/api/auth.py` (add to existing file from T006)

**Description**: Add signin endpoint to auth router.

**Implementation** (add to `auth.py`):
```python
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
```

**Acceptance Criteria**:
- [ ] Signin endpoint added: `POST /api/auth/signin`
- [ ] Accepts email + password only (no profile questions for signin)
- [ ] Returns 200 with user + profile + token on success
- [ ] Returns 401 with generic error on failure (same message for wrong email or wrong password)
- [ ] Token includes user profile from database

**Validation Steps**:
1. Signup a test user first (T006)
2. Test signin with curl:
   ```bash
   curl -X POST http://localhost:8000/api/auth/signin \
     -H "Content-Type: application/json" \
     -d '{"email": "test@example.com", "password": "SecurePass123"}'
   ```
3. Verify 200 response with token + profile
4. Test wrong password → 401 error

**Commit Message**: `feat(api): add signin endpoint with profile retrieval (T007)

POST /api/auth/signin endpoint:
- Accepts email + password
- Validates credentials against database
- Returns user + profile + JWT token
- Generic error message (prevents email enumeration)
- Profile automatically included in response`

---

### T008 [P] [API] Create session validation endpoint

**File**: `backend/src/api/auth.py` (add to existing file)

**Description**: Add session validation endpoint that verifies JWT tokens and returns user + profile.

**Implementation** (add to `auth.py`):
```python
@router.get("/session", response_model=SessionResponse)
async def get_session(authorization: Optional[str] = Header(None)):
    """
    Validate session token and return user data
    Expects: Authorization: Bearer <token>
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
```

**Acceptance Criteria**:
- [ ] Session endpoint added: `GET /api/auth/session`
- [ ] Accepts `Authorization: Bearer <token>` header
- [ ] Returns 200 with `authenticated: true` and user data if token valid
- [ ] Returns 200 with `authenticated: false` if token invalid/expired
- [ ] Does not return 401 (graceful degradation for optional login)

**Validation Steps**:
1. Get token from signin (T007)
2. Test session validation:
   ```bash
   TOKEN="<jwt-token-from-signin>"
   curl -X GET http://localhost:8000/api/auth/session \
     -H "Authorization: Bearer $TOKEN"
   ```
3. Verify response includes user + profile
4. Test with invalid token → `authenticated: false`

**Commit Message**: `feat(api): add session validation endpoint (T008)

GET /api/auth/session endpoint:
- Validates JWT token from Authorization header
- Returns user + profile if token valid
- Returns authenticated: false if token invalid (graceful degradation)
- Supports optional login (no 401 errors)`

---

**Checkpoint Phase 2**: All 3 auth endpoints implemented (signup, signin, session). Backend API complete. Can proceed to Phase 3 (Frontend forms).

---

## Phase 3: Frontend Auth Forms (Tasks 9-10) 🎯 Priority: P1

**Purpose**: Create React components for signup and signin with Docusaurus integration

---

### T009 [Signup] Create signup form with 4 profile questions

**File**: `frontend/src/components/Auth/SignupForm.tsx`

**Description**: Create React signup form component with email, password, and 4 profile questions.

**Implementation** (complete component - see full code in spec.md FR-006):

**Key Features**:
- Email input with format validation
- Password input with show/hide toggle (eye icon)
- 4 profile questions:
  1. RadioGroup: "Do you have an RTX GPU?" (yes/no)
  2. RadioGroup: "Do you own a Jetson Orin Nano?" (yes/no)
  3. RadioGroup: "Do you have access to a real robot?" (yes/no)
  4. Dropdown: "Your programming experience?" (beginner/intermediate/advanced)
- Client-side validation before API call
- Error message display (inline, red text)
- Loading state during submission
- Redirect to `/docs/intro?personalized=true` on success

**Styling File**: `frontend/src/css/auth.module.css`

**Acceptance Criteria**:
- [ ] File exists at `frontend/src/components/Auth/SignupForm.tsx`
- [ ] Component renders all inputs (email, password, 4 questions)
- [ ] Client-side validation prevents invalid submissions
- [ ] Successful signup stores token in localStorage
- [ ] Redirects to `/docs/intro?personalized=true` after signup
- [ ] Error messages display inline (email validation, password length, required questions)
- [ ] Show/hide password toggle works (eye icon)
- [ ] Styling matches Docusaurus theme (light/dark mode compatible)

**Validation Steps**:
1. Create component file (refer to spec.md FR-006 for full implementation)
2. Create dedicated page: `frontend/src/pages/signup.tsx`
3. Test form renders: `npm start` → visit `http://localhost:3000/signup`
4. Test validation: Try submitting with invalid email, short password
5. Test signup flow: Fill valid data → Submit → Verify redirect and localStorage token

**Commit Message**: `feat(frontend): add signup form with 4 profile questions (T009)

React component for user registration:
- Email + password inputs with validation
- 4 profile questions (RTX GPU, Jetson, robot, experience)
- Client-side validation (email format, password min 8 chars)
- Show/hide password toggle
- Error handling with inline messages
- Stores JWT token in localStorage
- Redirects to /docs/intro?personalized=true on success
- CSS Module styling (dark/light mode compatible)

Profile questions enable personalization:
1. Do you have an RTX GPU? (yes/no)
2. Do you own a Jetson Orin Nano? (yes/no)
3. Do you have access to a real robot? (yes/no)
4. Your programming experience? (beginner/intermediate/advanced)`

---

### T010 [Signin] Create signin form (simple email + password)

**File**: `frontend/src/components/Auth/SigninForm.tsx`

**Description**: Create React signin form component with just email and password (no profile questions for signin).

**Implementation** (simplified version of signup form):
```typescript
import React, { useState } from 'react';
import styles from '../../css/auth.module.css';

export default function SigninForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password })
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.detail || 'Invalid email or password');
        setLoading(false);
        return;
      }

      // Store token
      localStorage.setItem('auth_token', data.token);
      localStorage.setItem('user_profile', JSON.stringify(data.user.profile));

      // Redirect to last visited page or intro
      const lastPage = localStorage.getItem('last_page') || '/docs/intro';
      window.location.href = lastPage;

    } catch (err) {
      setError('Network error. Please try again.');
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.signinForm}>
      <h2>Welcome Back</h2>
      <p className={styles.subtitle}>Sign in to access your personalized learning path</p>

      {/* Email */}
      <div className={styles.formGroup}>
        <label htmlFor="email">Email</label>
        <input
          type="email"
          id="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="student@example.com"
          required
        />
      </div>

      {/* Password */}
      <div className={styles.formGroup}>
        <label htmlFor="password">Password</label>
        <div className={styles.passwordInput}>
          <input
            type={showPassword ? 'text' : 'password'}
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="••••••••"
            required
          />
          <button
            type="button"
            className={styles.togglePassword}
            onClick={() => setShowPassword(!showPassword)}
            aria-label="Toggle password visibility"
          >
            {showPassword ? '👁️' : '👁️‍🗨️'}
          </button>
        </div>
      </div>

      {/* Error Message */}
      {error && <div className={styles.errorMessage}>{error}</div>}

      {/* Submit */}
      <button type="submit" className={styles.submitButton} disabled={loading}>
        {loading ? 'Signing in...' : 'Sign In'}
      </button>

      {/* Signup Link */}
      <p className={styles.footerLink}>
        Don't have an account? <a href="/signup">Sign up</a>
      </p>
    </form>
  );
}
```

**Acceptance Criteria**:
- [ ] File exists at `frontend/src/components/Auth/SigninForm.tsx`
- [ ] Component renders email + password inputs
- [ ] Show/hide password toggle works
- [ ] Successful signin stores token in localStorage
- [ ] Redirects to last visited page or `/docs/intro`
- [ ] Error message displays for invalid credentials: "Invalid email or password"
- [ ] Styling matches signup form (same CSS module)

**Validation Steps**:
1. Create component file
2. Create page: `frontend/src/pages/signin.tsx`
3. Test form renders: `npm start` → visit `/signin`
4. Test signin with valid credentials (from T006 signup)
5. Verify token stored in localStorage and redirect works

**Commit Message**: `feat(frontend): add signin form component (T010)

Simple signin form:
- Email + password inputs
- Show/hide password toggle
- Stores JWT token + user profile in localStorage
- Redirects to last visited page or /docs/intro
- Error handling: "Invalid email or password" (generic for security)
- Links to signup form for new users`

---

**Checkpoint Phase 3**: Signup and signin forms implemented. Can proceed to Phase 4 (Navbar integration).

---

## Phase 4: Navbar & Pages Integration (Tasks 11-12) 🎯 Priority: P1

**Purpose**: Add authentication buttons to Docusaurus navbar and create dedicated pages

---

### T011 [Navbar] Swizzle Docusaurus Navbar and add auth buttons

**Description**: Swizzle (eject) Docusaurus Navbar component to add dynamic "Sign Up" / "Sign In" or "Profile" / "Sign Out" buttons based on authentication state.

**Commands**:
```bash
cd frontend

# Swizzle Navbar component (eject for full control)
npm run swizzle @docusaurus/theme-classic Navbar -- --eject

# This creates: frontend/src/theme/Navbar/index.tsx
```

**Modified Navbar** (add auth logic):
```typescript
// frontend/src/theme/Navbar/index.tsx
import React, { useState, useEffect } from 'react';
import NavbarOriginal from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): JSX.Element {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [userEmail, setUserEmail] = useState('');
  const [profileSummary, setProfileSummary] = useState('');

  useEffect(() => {
    // Check localStorage for auth token
    const token = localStorage.getItem('auth_token');
    const profileData = localStorage.getItem('user_profile');

    if (token && profileData) {
      // Validate token with backend (optional, for extra security)
      // For MVP: Trust localStorage (validates on page load)

      setIsAuthenticated(true);

      try {
        const profile = JSON.parse(profileData);
        // Generate welcome message
        const hardware = [];
        if (profile.has_rtx_gpu) hardware.push('RTX GPU');
        if (profile.has_jetson) hardware.push('Jetson');
        if (profile.has_real_robot) hardware.push('Real Robot');

        const hardwareStr = hardware.length > 0 ? hardware.join(', ') : 'No hardware yet';
        const experience = profile.programming_experience || 'beginner';
        setProfileSummary(`${experience.charAt(0).toUpperCase() + experience.slice(1)} User with ${hardwareStr}`);

        // Extract email from token (or fetch from /api/auth/session)
        // For MVP: Decode JWT client-side (public info only)
        setUserEmail('User'); // Simplified for MVP

      } catch (e) {
        console.error('Failed to parse profile:', e);
      }
    }
  }, []);

  const handleSignOut = () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_profile');
    window.location.href = '/';
  };

  // Inject auth buttons into navbar
  const navbarItems = isAuthenticated
    ? [
        // Authenticated: Show Profile dropdown
        {
          type: 'dropdown',
          label: profileSummary || 'Profile',
          position: 'right',
          items: [
            { label: userEmail, to: '/profile' },
            { label: 'Sign Out', onClick: handleSignOut },
          ],
        },
      ]
    : [
        // Anonymous: Show Sign Up / Sign In
        { type: 'link', label: 'Sign Up', to: '/signup', position: 'right', className: 'navbar-signup-button' },
        { type: 'link', label: 'Sign In', to: '/signin', position: 'right' },
      ];

  // Merge with existing navbar items
  const modifiedProps = {
    ...props,
    // Note: This is simplified. Actual implementation requires merging with existing themeConfig
  };

  return (
    <>
      <NavbarOriginal {...props} />
      {/* Custom auth buttons (simplified approach for MVP) */}
      <div style={{ position: 'absolute', right: '1rem', top: '0.5rem' }}>
        {isAuthenticated ? (
          <div>
            <span style={{ marginRight: '1rem' }}>{profileSummary}</span>
            <button onClick={handleSignOut}>Sign Out</button>
          </div>
        ) : (
          <div>
            <a href="/signup" style={{ marginRight: '1rem' }}>Sign Up</a>
            <a href="/signin">Sign In</a>
          </div>
        )}
      </div>
    </>
  );
}
```

**Note**: Above is simplified. Production implementation requires properly integrating with Docusaurus navbar items configuration. Alternative: Modify `docusaurus.config.js` instead of swizzling.

**Alternative Approach** (simpler for MVP):
Modify `docusaurus.config.js` to add navbar items:
```javascript
// frontend/docusaurus.config.js
module.exports = {
  themeConfig: {
    navbar: {
      items: [
        // ... existing items ...
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'navbar-signup-button',
        },
        {
          to: '/signin',
          label: 'Sign In',
          position: 'right',
        },
      ],
    },
  },
};
```

Then handle dynamic visibility in custom component or CSS.

**Acceptance Criteria**:
- [ ] Navbar shows "Sign Up" and "Sign In" buttons for anonymous users
- [ ] Navbar shows "Profile" dropdown with email and "Sign Out" for logged-in users
- [ ] "Sign Out" button clears localStorage and redirects to `/`
- [ ] Navbar updates immediately after signup/signin (no page refresh needed)
- [ ] Styling matches Docusaurus theme

**Validation Steps**:
1. Visit site as anonymous user → See "Sign Up" / "Sign In"
2. Complete signup → Navbar updates to show "Profile" / "Sign Out"
3. Click "Sign Out" → Navbar reverts to "Sign Up" / "Sign In"

**Commit Message**: `feat(navbar): add dynamic auth buttons with profile summary (T011)

Swizzled Docusaurus Navbar to add authentication:
- Anonymous users: "Sign Up" | "Sign In" buttons
- Logged-in users: "Profile" dropdown with welcome message
- Profile summary: "Advanced User with Jetson, RTX GPU"
- Sign Out: Clears localStorage, redirects to homepage
- Updates immediately after signup/signin (no refresh needed)`

---

### T012 [Pages] Create signup and signin pages

**Files**:
- `frontend/src/pages/signup.tsx`
- `frontend/src/pages/signin.tsx`

**Description**: Create dedicated Docusaurus pages for signup and signin routes.

**Implementation**:

**Signup Page** (`frontend/src/pages/signup.tsx`):
```typescript
import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';

export default function SignupPage() {
  return (
    <Layout
      title="Sign Up - Physical AI Textbook"
      description="Create your account to get personalized learning recommendations"
    >
      <div className="container" style={{ marginTop: '2rem', maxWidth: '500px' }}>
        <SignupForm />
      </div>
    </Layout>
  );
}
```

**Signin Page** (`frontend/src/pages/signin.tsx`):
```typescript
import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/Auth/SigninForm';

export default function SigninPage() {
  return (
    <Layout
      title="Sign In - Physical AI Textbook"
      description="Sign in to access your personalized learning path"
    >
      <div className="container" style={{ marginTop: '2rem', maxWidth: '500px' }}>
        <SigninForm />
      </div>
    </Layout>
  );
}
```

**Acceptance Criteria**:
- [ ] Signup page exists at `frontend/src/pages/signup.tsx`
- [ ] Signin page exists at `frontend/src/pages/signin.tsx`
- [ ] Both pages render with Docusaurus Layout (header, footer, sidebar)
- [ ] Pages accessible at `/signup` and `/signin` routes
- [ ] Forms centered on page (max-width: 500px)
- [ ] Meta tags set correctly (title, description for SEO)

**Validation Steps**:
1. Create both page files
2. Start dev server: `npm start`
3. Visit `/signup` → Form renders in Docusaurus layout
4. Visit `/signin` → Form renders
5. Test signup flow end-to-end
6. Test signin flow end-to-end

**Commit Message**: `feat(pages): add signup and signin pages (T012)

Dedicated Docusaurus pages for authentication:
- /signup route: SignupForm embedded in Docusaurus Layout
- /signin route: SigninForm embedded in Docusaurus Layout
- Both use Layout component (header, footer, sidebar)
- Centered forms (max-width: 500px)
- SEO meta tags (title, description)`

---

**Checkpoint Phase 4**: All frontend pages created. Complete auth flow functional (signup → signin → navbar updates). Can proceed to Phase 5 (Integration & Testing).

---

## Phase 5: Integration & Validation (Implied, not separate tasks)

**Purpose**: Run database migration, test complete auth flow, validate with Docusaurus build

**Integration Steps** (performed after T001-T012):

### Step 1: Run Database Migration

```bash
# Connect to Neon Postgres
psql $NEON_DATABASE_URL

# Run migration
\i backend/src/migrations/001_add_profile.sql

# Verify tables created
\dt  # Should show: users, sessions

# Verify profile column exists
\d users  # Should show: profile JSONB column

# Exit
\q
```

**Expected Result**: Tables created, profile column exists

---

### Step 2: Connect Backend to Database

**File**: `backend/src/main.py` (update)

**Add auth router**:
```python
# Add to imports
from .api import query, health, ingest, auth  # Add auth

# Add router
app.include_router(auth.router, prefix="", tags=["auth"])  # No prefix (already has /api/auth)
```

**Add migration runner** (optional, can run manually via psql):
```python
# In lifespan startup
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Starting RAG Chatbot Backend...")

    # Run database migrations
    try:
        from .services.auth_service import engine, Base
        Base.metadata.create_all(engine)  # Creates tables if not exist
        print("Database tables initialized")
    except Exception as e:
        print(f"WARNING: Database initialization: {e}")

    yield
```

---

### Step 3: Test Complete Auth Flow

**Scenario 1: Signup**
```bash
# 1. Start backend
cd backend
uvicorn src.main:app --reload

# 2. Test signup endpoint
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123",
    "profile": {
      "has_rtx_gpu": false,
      "has_jetson": false,
      "has_real_robot": false,
      "programming_experience": "beginner"
    }
  }'

# Expected: 201 response with user + token
```

**Scenario 2: Signin**
```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }'

# Expected: 200 response with user + profile + token
```

**Scenario 3: Session Validation**
```bash
TOKEN="<jwt-token-from-signin>"

curl -X GET http://localhost:8000/api/auth/session \
  -H "Authorization: Bearer $TOKEN"

# Expected: 200 response with authenticated: true, user + profile
```

---

### Step 4: Frontend End-to-End Test

```bash
# 1. Start backend (terminal 1)
cd backend
uvicorn src.main:app --reload

# 2. Start frontend (terminal 2)
cd frontend
npm start

# 3. Manual test flow:
# - Visit http://localhost:3000/signup
# - Fill form: email, password, answer 4 questions
# - Submit → Verify redirect to /docs/intro
# - Check localStorage: auth_token, user_profile stored
# - Visit /signin → Enter same credentials → Verify redirect
# - Check navbar: Should show "Profile" dropdown
# - Click "Sign Out" → Verify navbar shows "Sign Up" / "Sign In" again
```

---

### Step 5: Docusaurus Build Validation

```bash
cd frontend
npm run build

# Expected: Build succeeds with 0 errors
# Signup/signin pages should be in build output
```

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Backend Setup)**: No dependencies, all tasks can run in PARALLEL
  - T001: Install libraries
  - T002: Create migration
  - T003: Add JWT config (depends on T001)
  - T004: Create User model (can run parallel to T003)
  - T005: Create auth service (depends on T001, T003, T004)

- **Phase 2 (API Endpoints)**: Depends on Phase 1 complete, tasks run in PARALLEL
  - T006, T007, T008: All depend on T005 (auth_service)

- **Phase 3 (Frontend Forms)**: Can start in parallel with Phase 2, tasks run in PARALLEL
  - T009: Signup form (needs T006 API endpoint to test)
  - T010: Signin form (needs T007 API endpoint to test)

- **Phase 4 (Integration)**: Depends on Phase 2 and 3 complete
  - T011: Navbar (needs frontend forms to link to)
  - T012: Pages (needs forms to embed)

---

### Parallel Execution Strategy

#### Maximum Parallelism (3 developers)

**Developer A: Backend (T001-T005)**
```
T001: Install libraries → 15 min
T003: Add JWT config → 30 min (depends on T001)
T004: Create User model → 1 hour (parallel to T003)
T002: Create migration → 1 hour (parallel to T003, T004)
T005: Auth service → 3-4 hours (depends on T001, T003, T004)

Total: ~6-7 hours
```

**Developer B: Backend API (T006-T008)**
```
Wait for T005 complete, then:
T006: Signup endpoint → 2 hours (parallel to T007, T008)
T007: Signin endpoint → 1 hour (parallel to T006, T008)
T008: Session endpoint → 1 hour (parallel to T006, T007)

Total: ~2 hours (parallel) or ~4 hours (sequential)
```

**Developer C: Frontend (T009-T012)**
```
T009: Signup form → 3-4 hours (can start immediately, test with T006 when ready)
T010: Signin form → 2 hours (can start immediately, test with T007 when ready)
T011: Navbar swizzle → 2-3 hours (depends on T009, T010 for links)
T012: Create pages → 1 hour (depends on T009, T010 for components)

Total: ~8-10 hours
```

**Optimized Timeline**:
- **Hour 0-7**: Developer A (Backend setup)
- **Hour 0-10**: Developer C (Frontend forms, can start immediately)
- **Hour 7-9**: Developer B (API endpoints, starts after T005 complete)
- **Total wall-clock time**: ~10 hours (with 3 developers)

---

## Success Criteria (from Spec)

### Task-Level Success Criteria

Each task has acceptance criteria defined inline (checkboxes). Task is complete when ALL checkboxes are checked.

### Feature-Level Success Criteria (from spec.md)

- [ ] **SC-001**: Signup creates account with 4-question profile in Neon DB (tested with T006 + T009)
- [ ] **SC-002**: Profile stored in `users.profile` JSONB column (SQL query verification)
- [ ] **SC-003**: Signin authenticates users, returns token + profile (tested with T007 + T010)
- [ ] **SC-004**: Anonymous users read entire textbook without login (verify 20 pages)
- [ ] **SC-005**: Navbar shows auth state dynamically (T011 implementation)
- [ ] **SC-006**: After signup, redirect to `/docs/intro?personalized=true` (T009 implementation)
- [ ] **SC-007**: Personalized content appears based on profile (future enhancement, hook ready)
- [ ] **SC-008**: Database reuses existing `NEON_DATABASE_URL` (verified in T002, T005)
- [ ] **SC-009**: Database tables created successfully via migration (T002)
- [ ] **SC-010**: Form validation prevents invalid submissions (T009, T010 client-side validation)

**Current Status**: 0/10 complete (implementation pending)

---

## Notes

- **Python-native auth decision**: Plan chose `python-jose` + `passlib` over Better-Auth Node.js for simpler FastAPI integration (40% faster, same security)
- **Database reuse**: Migration extends existing Neon DB (no new database creation)
- **Profile storage**: JSONB column stores 4-question answers (hardware + experience)
- **Optional login**: Constitution Principle X requires book readable without authentication
- **Personalization hook**: Ready for future @personalizer subagent to generate custom content
- **Security**: bcrypt (cost 12), JWT (7-day expiration), CORS configured, generic error messages

---

## Estimated Effort

| Phase | Tasks | Estimated Effort |
|-------|-------|------------------|
| Phase 1: Backend Setup | T001-T005 | 6-8 hours |
| Phase 2: API Endpoints | T006-T008 | 4-6 hours (2h parallel, 4h sequential) |
| Phase 3: Frontend Forms | T009-T010 | 9-12 hours |
| Phase 4: Integration | T011-T012 | 5-6 hours |
| **Total** | **12 tasks** | **24-32 hours** |

**Optimized Parallel Execution**: ~16-20 hours (with 3 developers working in parallel)

---

**Tasks Version**: 1.0
**Last Updated**: 2025-12-06
**Author**: Claude Code (Opus 4.5)
**Status**: Ready for Implementation

**Next Steps**:
1. Run `/sp.implement` to execute tasks in order
2. Start with Phase 1 (Backend setup, can run T001-T004 in parallel)
3. Execute Phase 2 (API endpoints, T006-T008 in parallel)
4. Execute Phase 3 (Frontend forms, T009-T010 in parallel)
5. Execute Phase 4 (Integration, T011-T012 sequential)
6. Run database migration and test complete auth flow
7. Validate with 10 test signups and Docusaurus build
8. Deploy and earn 50 bonus points!
