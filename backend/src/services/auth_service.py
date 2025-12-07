"""
Authentication service for user signup, signin, and session management
Uses python-jose for JWT tokens and passlib for bcrypt password hashing
"""
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
import uuid

from ..config import settings
from ..models.user import User, Base

# Password hashing context (bcrypt with cost factor 12)
# truncate_error: pass password truncation errors through without raising
pwd_context = CryptContext(
    schemes=["bcrypt"],
    deprecated="auto",
    bcrypt__truncate_error=False  # Allow passwords > 72 bytes (they'll be auto-truncated)
)

# Database connection (reuse existing Neon URL)
engine = create_engine(settings.neon_database_url)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

class AuthService:
    """Authentication service for signup, signin, session management"""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash password using bcrypt (cost factor 12)
        Truncates to 72 bytes to comply with bcrypt limitation"""
        # Bcrypt has 72-byte limit - manually truncate before hashing
        # This is necessary due to Python 3.14 compatibility issues with passlib
        if len(password.encode('utf-8')) > 72:
            # Truncate at byte level, then decode back
            password = password.encode('utf-8')[:72].decode('utf-8', errors='ignore')
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify password against bcrypt hash"""
        # Match truncation from hash_password
        if len(plain_password.encode('utf-8')) > 72:
            plain_password = plain_password.encode('utf-8')[:72].decode('utf-8', errors='ignore')
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
        required_keys = ["hasRTX", "hasJetson", "hasRobot", "experience"]
        for key in required_keys:
            if key not in profile:
                raise ValueError(f"Missing required profile field: {key}")

        # Validate experience enum
        valid_experience = ["beginner", "intermediate", "advanced"]
        if profile["experience"] not in valid_experience:
            raise ValueError(f"Invalid experience. Must be one of: {valid_experience}")

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
