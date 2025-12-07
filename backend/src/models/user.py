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
            "created_at": self.created_at.isoformat() if self.created_at else None
        }

    def get_profile_summary(self):
        """Human-readable profile summary for welcome message"""
        profile = self.profile or {}
        hardware = []
        if profile.get("hasRTX"):
            hardware.append("RTX GPU")
        if profile.get("hasJetson"):
            hardware.append("Jetson Orin Nano")
        if profile.get("hasRobot"):
            hardware.append("Real Robot")

        hardware_str = ", ".join(hardware) if hardware else "No hardware yet"
        experience = profile.get("experience", "beginner").capitalize()

        return f"{experience} developer with {hardware_str}"
