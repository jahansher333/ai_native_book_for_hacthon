from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from uuid import UUID


class Source(BaseModel):
    """Source citation model"""
    title: str
    url: str
    chunk_id: int
    score: float


class ChatMessage(BaseModel):
    """Model for individual chat messages"""
    id: int
    session_id: UUID
    role: str  # "user" or "assistant"
    content: str
    sources: Optional[List[Source]] = None
    created_at: datetime

    class Config:
        json_schema_extra = {
            "example": {
                "id": 1,
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "role": "user",
                "content": "What is ROS 2?",
                "sources": None,
                "created_at": "2025-12-04T10:00:00Z"
            }
        }


class ChatSession(BaseModel):
    """Model for chat sessions"""
    id: UUID
    created_at: datetime
    last_activity: datetime
    expires_at: datetime
    metadata: dict = {}

    class Config:
        json_schema_extra = {
            "example": {
                "id": "550e8400-e29b-41d4-a716-446655440000",
                "created_at": "2025-12-04T10:00:00Z",
                "last_activity": "2025-12-04T10:15:00Z",
                "expires_at": "2025-12-11T10:00:00Z",
                "metadata": {}
            }
        }
