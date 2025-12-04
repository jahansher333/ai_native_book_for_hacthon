from pydantic import BaseModel, Field
from typing import Optional, List
from uuid import UUID


class Source(BaseModel):
    """Source citation for RAG answers"""
    title: str
    url: str
    chunk_id: int
    score: float


class QueryRequest(BaseModel):
    """Request model for /query endpoint"""
    question: str = Field(..., min_length=1, max_length=1000)
    selected_text: Optional[str] = Field(None, max_length=2000)
    session_id: str
    module_filter: str = "all"

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is ROS 2?",
                "selected_text": None,
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "module_filter": "all"
            }
        }


class QueryResponse(BaseModel):
    """Response model for /query endpoint"""
    answer: str
    sources: List[Source]
    confidence: float = Field(..., ge=0, le=1)
    processing_time_ms: int

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 is the industry-standard middleware... [Source: Module 1]",
                "sources": [
                    {
                        "title": "Module 1 - ROS 2 Fundamentals",
                        "url": "/docs/ros2",
                        "chunk_id": 12345,
                        "score": 0.92
                    }
                ],
                "confidence": 0.88,
                "processing_time_ms": 1850
            }
        }
