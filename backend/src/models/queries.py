"""
Query and response data models for RAG chatbot API
Matches API contract from contracts/api.openapi.yaml
"""
from typing import Optional, Literal, List
from pydantic import BaseModel, Field
from datetime import datetime


class Citation(BaseModel):
    """Source reference for generated answer"""
    module: str = Field(..., description="Module name")
    section: str = Field(..., description="Section identifier")
    page: int = Field(..., description="Page number")
    source_file: str = Field(..., description="Source file path")
    chunk_id: str = Field(..., description="Qdrant chunk ID")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score")
    text_snippet: str = Field(..., max_length=200, description="First 100-200 chars of chunk")

    class Config:
        json_schema_extra = {
            "example": {
                "module": "ros2",
                "section": "introduction",
                "page": 8,
                "source_file": "module-1-ros2/intro.md",
                "chunk_id": "f9a3c2d1-4e5f-6789-abcd-ef1234567890",
                "relevance_score": 0.92,
                "text_snippet": "ROS 2 is the next generation..."
            }
        }


class QueryRequest(BaseModel):
    """Request model for POST /query endpoint"""
    question: str = Field(..., min_length=1, max_length=1000, description="User's question")
    mode: Literal["general", "selected"] = Field(default="general", description="Query mode")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Highlighted text (selected mode only)")
    session_id: Optional[str] = Field(None, description="Client session UUID for history")

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is ROS 2?",
                "mode": "general",
                "selected_text": None,
                "session_id": "b7c9e3f1-6a2d-4f8b-9c3e-1a2b3c4d5e6f"
            }
        }


class QueryResponse(BaseModel):
    """Response model for POST /query endpoint"""
    answer: str = Field(..., description="Generated response text")
    citations: List[Citation] = Field(default_factory=list, description="Source references")
    latency_ms: int = Field(..., description="Total query latency")
    query_id: str = Field(..., description="UUID for this query (logging)")
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 is the next generation of the Robot Operating System...",
                "citations": [
                    {
                        "module": "ros2",
                        "section": "introduction",
                        "page": 8,
                        "source_file": "module-1-ros2/intro.md",
                        "chunk_id": "f9a3c2d1-4e5f-6789-abcd-ef1234567890",
                        "relevance_score": 0.92,
                        "text_snippet": "ROS 2 is the next generation..."
                    }
                ],
                "latency_ms": 2341,
                "query_id": "c8d1e4f2-7b3e-5a9c-0d4f-2b3c4d5e6f7g",
                "timestamp": "2025-12-05T15:45:33Z"
            }
        }


class ErrorResponse(BaseModel):
    """Error response model"""
    error: str = Field(..., description="Error message")
    error_code: str = Field(..., description="Error code")
    details: Optional[dict] = Field(None, description="Additional error details")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Rate limit exceeded",
                "error_code": "RATE_LIMIT_ERROR",
                "details": {"retry_after": 60}
            }
        }


class HealthResponse(BaseModel):
    """Health check response model"""
    status: Literal["healthy", "degraded", "unhealthy"] = Field(..., description="Service status")
    services: dict = Field(..., description="Individual service health")
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "services": {
                    "qdrant": "healthy",
                    "neon": "healthy",
                    "gemini": "healthy"
                },
                "timestamp": "2025-12-05T15:45:33Z"
            }
        }
