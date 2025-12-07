"""
Content chunk data models for RAG chatbot
Represents embedded textbook content segments stored in Qdrant
"""
from typing import Optional, List
from pydantic import BaseModel, Field
from datetime import datetime


class ContentChunk(BaseModel):
    """
    Represents a semantically meaningful segment of textbook content
    Matches Qdrant payload schema from data-model.md
    """
    id: str = Field(..., description="UUID or hash(source_file + chunk_index)")
    text: str = Field(..., min_length=10, max_length=1200, description="Original chunk text")
    module: str = Field(..., description="Module name (e.g., 'ros2', 'gazebo-unity')")
    section: str = Field(..., max_length=100, description="Section identifier")
    page: int = Field(..., ge=1, description="Logical page number for citation")
    source_file: str = Field(..., description="Relative path from /docs/")
    chunk_index: int = Field(..., ge=0, description="Position in document")
    parent_heading: str = Field(..., max_length=200, description="Markdown heading context")
    created_at: str = Field(default_factory=lambda: datetime.utcnow().isoformat(), description="Ingestion timestamp")
    content_hash: str = Field(..., min_length=64, max_length=64, description="SHA-256 hash for deduplication")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "a3f8c2d1-4e5f-6789-abcd-ef1234567890",
                "text": "ROS 2 uses a Data Distribution Service (DDS) middleware...",
                "module": "ros2",
                "section": "pub-sub-basics",
                "page": 12,
                "source_file": "module-1-ros2/pub-sub.md",
                "chunk_index": 3,
                "parent_heading": "## Understanding Publishers and Subscribers",
                "created_at": "2025-12-05T10:30:00Z",
                "content_hash": "d4f6a2c8e1b3..."
            }
        }


class ChunkWithVector(ContentChunk):
    """Content chunk with embedding vector (used during ingestion)"""
    vector: List[float] = Field(..., description="Embedding vector from Gemini")

    class Config:
        json_schema_extra = {
            "example": {
                **ContentChunk.Config.json_schema_extra["example"],
                "vector": [0.021, -0.143, 0.087]  # Truncated for brevity
            }
        }
