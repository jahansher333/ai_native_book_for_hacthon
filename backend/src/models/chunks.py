from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class TextbookChunk(BaseModel):
    """Model for textbook content chunks stored in Qdrant"""
    id: int
    text: str
    module: str
    chapter: str
    section: Optional[str] = None
    source_url: str
    chunk_index: int
    created_at: str

    class Config:
        json_schema_extra = {
            "example": {
                "id": 12345,
                "text": "A ROS 2 node is a process that performs computation...",
                "module": "01-ros2",
                "chapter": "nodes.md",
                "section": "What is a Node?",
                "source_url": "/docs/ros2/nodes",
                "chunk_index": 0,
                "created_at": "2025-12-04T10:30:00Z"
            }
        }
