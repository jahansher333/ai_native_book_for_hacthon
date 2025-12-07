"""
Chat session data models for Neon Postgres
Matches data-model.md schema definitions
"""
from typing import Optional
from datetime import datetime, timedelta
from sqlalchemy import Column, String, Integer, DateTime, JSON, Index, Text
from sqlalchemy.dialects.postgresql import UUID as PGUUID, ARRAY
from sqlalchemy.ext.declarative import declarative_base
import uuid


Base = declarative_base()


class ChatSession(Base):
    """
    User browsing session model
    Stores session metadata and tracks query activity
    """
    __tablename__ = "chat_sessions"

    session_id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String(255), nullable=True)  # NULL for anonymous
    device_type = Column(String(50), nullable=False, default="unknown")  # desktop, mobile, tablet
    browser = Column(String(100), nullable=True)  # e.g., "Chrome 120"
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    last_active = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    query_count = Column(Integer, nullable=False, default=0)
    metadata_json = Column("metadata", JSON, nullable=False, default=dict)  # Flexible: {ip_country, referrer}
    expires_at = Column(DateTime, nullable=False, default=lambda: datetime.utcnow() + timedelta(days=30))

    __table_args__ = (
        Index('idx_session_last_active', 'last_active'),
        Index('idx_session_expires', 'expires_at'),
        Index('idx_session_user', 'user_id'),
    )

    def __repr__(self):
        return f"<ChatSession(session_id={self.session_id}, user_id={self.user_id}, query_count={self.query_count})>"


class QueryLog(Base):
    """
    Query and response log model
    Stores individual queries with performance metrics
    """
    __tablename__ = "query_logs"

    query_id = Column(PGUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PGUUID(as_uuid=True), nullable=False)  # Foreign key to chat_sessions

    # Query fields
    query_text = Column(Text, nullable=False)
    query_mode = Column(String(20), nullable=False, default="general")  # 'general' or 'selected'
    selected_text = Column(Text, nullable=True)
    query_embedding_hash = Column(String(64), nullable=True)  # SHA-256 of embedding

    # Response fields
    response_text = Column(Text, nullable=False)
    citations = Column(JSON, nullable=False, default=list)  # Array of Citation objects
    retrieved_chunk_ids = Column(ARRAY(Text), nullable=False, default=list)  # Array of Qdrant IDs

    # Performance metrics
    latency_ms = Column(Integer, nullable=False)
    embedding_latency_ms = Column(Integer, nullable=True)
    retrieval_latency_ms = Column(Integer, nullable=True)
    generation_latency_ms = Column(Integer, nullable=True)

    # Metadata
    model_version = Column(String(50), nullable=False, default="gemini-2.0-flash-exp")
    num_chunks_retrieved = Column(Integer, nullable=False, default=5)
    timestamp = Column(DateTime, nullable=False, default=datetime.utcnow)

    # Quality tracking (future: user feedback)
    user_rating = Column(Integer, nullable=True)  # 1-5 stars
    user_feedback = Column(Text, nullable=True)

    __table_args__ = (
        Index('idx_query_session', 'session_id', 'timestamp'),
        Index('idx_query_timestamp', 'timestamp'),
        Index('idx_query_mode', 'query_mode'),
    )

    def __repr__(self):
        return f"<QueryLog(query_id={self.query_id}, session_id={self.session_id}, query_text={self.query_text[:50]})>"
