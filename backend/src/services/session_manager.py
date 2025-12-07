"""
Session manager for Neon Postgres
Handles chat session and query log operations
"""
from typing import Optional, List, Dict
from datetime import datetime, timedelta
import uuid
import asyncpg
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from ..config import settings
from ..models.sessions import Base, ChatSession, QueryLog


class SessionManager:
    """Manages chat sessions and query logs in Neon Postgres"""

    def __init__(self):
        self.database_url = settings.neon_database_url
        self.engine = None
        self.Session = None

        # Only initialize database if URL is provided
        if self.database_url and self.database_url.strip():
            try:
                self.engine = create_engine(self.database_url)
                self.Session = sessionmaker(bind=self.engine)
            except Exception as e:
                print(f"WARNING: Could not initialize database: {e}")
                self.engine = None
                self.Session = None

    def create_tables(self):
        """Create all tables if they don't exist"""
        Base.metadata.create_all(self.engine)
        print("✅ Database tables created/verified")

    async def create_session(self, user_id: Optional[str] = None, device_type: str = "unknown") -> str:
        """
        Create a new chat session

        Args:
            user_id: Optional user identifier
            device_type: Device type (desktop, mobile, tablet)

        Returns:
            session_id as string
        """
        session = self.Session()
        try:
            new_session = ChatSession(
                user_id=user_id,
                device_type=device_type
            )
            session.add(new_session)
            session.commit()
            return str(new_session.session_id)
        finally:
            session.close()

    async def get_session(self, session_id: str) -> Optional[Dict]:
        """Get session by ID"""
        session = self.Session()
        try:
            result = session.query(ChatSession).filter(
                ChatSession.session_id == uuid.UUID(session_id)
            ).first()

            if result:
                return {
                    "session_id": str(result.session_id),
                    "user_id": result.user_id,
                    "device_type": result.device_type,
                    "created_at": result.created_at.isoformat(),
                    "last_active": result.last_active.isoformat(),
                    "query_count": result.query_count
                }
            return None
        finally:
            session.close()

    async def update_session_activity(self, session_id: str):
        """Update last_active timestamp and increment query_count"""
        session = self.Session()
        try:
            result = session.query(ChatSession).filter(
                ChatSession.session_id == uuid.UUID(session_id)
            ).first()

            if result:
                result.last_active = datetime.utcnow()
                result.query_count += 1
                result.expires_at = datetime.utcnow() + timedelta(days=30)
                session.commit()
        finally:
            session.close()

    async def add_query_log(
        self,
        session_id: str,
        query_text: str,
        response_text: str,
        citations: List[Dict],
        latency_ms: int,
        query_mode: str = "general",
        selected_text: Optional[str] = None
    ) -> str:
        """
        Add a query log entry

        Returns:
            query_id as string
        """
        session = self.Session()
        try:
            query_log = QueryLog(
                session_id=uuid.UUID(session_id),
                query_text=query_text,
                query_mode=query_mode,
                selected_text=selected_text,
                response_text=response_text,
                citations=citations,
                retrieved_chunk_ids=[c.get("chunk_id", "") for c in citations],
                latency_ms=latency_ms
            )
            session.add(query_log)
            session.commit()
            return str(query_log.query_id)
        finally:
            session.close()

    async def get_query_history(self, session_id: str, limit: int = 20) -> List[Dict]:
        """Get query history for a session"""
        session = self.Session()
        try:
            results = session.query(QueryLog).filter(
                QueryLog.session_id == uuid.UUID(session_id)
            ).order_by(QueryLog.timestamp.desc()).limit(limit).all()

            history = []
            for result in results:
                history.append({
                    "query_id": str(result.query_id),
                    "query_text": result.query_text,
                    "response_text": result.response_text,
                    "citations": result.citations,
                    "timestamp": result.timestamp.isoformat(),
                    "query_mode": result.query_mode
                })

            return list(reversed(history))  # Return chronological order
        finally:
            session.close()

    def check_health(self) -> bool:
        """Check if database is accessible"""
        try:
            session = self.Session()
            session.execute("SELECT 1")
            session.close()
            return True
        except:
            return False


# Global service instance
session_manager = SessionManager()
