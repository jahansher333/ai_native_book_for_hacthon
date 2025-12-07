"""
Query API endpoint for RAG chatbot
Handles question answering with retrieval-augmented generation
"""
import time
import uuid
from fastapi import APIRouter, HTTPException
from typing import Optional

from ..models.queries import QueryRequest, QueryResponse, Citation
from ..services.agent import run_rag_agent
from ..services.session_manager import session_manager

router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """
    Answer a question using RAG (Retrieval-Augmented Generation)

    This endpoint:
    1. Retrieves relevant textbook content based on the question
    2. Uses OpenAI Agents SDK with Gemini to generate an accurate answer
    3. Returns the answer with source citations
    4. Logs the query and response to the database

    Args:
        request: QueryRequest with question, optional selected_text, and session_id

    Returns:
        QueryResponse with answer, citations, and metadata
    """
    start_time = time.time()

    try:
        # Validate session or create new one
        session_id = request.session_id
        if not session_id:
            session_id = await session_manager.create_session()
        else:
            # Check if session exists
            existing_session = await session_manager.get_session(session_id)
            if not existing_session:
                session_id = await session_manager.create_session()

        # Run RAG agent
        result = await run_rag_agent(
            question=request.question,
            selected_text=request.selected_text,
            session_id=session_id
        )

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        # Create response
        response = QueryResponse(
            answer=result["answer"],
            citations=[Citation(**c) for c in result["citations"]],
            latency_ms=latency_ms,
            query_id=str(uuid.uuid4())
        )

        # Log query to database
        try:
            await session_manager.add_query_log(
                session_id=session_id,
                query_text=request.question,
                response_text=result["answer"],
                citations=result["citations"],
                latency_ms=latency_ms,
                query_mode=request.mode,
                selected_text=request.selected_text
            )

            # Update session activity
            await session_manager.update_session_activity(session_id)
        except Exception as e:
            print(f"⚠️  Failed to log query: {e}")

        return response

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to process query: {str(e)}"
        )


@router.get("/history/{session_id}")
async def get_history(session_id: str, limit: int = 20):
    """
    Get query history for a session

    Args:
        session_id: Session UUID
        limit: Maximum number of messages to return (default 20)

    Returns:
        List of previous queries and responses
    """
    try:
        history = await session_manager.get_query_history(session_id, limit=limit)

        return {
            "session_id": session_id,
            "messages": history,
            "total_messages": len(history),
            "has_more": len(history) >= limit
        }

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to retrieve history: {str(e)}"
        )
