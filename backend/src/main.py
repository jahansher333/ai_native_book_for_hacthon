"""
FastAPI main application for RAG Chatbot
Entry point for the backend API
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from .config import settings
from .services.vector_store import vector_store_service
from .services.session_manager import session_manager
from .api import query, health, ingest


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    # Startup
    print("🚀 Starting RAG Chatbot Backend...")
    print(f"Environment: {settings.environment}")
    print(f"Model: {settings.model_name}")

    # Initialize database tables
    try:
        session_manager.create_tables()
    except Exception as e:
        print(f"⚠️  Database initialization warning: {e}")

    # Create Qdrant collection if needed
    try:
        vector_store_service.create_collection()
    except Exception as e:
        print(f"⚠️  Qdrant collection warning: {e}")

    yield

    # Shutdown
    print("👋 Shutting down RAG Chatbot Backend...")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI textbook",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.get_cors_origins(),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(query.router, prefix="/api", tags=["query"])
app.include_router(health.router, prefix="/api", tags=["health"])
app.include_router(ingest.router, prefix="/api", tags=["ingest"])


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running"
    }
