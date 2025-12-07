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
from .api import query, health, ingest, auth, personalize, translate


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    # Startup
    print("Starting RAG Chatbot Backend with LiteLLM/Groq...")
    print(f"Environment: {settings.environment}")
    print(f"Model: {settings.model_name}")

    # Initialize database tables
    try:
        session_manager.create_tables()
    except Exception as e:
        print(f"WARNING: Database initialization warning: {e}")

    # Initialize auth tables (users, sessions)
    try:
        from .models.user import Base
        from .services.auth_service import engine
        Base.metadata.create_all(engine)
        print("Auth tables initialized")
    except Exception as e:
        print(f"WARNING: Auth tables initialization: {e}")

    # Create Qdrant collection if needed
    try:
        vector_store_service.create_collection()
    except Exception as e:
        print(f"WARNING: Qdrant collection warning: {e}")

    yield

    # Shutdown
    print("Shutting down RAG Chatbot Backend...")


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
app.include_router(auth.router, tags=["authentication"])  # Auth already has /api/auth prefix
app.include_router(personalize.router, tags=["personalization"])  # Personalize endpoint
app.include_router(translate.router, tags=["translation"])  # Urdu translation endpoint


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running"
    }
