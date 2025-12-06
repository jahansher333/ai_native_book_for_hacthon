"""
Configuration management for RAG Chatbot backend
Loads environment variables using Gemini API via OpenAI-compatible endpoint
"""
import os
from typing import List
from pydantic_settings import BaseSettings
from pydantic import Field
from dotenv import load_dotenv

# Load environment variables
load_dotenv(override=True)


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # Gemini API via OpenAI-compatible endpoint
    gemini_api_key: str = Field(..., env="GEMINI_API_KEY")
    base_url: str = Field(
        default="https://generativelanguage.googleapis.com/v1beta/openai/",
        env="BASE_URL"
    )
    model_name: str = Field(default="gemini-2.0-flash-exp", env="MODEL_NAME")

    # Qdrant Cloud
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="book_vectors", env="QDRANT_COLLECTION_NAME")

    # Neon Postgres
    neon_database_url: str = Field(..., env="NEON_DATABASE_URL")

    # Application settings
    environment: str = Field(default="development", env="ENVIRONMENT")
    debug: bool = Field(default=False, env="DEBUG")
    log_level: str = Field(default="INFO", env="LOG_LEVEL")

    # CORS settings
    cors_origins: str = Field(
        default="http://localhost:3000,http://localhost:3001",
        env="CORS_ORIGINS"
    )

    # Chunking settings
    chunk_size: int = Field(default=1000, env="CHUNK_SIZE")
    chunk_overlap: int = Field(default=200, env="CHUNK_OVERLAP")

    # RAG settings
    top_k_results: int = Field(default=5, env="TOP_K_RESULTS")
    similarity_threshold: float = Field(default=0.3, env="SIMILARITY_THRESHOLD")

    # JWT Authentication Settings
    jwt_secret: str = Field(..., env="JWT_SECRET")
    jwt_algorithm: str = Field(default="HS256", env="JWT_ALGORITHM")
    jwt_expiration_days: int = Field(default=7, env="JWT_EXPIRATION_DAYS")

    class Config:
        env_file = ".env"
        case_sensitive = False

    def get_cors_origins(self) -> List[str]:
        """Parse CORS origins from comma-separated string"""
        return [origin.strip() for origin in self.cors_origins.split(",")]


# Global settings instance
settings = Settings()
