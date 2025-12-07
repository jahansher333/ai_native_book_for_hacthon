"""
Embedding service using Gemini API via OpenAI-compatible endpoint
Generates embeddings for text chunks and queries
"""
import hashlib
from typing import List
from openai import AsyncOpenAI
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

from ..config import settings


class EmbeddingService:
    """Service for generating embeddings using Gemini via OpenAI endpoint"""

    def __init__(self):
        # Initialize AsyncOpenAI client with Gemini endpoint
        self.client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url=settings.base_url
        )
        self.model_name = "text-embedding-004"  # Gemini embedding model
        self.dimension = 768  # Gemini embedding dimension

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10),
        retry=retry_if_exception_type(Exception)
    )
    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Input text to embed (max 2048 tokens)

        Returns:
            Embedding vector

        Raises:
            Exception: If API call fails after retries
        """
        try:
            response = await self.client.embeddings.create(
                model=self.model_name,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            raise Exception(f"Gemini embedding failed: {str(e)}")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10)
    )
    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batches

        Args:
            texts: List of texts to embed
            batch_size: Number of texts per API call (default 100)

        Returns:
            List of embedding vectors
        """
        embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                # Process batch
                for text in batch:
                    response = await self.client.embeddings.create(
                        model=self.model_name,
                        input=text
                    )
                    embeddings.append(response.data[0].embedding)
            except Exception as e:
                raise Exception(f"Batch embedding failed at index {i}: {str(e)}")

        return embeddings

    async def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a search query

        Args:
            query: Search query text

        Returns:
            Query embedding vector
        """
        return await self.embed_text(query)

    def compute_embedding_hash(self, embedding: List[float]) -> str:
        """
        Compute SHA-256 hash of embedding for caching/deduplication

        Args:
            embedding: Vector embedding

        Returns:
            64-character hex hash
        """
        embedding_bytes = str(embedding).encode('utf-8')
        return hashlib.sha256(embedding_bytes).hexdigest()


# Global service instance
embedding_service = EmbeddingService()
