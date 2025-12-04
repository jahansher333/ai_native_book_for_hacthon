from openai import OpenAI
from typing import List
import os


class EmbeddingService:
    """Service for generating text embeddings using OpenAI"""

    def __init__(self, api_key: str = None):
        self.client = OpenAI(api_key=api_key or os.getenv("OPENAI_API_KEY"))
        self.model = "text-embedding-3-small"

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for given text

        Args:
            text: Text to embed

        Returns:
            List of floats representing the embedding vector (1536 dimensions)
        """
        try:
            response = self.client.embeddings.create(
                input=text,
                model=self.model
            )
            return response.data[0].embedding
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise

    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embeddings.create(
                input=texts,
                model=self.model
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            print(f"Error generating batch embeddings: {e}")
            raise
