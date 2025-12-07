from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
from typing import List, Dict, Any
import os


class VectorStoreService:
    """Service for interacting with Qdrant vector database"""

    def __init__(self, url: str = None, api_key: str = None):
        self.client = QdrantClient(
            url=url or os.getenv("QDRANT_URL"),
            api_key=api_key or os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = "textbook_embeddings"

    def create_collection(self):
        """Create Qdrant collection with proper schema"""
        try:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1536,  # OpenAI text-embedding-3-small dimension
                    distance=Distance.COSINE
                ),
                hnsw_config={
                    "m": 32,
                    "ef_construct": 200,
                    "ef": 128
                }
            )
            print(f"✅ Collection '{self.collection_name}' created successfully")
        except Exception as e:
            print(f"Collection may already exist or error occurred: {e}")

    def upsert_chunk(self, chunk_id: int, text: str, embedding: List[float], metadata: Dict[str, Any]):
        """
        Insert or update a chunk in Qdrant

        Args:
            chunk_id: Unique ID for the chunk
            text: Chunk text content
            embedding: Embedding vector
            metadata: Additional metadata (module, chapter, source_url, etc.)
        """
        point = PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                "text": text,
                **metadata
            }
        )

        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

    def upsert_batch(self, points: List[Dict[str, Any]]):
        """Batch upsert multiple chunks"""
        qdrant_points = [
            PointStruct(
                id=point["id"],
                vector=point["vector"],
                payload=point["payload"]
            )
            for point in points
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=qdrant_points
        )

    def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        module_filter: str = "all",
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks

        Args:
            query_embedding: Query embedding vector
            limit: Maximum number of results
            module_filter: Filter by module (e.g., "01-ros2" or "all")
            score_threshold: Minimum similarity score

        Returns:
            List of matching chunks with scores
        """
        search_filter = None
        if module_filter != "all":
            search_filter = {
                "must": [
                    {"key": "module", "match": {"value": module_filter}}
                ]
            }

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=search_filter
        )

        return [
            {
                "text": hit.payload["text"],
                "score": hit.score,
                "module": hit.payload.get("module", ""),
                "chapter": hit.payload.get("chapter", ""),
                "source_url": hit.payload.get("source_url", ""),
                "chunk_id": hit.id
            }
            for hit in results
        ]
