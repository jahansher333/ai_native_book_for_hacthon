"""
Vector store service using Qdrant Cloud
Manages vector search and upsert operations
"""
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
import uuid

from ..config import settings
from ..models.chunks import ContentChunk


class VectorStoreService:
    """Service for Qdrant vector operations"""

    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name

    def create_collection(self, vector_size: int = 768):
        """
        Create Qdrant collection if it doesn't exist

        Args:
            vector_size: Dimension of embedding vectors (default 768 for Gemini)
        """
        try:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Created collection: {self.collection_name}")
        except Exception as e:
            if "already exists" in str(e).lower():
                print(f"✓ Collection already exists: {self.collection_name}")
            else:
                raise e

    async def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        module_filter: Optional[str] = None,
        score_threshold: float = 0.3
    ) -> List[Dict]:
        """
        Search for similar chunks in Qdrant

        Args:
            query_embedding: Query vector
            limit: Number of results to return
            module_filter: Optional module filter (e.g., "ros2")
            score_threshold: Minimum similarity score

        Returns:
            List of search results with payload and score
        """
        try:
            # Build filter if module specified
            search_filter = None
            if module_filter and module_filter != "all":
                search_filter = Filter(
                    must=[
                        FieldCondition(
                            key="module",
                            match=MatchValue(value=module_filter)
                        )
                    ]
                )

            # Search Qdrant
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=search_filter,
                limit=limit,
                score_threshold=score_threshold
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                })

            return formatted_results

        except Exception as e:
            raise Exception(f"Qdrant search failed: {str(e)}")

    async def upsert_chunk(self, chunk: ContentChunk, embedding: List[float]):
        """
        Insert or update a single chunk in Qdrant

        Args:
            chunk: Content chunk with metadata
            embedding: Embedding vector
        """
        try:
            point = PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "text": chunk.text,
                    "module": chunk.module,
                    "section": chunk.section,
                    "page": chunk.page,
                    "source_file": chunk.source_file,
                    "chunk_index": chunk.chunk_index,
                    "parent_heading": chunk.parent_heading,
                    "created_at": chunk.created_at,
                    "content_hash": chunk.content_hash
                }
            )

            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

        except Exception as e:
            raise Exception(f"Qdrant upsert failed: {str(e)}")

    async def upsert_batch(self, chunks: List[ContentChunk], embeddings: List[List[float]]):
        """
        Batch insert or update chunks in Qdrant

        Args:
            chunks: List of content chunks
            embeddings: List of embedding vectors
        """
        try:
            points = []
            for chunk, embedding in zip(chunks, embeddings):
                point = PointStruct(
                    id=chunk.id,
                    vector=embedding,
                    payload={
                        "text": chunk.text,
                        "module": chunk.module,
                        "section": chunk.section,
                        "page": chunk.page,
                        "source_file": chunk.source_file,
                        "chunk_index": chunk.chunk_index,
                        "parent_heading": chunk.parent_heading,
                        "created_at": chunk.created_at,
                        "content_hash": chunk.content_hash
                    }
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

        except Exception as e:
            raise Exception(f"Qdrant batch upsert failed: {str(e)}")

    def check_health(self) -> bool:
        """Check if Qdrant is accessible"""
        try:
            collections = self.client.get_collections()
            return True
        except:
            return False


# Global service instance
vector_store_service = VectorStoreService()
