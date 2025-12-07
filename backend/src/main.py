from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from dotenv import load_dotenv
from pathlib import Path

# Load environment variables
load_dotenv()

# Import services
from .services.embeddings import EmbeddingService
from .services.vector_store import VectorStoreService
from .services.agent import RAGAgent
from .models.queries import QueryRequest, QueryResponse, Source

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="Retrieval-Augmented Generation chatbot for Physical AI textbook"
)

# CORS middleware for GitHub Pages and local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://jahansher333.github.io",
        "http://localhost:3000",
        "http://127.0.0.1:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
embedding_service = EmbeddingService()
vector_store_service = VectorStoreService()
rag_agent = RAGAgent(vector_store_service, embedding_service)


@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "ok",
        "message": "RAG Chatbot API",
        "version": "1.0.0"
    }


@app.post("/query", response_model=QueryResponse)
async def query_textbook(request: QueryRequest):
    """
    Query the textbook using RAG

    Args:
        request: Query request with question, optional selected_text, session_id

    Returns:
        QueryResponse with answer, sources, confidence
    """
    try:
        result = rag_agent.run_rag_agent(
            question=request.question,
            selected_text=request.selected_text or "",
            module_filter=request.module_filter
        )

        return QueryResponse(
            answer=result["answer"],
            sources=[Source(**s) for s in result["sources"]],
            confidence=result["confidence"],
            processing_time_ms=result["processing_time_ms"]
        )

    except Exception as e:
        print(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Query processing failed: {str(e)}")


@app.post("/ingest")
async def ingest_textbook(admin_key: str):
    """
    Ingest textbook content into Qdrant

    Args:
        admin_key: Admin API key for authentication

    Returns:
        Ingestion status and chunk count
    """
    # Check admin key
    if admin_key != os.getenv("ADMIN_API_KEY"):
        raise HTTPException(status_code=401, detail="Unauthorized")

    try:
        from .utils.chunking import MDXTextSplitter
        import time

        docs_dir = Path("../frontend/docs")
        if not docs_dir.exists():
            raise HTTPException(status_code=404, detail="Docs directory not found")

        splitter = MDXTextSplitter(chunk_size=1024, chunk_overlap=256)

        all_chunks = []
        chunk_id = 0

        # Process all .md and .mdx files
        for mdx_file in docs_dir.rglob("*.md*"):
            print(f"Processing: {mdx_file.name}")

            content = mdx_file.read_text(encoding='utf-8')

            # Extract module from path
            rel_path = mdx_file.relative_to(docs_dir)
            module = str(rel_path.parts[0]) if len(rel_path.parts) > 1 else "intro"

            # Split into chunks
            chunks = splitter.split_mdx_file(content, mdx_file.stem)

            for chunk in chunks:
                if not chunk["text"].strip():
                    continue

                # Generate embedding
                embedding = embedding_service.embed_text(chunk["text"])

                # Prepare point
                all_chunks.append({
                    "id": chunk_id,
                    "vector": embedding,
                    "payload": {
                        "text": chunk["text"],
                        "module": module,
                        "chapter": chunk["chapter"],
                        "source_url": f"/docs/{rel_path.with_suffix('').as_posix()}",
                        "chunk_index": chunk["chunk_index"],
                        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ")
                    }
                })

                chunk_id += 1

                # Batch upsert every 50 chunks
                if len(all_chunks) >= 50:
                    vector_store_service.upsert_batch(all_chunks)
                    print(f"  Uploaded {len(all_chunks)} chunks...")
                    all_chunks = []

        # Upload remaining chunks
        if all_chunks:
            vector_store_service.upsert_batch(all_chunks)

        return {
            "status": "success",
            "chunks_ingested": chunk_id,
            "message": f"Ingested {chunk_id} chunks successfully"
        }

    except Exception as e:
        print(f"Error during ingestion: {e}")
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
