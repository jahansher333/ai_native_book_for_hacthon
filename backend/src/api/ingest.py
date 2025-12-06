"""
Content ingestion API endpoint
Admin endpoint to ingest textbook content into Qdrant
"""
import os
import hashlib
import uuid
from pathlib import Path
from fastapi import APIRouter, HTTPException
from langchain_text_splitters import RecursiveCharacterTextSplitter
from typing import List

from ..config import settings
from ..models.chunks import ContentChunk
from ..services.embeddings import embedding_service
from ..services.vector_store import vector_store_service

router = APIRouter()


def extract_metadata_from_path(file_path: Path) -> dict:
    """Extract module, section info from file path"""
    parts = file_path.parts

    # Determine module from path
    module = "intro"
    if "module-1" in str(file_path) or "ros2" in str(file_path):
        module = "ros2"
    elif "module-2" in str(file_path) or "gazebo" in str(file_path):
        module = "gazebo-unity"
    elif "module-3" in str(file_path) or "isaac" in str(file_path):
        module = "isaac-sim"
    elif "module-4" in str(file_path) or "vla" in str(file_path):
        module = "vla-capstone"
    elif "hardware" in str(file_path):
        module = "hardware"

    # Section from filename
    section = file_path.stem.replace("_", "-").replace(" ", "-")

    return {
        "module": module,
        "section": section,
        "source_file": str(file_path.relative_to(file_path.parent.parent))
    }


@router.post("/ingest")
async def ingest_content(docs_dir: str = "../frontend/docs"):
    """
    Ingest textbook content from Markdown files

    This endpoint:
    1. Loads all .md/.mdx files from the docs directory
    2. Chunks the content (1000 chars, 200 overlap)
    3. Generates embeddings using Gemini
    4. Uploads to Qdrant vector store

    Args:
        docs_dir: Path to documentation directory (relative to backend)

    Returns:
        Ingestion statistics
    """
    try:
        # Resolve docs directory
        current_dir = Path(__file__).parent.parent.parent
        docs_path = (current_dir / docs_dir).resolve()

        if not docs_path.exists():
            raise HTTPException(
                status_code=404,
                detail=f"Documentation directory not found: {docs_path}"
            )

        print(f"📚 Loading documents from: {docs_path}")

        # Initialize text splitter
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap,
            separators=["\n## ", "\n### ", "\n\n", "\n", " ", ""]
        )

        # Collect all markdown files
        md_files = list(docs_path.rglob("*.md")) + list(docs_path.rglob("*.mdx"))
        print(f"📄 Found {len(md_files)} markdown files")

        total_chunks = 0
        all_chunks = []

        # Process each file
        for file_path in md_files:
            try:
                # Read file content
                content = file_path.read_text(encoding='utf-8')

                # Extract metadata
                metadata = extract_metadata_from_path(file_path)

                # Split into chunks
                chunks = text_splitter.split_text(content)

                # Create ContentChunk objects
                for idx, chunk_text in enumerate(chunks):
                    content_hash = hashlib.sha256(chunk_text.encode()).hexdigest()
                    chunk_id = str(uuid.uuid4())

                    chunk = ContentChunk(
                        id=chunk_id,
                        text=chunk_text,
                        module=metadata["module"],
                        section=metadata["section"],
                        page=idx + 1,  # Simplified page numbering
                        source_file=metadata["source_file"],
                        chunk_index=idx,
                        parent_heading="",  # Simplified
                        content_hash=content_hash
                    )
                    all_chunks.append(chunk)

                total_chunks += len(chunks)
                print(f"  ✓ {file_path.name}: {len(chunks)} chunks")

            except Exception as e:
                print(f"  ⚠️  Error processing {file_path.name}: {e}")
                continue

        print(f"\n🔢 Total chunks created: {total_chunks}")
        print(f"🧠 Generating embeddings...")

        # Generate embeddings in batches
        batch_size = 50
        ingested_count = 0

        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i + batch_size]

            # Generate embeddings
            embeddings = await embedding_service.embed_batch(
                [chunk.text for chunk in batch]
            )

            # Upload to Qdrant
            await vector_store_service.upsert_batch(batch, embeddings)

            ingested_count += len(batch)
            print(f"  Uploaded batch {i//batch_size + 1}: {ingested_count}/{total_chunks} chunks")

        print(f"\n✅ Ingestion complete! {ingested_count} chunks uploaded to Qdrant")

        return {
            "status": "success",
            "files_processed": len(md_files),
            "chunks_created": total_chunks,
            "chunks_ingested": ingested_count
        }

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Ingestion failed: {str(e)}"
        )
