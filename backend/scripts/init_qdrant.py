"""Initialize Qdrant collection for textbook embeddings"""
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.vector_store import VectorStoreService
from dotenv import load_dotenv

load_dotenv()


def main():
    print("Initializing Qdrant collection...")

    try:
        vector_store = VectorStoreService()
        vector_store.create_collection()
        print("\n✅ Qdrant collection 'textbook_embeddings' created successfully!")
        print("   - Vector dimension: 1536")
        print("   - Distance metric: COSINE")
        print("   - HNSW config: m=32, ef_construct=200, ef=128")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
