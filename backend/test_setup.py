"""
Quick setup test script
Verifies all API connections before starting the server
"""
import asyncio
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.config import settings
from src.services.embeddings import embedding_service
from src.services.vector_store import vector_store_service
from src.services.session_manager import session_manager


async def test_gemini():
    """Test Gemini API connection"""
    print("\n🧪 Testing Gemini API...")
    try:
        embedding = await embedding_service.embed_text("Hello, world!")
        print(f"✅ Gemini API working! Embedding dimension: {len(embedding)}")
        return True
    except Exception as e:
        print(f"❌ Gemini API failed: {e}")
        return False


def test_qdrant():
    """Test Qdrant connection"""
    print("\n🧪 Testing Qdrant Cloud...")
    try:
        is_healthy = vector_store_service.check_health()
        if is_healthy:
            print("✅ Qdrant Cloud connected!")
            # Create collection
            vector_store_service.create_collection()
            return True
        else:
            print("❌ Qdrant Cloud not accessible")
            return False
    except Exception as e:
        print(f"❌ Qdrant failed: {e}")
        return False


def test_neon():
    """Test Neon Postgres connection"""
    print("\n🧪 Testing Neon Postgres...")
    try:
        session_manager.create_tables()
        is_healthy = session_manager.check_health()
        if is_healthy:
            print("✅ Neon Postgres connected!")
            return True
        else:
            print("❌ Neon Postgres not accessible")
            return False
    except Exception as e:
        print(f"❌ Neon Postgres failed: {e}")
        return False


async def main():
    """Run all tests"""
    print("=" * 60)
    print("🚀 RAG Chatbot Setup Test")
    print("=" * 60)

    print(f"\n📋 Configuration:")
    print(f"   Model: {settings.model_name}")
    print(f"   Qdrant URL: {settings.qdrant_url}")
    print(f"   Qdrant Collection: {settings.qdrant_collection_name}")
    print(f"   Database: {settings.neon_database_url.split('@')[1].split('/')[0] if '@' in settings.neon_database_url else 'configured'}")

    # Run tests
    results = []
    results.append(await test_gemini())
    results.append(test_qdrant())
    results.append(test_neon())

    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Results:")
    print("=" * 60)
    print(f"Gemini API:      {'✅ PASS' if results[0] else '❌ FAIL'}")
    print(f"Qdrant Cloud:    {'✅ PASS' if results[1] else '❌ FAIL'}")
    print(f"Neon Postgres:   {'✅ PASS' if results[2] else '❌ FAIL'}")
    print("=" * 60)

    if all(results):
        print("\n🎉 All tests passed! Ready to start the server.")
        print("\nNext steps:")
        print("1. Start the server:")
        print("   uvicorn src.main:app --reload --host 0.0.0.0 --port 8000")
        print("\n2. Ingest content:")
        print("   curl -X POST http://localhost:8000/api/ingest")
        print("\n3. Test query:")
        print("   curl -X POST http://localhost:8000/api/query \\")
        print("     -H 'Content-Type: application/json' \\")
        print("     -d '{\"question\": \"What is ROS 2?\"}'")
    else:
        print("\n⚠️  Some tests failed. Please check your configuration.")
        print("Review the .env file and verify your API keys.")

    return all(results)


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
