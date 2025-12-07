# RAG Chatbot Backend

Backend API for the Physical AI & Humanoid Robotics textbook chatbot using FastAPI, OpenAI, Qdrant, and RAG architecture.

## Quick Start

### 1. Install Dependencies

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate

pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
```

Required environment variables:
- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Qdrant Cloud URL (or use local: `http://localhost:6333`)
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `DATABASE_URL`: Neon Postgres connection string (optional for MVP)
- `ADMIN_API_KEY`: Random string for /ingest endpoint authentication

### 3. Initialize Qdrant Collection

```bash
python scripts/init_qdrant.py
```

### 4. Run the Server

```bash
# Development mode with auto-reload
uvicorn src.main:app --reload

# Or using Python directly
python -m src.main
```

The API will be available at `http://localhost:8000`

API Documentation: `http://localhost:8000/docs`

## API Endpoints

### GET /
Health check endpoint

### POST /query
Query the textbook using RAG

**Request:**
```json
{
  "question": "What is ROS 2?",
  "selected_text": null,
  "session_id": "uuid-here",
  "module_filter": "all"
}
```

**Response:**
```json
{
  "answer": "ROS 2 is... [Source: Module 1]",
  "sources": [...],
  "confidence": 0.92,
  "processing_time_ms": 1850
}
```

### POST /ingest?admin_key=YOUR_KEY
Ingest textbook content (admin only)

## Testing

```bash
# Test health endpoint
curl http://localhost:8000/

# Test query endpoint
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "selected_text": null,
    "session_id": "test-123",
    "module_filter": "all"
  }'
```

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app
│   ├── api/                 # API routes
│   ├── services/            # Business logic
│   │   ├── embeddings.py    # OpenAI embeddings
│   │   ├── vector_store.py  # Qdrant client
│   │   └── agent.py         # RAG agent
│   ├── models/              # Pydantic models
│   └── utils/               # Utilities
├── scripts/                 # Setup scripts
└── tests/                   # Test files
```

## Notes

- The `/ingest` endpoint processes all MDX files from `../frontend/docs/`
- Embeddings are generated using OpenAI `text-embedding-3-small` (1536 dimensions)
- Vector search uses Qdrant with COSINE similarity
- RAG agent uses OpenAI function calling for retrieval
