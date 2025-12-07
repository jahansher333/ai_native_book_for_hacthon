# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot using OpenAI Agents SDK with Gemini API.

## Features

- ✅ **RAG (Retrieval-Augmented Generation)** using OpenAI Agents SDK
- ✅ **Gemini 2.0 Flash** via OpenAI-compatible endpoint for embeddings and chat
- ✅ **Qdrant Cloud** for vector storage (free tier)
- ✅ **Neon Postgres** for session management (free tier)
- ✅ **Book-only retrieval** with source citations
- ✅ **Selected-text query** support
- ✅ **Chat history** persistence

## Architecture

```
User Question
    ↓
FastAPI /query Endpoint
    ↓
OpenAI Agents SDK (function_tool: search_textbook)
    ↓
Gemini Embedding → Qdrant Search → Top-5 Chunks
    ↓
Gemini 2.0 Flash (via OpenAI Agent) → Generate Answer
    ↓
Store in Neon Postgres → Return Response with Citations
```

## Setup

### 1. Install Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
# Gemini API (via OpenAI-compatible endpoint)
GEMINI_API_KEY=your_gemini_api_key_here
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
MODEL_NAME=gemini-2.0-flash-exp

# Qdrant Cloud (sign up at https://cloud.qdrant.io)
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_vectors

# Neon Postgres (sign up at https://neon.tech)
NEON_DATABASE_URL=postgresql://user:password@your-neon-endpoint/dbname

# Application settings
ENVIRONMENT=development
DEBUG=True
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# RAG settings
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5
SIMILARITY_THRESHOLD=0.3
```

### 3. Get API Keys

**Gemini API**:
1. Go to https://makersuite.google.com/app/apikey
2. Create API key
3. Copy to `GEMINI_API_KEY`

**Qdrant Cloud**:
1. Sign up at https://cloud.qdrant.io
2. Create a free 1GB cluster
3. Copy cluster URL and API key

**Neon Postgres**:
1. Sign up at https://neon.tech
2. Create a new project
3. Copy connection string

### 4. Initialize Databases

The application will automatically create:
- Qdrant collection `book_vectors` (768-dim vectors, COSINE similarity)
- Neon Postgres tables (`chat_sessions`, `query_logs`)

### 5. Ingest Textbook Content

Before using the chatbot, you need to ingest the textbook content:

```bash
# Start the server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# In another terminal, call the ingest endpoint
curl -X POST "http://localhost:8000/api/ingest" \
  -H "Content-Type: application/json"
```

This will:
- Load all `.md` files from `../docs/`
- Chunk content (1000 chars, 200 overlap)
- Generate embeddings with Gemini
- Upload to Qdrant

Expected output: `450-500 chunks ingested` (takes 2-3 minutes)

## Running the Server

```bash
# Development mode with auto-reload
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Production mode
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

Server will be available at: http://localhost:8000

API Documentation: http://localhost:8000/docs

## API Endpoints

### POST /api/query

Answer a question using RAG.

**Request**:
```json
{
  "question": "What is ROS 2?",
  "mode": "general",
  "selected_text": null,
  "session_id": "optional-uuid"
}
```

**Response**:
```json
{
  "answer": "ROS 2 is the Robot Operating System 2...",
  "citations": [
    {
      "module": "ros2",
      "section": "introduction",
      "page": 8,
      "source_file": "module-1-ros2/intro.md",
      "relevance_score": 0.92
    }
  ],
  "latency_ms": 2341,
  "query_id": "uuid",
  "timestamp": "2025-12-05T15:45:33Z"
}
```

### GET /api/history/{session_id}

Retrieve chat history for a session.

**Response**:
```json
{
  "session_id": "uuid",
  "messages": [
    {
      "query_text": "What is ROS 2?",
      "response_text": "ROS 2 is...",
      "citations": [...],
      "timestamp": "2025-12-05T15:45:33Z"
    }
  ],
  "total_messages": 5,
  "has_more": false
}
```

### GET /api/health

Check service health.

**Response**:
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "healthy",
    "neon": "healthy",
    "gemini": "healthy"
  },
  "timestamp": "2025-12-05T15:45:33Z"
}
```

### POST /api/ingest

Ingest textbook content (admin only).

**Response**:
```json
{
  "status": "success",
  "files_processed": 29,
  "chunks_created": 482,
  "chunks_ingested": 482
}
```

## Testing

### Manual Testing

```bash
# Test health check
curl http://localhost:8000/api/health

# Test query (general mode)
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "mode": "general"
  }'

# Test query (selected text mode)
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this hardware",
    "mode": "selected",
    "selected_text": "Jetson Orin Nano: $249, 40 TOPS INT8"
  }'
```

### Expected Behavior

**✅ Correct Responses**:
- Answers only use textbook content
- All answers include citations: `[Source: Module - Section, Page X]`
- Response time < 3 seconds (p95)
- Refuses non-textbook questions politely

**❌ Incorrect Responses**:
- If you see answers about weather, news, or topics outside the textbook, the agent instructions need adjustment
- If citations are missing, check the `extract_citations()` function in `agent.py`

## Troubleshooting

### Error: "Qdrant connection failed"
- Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Verify cluster is running at https://cloud.qdrant.io

### Error: "Neon Postgres connection failed"
- Check `NEON_DATABASE_URL` format: `postgresql://user:password@host/db`
- Verify project is active at https://neon.tech

### Error: "Gemini API rate limit"
- Gemini free tier: 60 requests/minute for embeddings
- Implement request queuing or upgrade to paid tier

### Slow responses (>5 seconds)
- Check network latency to Qdrant/Neon
- Optimize Qdrant HNSW index settings
- Use batch embedding for ingestion

## Deployment

### Deploy to Render (Free Tier)

1. Create `render.yaml`:
```yaml
services:
  - type: web
    name: rag-chatbot-backend
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: GEMINI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: NEON_DATABASE_URL
        sync: false
```

2. Push to GitHub and connect to Render
3. Set environment variables in Render dashboard
4. Deploy!

Production URL: `https://your-app.onrender.com`

## Project Structure

```
backend/
├── src/
│   ├── __init__.py
│   ├── config.py              # Environment configuration
│   ├── main.py                # FastAPI app entry point
│   ├── models/                # Pydantic models
│   │   ├── chunks.py          # Content chunk schema
│   │   ├── queries.py         # Query/response models
│   │   └── sessions.py        # Session/log models
│   ├── services/              # Business logic
│   │   ├── embeddings.py      # Gemini embedding service
│   │   ├── vector_store.py    # Qdrant operations
│   │   ├── session_manager.py # Neon Postgres operations
│   │   └── agent.py           # OpenAI Agents SDK RAG agent
│   └── api/                   # API endpoints
│       ├── query.py           # POST /query, GET /history
│       ├── health.py          # GET /health
│       └── ingest.py          # POST /ingest
├── requirements.txt
├── .env.example
└── README.md
```

## Next Steps

1. ✅ Backend is complete and running
2. 🔄 Create frontend RagChat component (React + ChatKit)
3. 🔄 Integrate widget into Docusaurus
4. 🔄 Test end-to-end flow
5. 🔄 Deploy to production

## Support

For issues or questions:
- Check logs: `uvicorn` output shows request/response details
- API docs: http://localhost:8000/docs
- GitHub issues: [Create issue]

## License

MIT License - See LICENSE file for details
