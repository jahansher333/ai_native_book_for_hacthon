# Quickstart Guide: RAG Chatbot Setup

**Feature**: 002-rag-chatbot
**Date**: 2025-12-04
**Estimated Setup Time**: 45-60 minutes

This guide walks through setting up the complete RAG chatbot system from scratch.

---

## Prerequisites

### Required Accounts (All Free Tier)
- [ ] **OpenAI Account** with API key ($5 minimum credit)
- [ ] **Qdrant Cloud Account** (no credit card required)
- [ ] **Neon Postgres Account** (no credit card required)
- [ ] **Render Account** (for backend deployment, optional for local dev)

### Required Software
- [ ] **Python 3.11+** (`python --version`)
- [ ] **Node.js 18+** (`node --version`)
- [ ] **Git** (`git --version`)
- [ ] **Docker** (optional, for local Qdrant testing)

---

## Part 1: Environment Setup (10 min)

### 1.1 Clone Repository

```bash
git clone https://github.com/jahansher333/Ai_Native_Books_Pyhsical_Ai.git
cd Ai_Native_Books_Pyhsical_Ai
git checkout -b 002-rag-chatbot
```

### 1.2 Create Backend Directory Structure

```bash
mkdir -p backend/src/{api,services,models,utils}
mkdir -p backend/tests/{unit,integration,contract}
mkdir -p backend/scripts
```

### 1.3 Install Python Dependencies

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install fastapi[all] uvicorn openai qdrant-client psycopg[binary] langchain python-dotenv pydantic
```

Create `backend/requirements.txt`:

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
openai==1.3.8
qdrant-client==1.6.9
psycopg[binary]==3.1.15
langchain==0.1.0
python-dotenv==1.0.0
pydantic==2.5.2
pytest==7.4.3
pytest-asyncio==0.21.1
```

### 1.4 Create Environment Variables

Create `backend/.env`:

```bash
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://YOUR_CLUSTER.qdrant.io
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://user:pass@ep-cool-morning-123.us-east-1.aws.neon.tech/textbook_db?sslmode=require

# Admin API Key (for /ingest endpoint)
ADMIN_API_KEY=your-secure-random-key

# Environment
ENVIRONMENT=development
```

**Security**: Add `backend/.env` to `.gitignore`!

---

## Part 2: External Services Setup (15 min)

### 2.1 OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Click **"Create new secret key"**
3. Copy key (starts with `sk-`)
4. Add $5 minimum credit at https://platform.openai.com/account/billing

**Cost Estimate**: ~$0.10 for 1000 queries

### 2.2 Qdrant Cloud Setup

1. **Sign up** at https://cloud.qdrant.io
2. **Create cluster**:
   - Name: `textbook-embeddings`
   - Region: `us-east-1` (or nearest)
   - Tier: **Free**
3. **Generate API key**:
   - Go to **API Keys** tab
   - Click **"Create API Key"**
   - Copy key
4. **Note cluster URL**: `https://<cluster-id>.qdrant.io`

### 2.3 Neon Postgres Setup

1. **Sign up** at https://neon.tech
2. **Create project**:
   - Name: `textbook-chatbot`
   - Region: `us-east-1`
   - Postgres version: `16`
3. **Get connection string**:
   - Go to **Dashboard** → **Connection Details**
   - Copy **Connection string** (starts with `postgresql://`)
4. **Enable pgvector** (already included in Neon free tier)

---

## Part 3: Database Initialization (5 min)

### 3.1 Initialize Qdrant Collection

Create `backend/scripts/init_qdrant.py`:

```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection
client.create_collection(
    collection_name="textbook_embeddings",
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small
        distance=Distance.COSINE
    ),
    hnsw_config={
        "m": 32,
        "ef_construct": 200,
        "ef": 128
    }
)

print("✅ Qdrant collection created successfully!")
```

Run:

```bash
python scripts/init_qdrant.py
```

### 3.2 Initialize Neon Postgres Schema

Create `backend/scripts/init_postgres.py`:

```python
import psycopg
import os
from dotenv import load_dotenv

load_dotenv()

conn = psycopg.connect(os.getenv("DATABASE_URL"))

with conn.cursor() as cur:
    # Enable pgvector
    cur.execute("CREATE EXTENSION IF NOT EXISTS vector;")

    # Create tables
    cur.execute("""
        CREATE TABLE IF NOT EXISTS chat_sessions (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            expires_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP + INTERVAL '7 days',
            metadata JSONB DEFAULT '{}'::jsonb
        );

        CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON chat_sessions(expires_at);
    """)

    cur.execute("""
        CREATE TABLE IF NOT EXISTS chat_messages (
            id SERIAL PRIMARY KEY,
            session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
            role VARCHAR(10) CHECK (role IN ('user', 'assistant')),
            content TEXT NOT NULL,
            sources JSONB,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );

        CREATE INDEX IF NOT EXISTS idx_messages_session ON chat_messages(session_id, created_at DESC);
    """)

    conn.commit()

print("✅ Postgres schema created successfully!")
```

Run:

```bash
python scripts/init_postgres.py
```

---

## Part 4: Content Ingestion (10 min)

### 4.1 Create Ingestion Script

Create `backend/scripts/ingest_textbook.py`:

```python
import os
import sys
from pathlib import Path
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient
from langchain.text_splitter import RecursiveCharacterTextSplitter
import re
import time

load_dotenv()

# Initialize clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Text splitter
splitter = RecursiveCharacterTextSplitter(
    separators=["\n## ", "\n### ", "\n\n", "\n", " ", ""],
    chunk_size=1024,
    chunk_overlap=256
)

def strip_mdx(text):
    """Remove JSX/MDX syntax"""
    text = re.sub(r'^import\s+.*?;$', '', text, flags=re.MULTILINE)
    text = re.sub(r'^export\s+.*$', '', text, flags=re.MULTILINE)
    text = re.sub(r'<[A-Z]\w*[^>]*>', '', text)
    text = re.sub(r'</[A-Z]\w*>', '', text)
    text = re.sub(r'<[A-Z]\w*[^>]*/>', '', text)
    return text.strip()

def ingest():
    docs_dir = Path("../../frontend/docs")
    if not docs_dir.exists():
        print(f"❌ Error: {docs_dir} not found")
        sys.exit(1)

    all_chunks = []
    chunk_id = 0

    # Process all .md and .mdx files
    for mdx_file in docs_dir.rglob("*.md*"):
        print(f"Processing: {mdx_file.name}")

        content = mdx_file.read_text(encoding='utf-8')
        clean_text = strip_mdx(content)

        # Extract module from path
        rel_path = mdx_file.relative_to(docs_dir)
        module = str(rel_path.parts[0]) if len(rel_path.parts) > 1 else "intro"

        # Split into chunks
        chunks = splitter.split_text(clean_text)

        for i, chunk in enumerate(chunks):
            if not chunk.strip():
                continue

            # Generate embedding
            embedding = openai_client.embeddings.create(
                input=chunk,
                model="text-embedding-3-small"
            ).data[0].embedding

            # Prepare point
            all_chunks.append({
                "id": chunk_id,
                "vector": embedding,
                "payload": {
                    "text": chunk,
                    "module": module,
                    "chapter": mdx_file.stem,
                    "source_url": f"/docs/{rel_path.with_suffix('').as_posix()}",
                    "chunk_index": i,
                    "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ")
                }
            })

            chunk_id += 1

            # Batch upsert every 50 chunks
            if len(all_chunks) >= 50:
                qdrant_client.upsert(
                    collection_name="textbook_embeddings",
                    points=all_chunks
                )
                print(f"  Uploaded {len(all_chunks)} chunks...")
                all_chunks = []

    # Upload remaining chunks
    if all_chunks:
        qdrant_client.upsert(
            collection_name="textbook_embeddings",
            points=all_chunks
        )

    print(f"\n✅ Ingestion complete! Total chunks: {chunk_id}")

if __name__ == "__main__":
    ingest()
```

### 4.2 Run Ingestion

```bash
cd backend/scripts
python ingest_textbook.py
```

**Expected Output**:

```
Processing: intro.md
Processing: nodes.md
...
  Uploaded 50 chunks...
  Uploaded 50 chunks...
...
✅ Ingestion complete! Total chunks: 450
```

**Ingestion Time**: ~2-3 minutes for 450 chunks

---

## Part 5: Backend Development (10 min)

### 5.1 Create FastAPI App

Create `backend/src/main.py`:

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

load_dotenv()

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# CORS for GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://jahansher333.github.io", "http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    session_id: str

@app.post("/query")
async def query_textbook(req: QueryRequest):
    # TODO: Implement RAG logic (see research.md for details)
    return {
        "answer": "This is a placeholder response. Implement RAG agent here.",
        "sources": [],
        "confidence": 0.5
    }

@app.get("/")
async def root():
    return {"status": "ok", "message": "RAG Chatbot API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

### 5.2 Run Development Server

```bash
cd backend
python -m uvicorn src.main:app --reload
```

Visit: http://localhost:8000/docs (Swagger UI)

---

## Part 6: Frontend Integration (5 min)

### 6.1 Install Frontend Dependencies

```bash
cd frontend
npm install dompurify
npm install --save-dev @types/dompurify
```

### 6.2 Create Chat Widget Component

Create `frontend/src/components/ChatWidget/index.tsx` (see research.md section 4 for full code)

### 6.3 Add to Docusaurus

Edit `frontend/src/theme/Root.tsx`:

```tsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

---

## Part 7: Testing (5 min)

### 7.1 Test Ingestion

```bash
curl http://localhost:8000/
# Expected: {"status":"ok","message":"RAG Chatbot API"}
```

### 7.2 Test Query Endpoint

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "selected_text": null,
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'
```

### 7.3 Test Frontend Widget

```bash
cd frontend
npm start
```

Visit: http://localhost:3000
- Click chat button (bottom-right)
- Type a question
- Verify API call in browser DevTools

---

## Part 8: Deployment (Optional, 15 min)

### 8.1 Deploy Backend to Render

1. Push code to GitHub
2. Go to https://render.com/dashboard
3. Click **"New +" → "Web Service"**
4. Connect GitHub repo
5. Configure:
   - **Name**: `textbook-rag-backend`
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - **Environment**: Add all `.env` variables

### 8.2 Update Frontend API URL

Edit `frontend/src/components/ChatWidget/ChatWidgetClient.tsx`:

```typescript
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://textbook-rag-backend.onrender.com'
  : 'http://localhost:8000';
```

### 8.3 Deploy Frontend to GitHub Pages

```bash
cd frontend
npm run build
npm run deploy
```

---

## Verification Checklist

- [ ] Qdrant collection created with 450 chunks
- [ ] Postgres tables created (chat_sessions, chat_messages)
- [ ] Backend runs locally at http://localhost:8000
- [ ] Frontend runs locally at http://localhost:3000
- [ ] Chat widget appears on page
- [ ] Query endpoint returns answers
- [ ] Sources are cited correctly
- [ ] Session history persists across page nav

---

## Troubleshooting

### Issue: "Failed to connect to Qdrant"

**Solution**: Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`

```bash
# Test connection
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
print(client.get_collections())
"
```

### Issue: "OpenAI API rate limit exceeded"

**Solution**: Add delays between embedding calls in ingestion script

```python
import time
time.sleep(0.1)  # 100ms delay per embedding
```

### Issue: "Chat widget not loading"

**Solution**: Check browser console for CORS errors. Update `CORSMiddleware` origins in `main.py`.

---

## Next Steps

1. ✅ Environment and databases set up
2. ✅ Content ingested to Qdrant
3. ⏭ Implement full RAG agent logic (see `research.md` section 1)
4. ⏭ Add rate limiting (10 queries/minute)
5. ⏭ Deploy to production
6. ⏭ Validate with gold Q&A set (50 questions)

---

**Quickstart Version**: 1.0
**Last Updated**: 2025-12-04
**Estimated Total Time**: 60 minutes
