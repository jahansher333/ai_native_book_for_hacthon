# ✅ CORRECT Environment Variables for Railway

**IMPORTANT:** Your backend uses **Groq API with LiteLLM**, NOT Gemini!

---

## 🔑 Required Environment Variables

### 1. Groq API (for LLM responses) - **CRITICAL**
```bash
GROQ_API_KEY=your_groq_api_key_here
```
- **Used for:** RAG agent with `groq/llama-3.3-70b-versatile` model
- **Get from:** https://console.groq.com/keys
- **Free tier:** Available
- **Cost:** 90% cheaper than Gemini (as noted in code)

### 2. Gemini API (for embeddings only) - **CRITICAL**
```bash
GEMINI_API_KEY=your_gemini_api_key_here
```
- **Used for:** Text embeddings for vector search
- **Get from:** https://aistudio.google.com/apikey
- **Free tier:** 1500 requests/day
- **Note:** Only for embeddings, not for chat responses

### 3. Qdrant Vector Database - **CRITICAL**
```bash
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_vectors
```
- **Used for:** Semantic search of textbook content
- **Get from:** https://cloud.qdrant.io/
- **Free tier:** 1GB storage

### 4. Neon PostgreSQL - **REQUIRED for Auth**
```bash
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require
```
- **Used for:** User authentication, sessions, query logs
- **Get from:** https://neon.tech/
- **Free tier:** 0.5GB storage

### 5. JWT Secret - **REQUIRED for Auth**
```bash
JWT_SECRET=your_super_secret_key_minimum_32_characters_long
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7
```
- **Used for:** Secure authentication tokens
- **Generate:** Use a random 32+ character string

### 6. CORS Configuration - **RECOMMENDED**
```bash
CORS_ORIGINS=http://localhost:3000,https://ai-native-books-pyhsical-ai-kcpd.vercel.app,https://web-production-d0418.up.railway.app
```
- **Used for:** Frontend access control
- **Update:** Add your actual Vercel frontend URL

---

## 🏗️ Architecture - What Uses What

```
┌─────────────────────────────────────┐
│  User Question                       │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  1. GEMINI API (Embeddings)         │  ← GEMINI_API_KEY
│  • Convert question to vector       │
│  • Used by: embedding_service       │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  2. QDRANT (Vector Search)          │  ← QDRANT_URL, QDRANT_API_KEY
│  • Find relevant textbook chunks    │
│  • Used by: vector_store_service    │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  3. GROQ API (LLM Response)         │  ← GROQ_API_KEY
│  • Model: llama-3.3-70b-versatile   │
│  • Generate answer with citations   │
│  • Used by: LiteLLM RAG agent       │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  4. NEON DB (Logging)               │  ← NEON_DATABASE_URL
│  • Store query history              │
│  • Store user sessions              │
└─────────────────────────────────────┘
```

---

## ⚙️ Optional Environment Variables

```bash
# Model Configuration
MODEL_NAME=gemini-2.0-flash-exp  # For embeddings
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# RAG Settings
TOP_K_RESULTS=5
SIMILARITY_THRESHOLD=0.3
CHUNK_SIZE=1000
CHUNK_OVERLAP=200

# Application
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO
```

---

## 🔍 What Each Service Does

### Groq API (PRIMARY LLM) ✅
- **Purpose:** Generate RAG responses
- **Model:** `groq/llama-3.3-70b-versatile`
- **File:** `backend/src/ai_agents/litellm_converted/rag_agent.py:90`
- **Endpoint:** Used in `/api/query`
- **Why Groq?** 90% cost reduction vs Gemini (noted in code comments)

### Gemini API (EMBEDDINGS ONLY) ✅
- **Purpose:** Text-to-vector conversion
- **Model:** `gemini-2.0-flash-exp` (configurable)
- **File:** `backend/src/services/embeddings.py`
- **Endpoint:** Used before Qdrant search
- **Note:** NOT used for chat completions

### Qdrant (VECTOR DATABASE) ✅
- **Purpose:** Semantic search of textbook
- **Collection:** `book_vectors` (default)
- **File:** `backend/src/services/vector_store.py`
- **Endpoint:** Used in `/api/query` for context retrieval

### Neon (USER DATABASE) ✅
- **Purpose:** User auth, sessions, logs
- **Tables:** `users`, `sessions`, `query_logs`
- **File:** `backend/src/services/session_manager.py`, `backend/src/services/auth_service.py`
- **Endpoints:** `/api/auth/*`, `/api/query` (logging)

---

## 🚨 Common Mistakes

### ❌ WRONG: Setting only GEMINI_API_KEY
```bash
# This will NOT work!
GEMINI_API_KEY=your_key  # Only for embeddings
# Missing: GROQ_API_KEY (for chat!)
```

### ✅ CORRECT: Setting both GROQ and GEMINI
```bash
GROQ_API_KEY=your_groq_key       # For chat responses
GEMINI_API_KEY=your_gemini_key   # For embeddings
```

---

## 📋 Quick Setup Checklist

1. [ ] Get Groq API key from https://console.groq.com/keys
2. [ ] Get Gemini API key from https://aistudio.google.com/apikey
3. [ ] Create Qdrant cluster at https://cloud.qdrant.io/
4. [ ] Create Neon database at https://neon.tech/
5. [ ] Generate JWT secret (32+ chars)
6. [ ] Add all variables to Railway
7. [ ] Wait for redeploy
8. [ ] Test with: `bash test-api-keys.sh`

---

## 🧪 Testing After Configuration

### Test 1: Health Check
```bash
curl https://web-production-d0418.up.railway.app/api/health
```

**Expected (after config):**
```json
{
    "status": "healthy",
    "services": {
        "qdrant": "healthy",
        "neon": "healthy",
        "gemini": "healthy"
    }
}
```

### Test 2: RAG Query
```bash
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS2?"}'
```

**Expected (after config):**
```json
{
    "answer": "ROS 2 is the next generation... [Source: ROS 2 - Introduction, Page 5]",
    "citations": [...],
    "query_id": "...",
    "latency_ms": 2500
}
```

---

## 💰 Cost Comparison (as per your code comments)

| Service | Cost vs Gemini |
|---------|---------------|
| **Groq API** | **90% cheaper** ✅ |
| Gemini (embeddings only) | Minimal usage |
| **Total Savings** | ~90% |

This is why your backend was migrated from Gemini to Groq!

---

## 🎯 Priority Order

1. **GROQ_API_KEY** - Without this, NO chat responses work
2. **QDRANT_URL + QDRANT_API_KEY** - Without these, NO search works
3. **GEMINI_API_KEY** - Without this, NO embeddings work
4. **NEON_DATABASE_URL** - Without this, auth/logging doesn't work
5. **JWT_SECRET** - Without this, auth tokens don't work

---

**TL;DR:** Your backend uses **GROQ for chat** and **GEMINI for embeddings**. Both are required! 🚀
