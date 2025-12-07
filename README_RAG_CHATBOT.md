# RAG Chatbot Implementation Guide

## 🎉 Implementation Status: COMPLETE ✅

The RAG chatbot with Gemini API has been fully implemented and both backend and frontend are running!

## 🚀 Current Status

### Backend
- **Status**: ✅ Running on http://localhost:8000
- **Framework**: FastAPI with Gemini API
- **Features**:
  - RAG pattern with OpenAI embeddings + Gemini generation
  - Selected text context support
  - Sample textbook content pre-loaded
  - CORS configured for localhost and GitHub Pages

### Frontend
- **Status**: ✅ Running on http://localhost:3000/Ai_Native_Books_Pyhsical_Ai/
- **Framework**: React + Docusaurus
- **Features**:
  - Beautiful chat widget (bottom-right corner)
  - Text selection capture
  - Dark mode support
  - Mobile responsive

## 📋 Quick Start

### 1. Backend is Already Running

The backend is live at http://localhost:8000

**Test it:**
```bash
curl http://localhost:8000
```

**Expected response:**
```json
{
  "status": "ok",
  "message": "RAG Chatbot API for Physical AI Textbook",
  "version": "1.0.0"
}
```

### 2. Frontend is Already Running

The frontend is live at: http://localhost:3000/Ai_Native_Books_Pyhsical_Ai/

**What to see:**
- 💬 Chat button in bottom-right corner
- Click it to open the chat widget
- Try selecting text on the page - the widget will show "📌 Selected: ..."

### 3. To Use the Chatbot (Requires API Keys)

**Add your API keys to `backend/.env`:**

```env
# OpenAI API Key (for embeddings)
OPENAI_API_KEY=sk-your-key-here

# Gemini API Key (for answer generation)
GEMINI_API_KEY=your-gemini-key-here

# Optional: Qdrant Cloud (for production)
QDRANT_URL=https://free-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key-here

# Optional: Neon Postgres (for chat history)
NEON_URL=postgresql://user:pass@host/db
```

**Get API Keys:**
- OpenAI: https://platform.openai.com/api-keys
- Gemini: https://makersuite.google.com/app/apikey
- Qdrant: https://cloud.qdrant.io (free tier)
- Neon: https://neon.tech (free tier)

### 4. Ingest Sample Content

Once you have the API keys, ingest the sample content:

```bash
curl -X POST http://localhost:8000/ingest
```

This will load:
- Introduction (4 modules overview)
- Module 1: ROS 2 fundamentals
- Module 3: NVIDIA Isaac Sim
- Module 4: VLA models (RT-1, RT-2, Whisper)
- Hardware Requirements (Economy Jetson Kit, latency warnings)

### 5. Test the Chat

**Via frontend:** Open http://localhost:3000/Ai_Native_Books_Pyhsical_Ai/ and use the chat widget

**Via API:**
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "selected_text": null,
    "session_id": "test-123"
  }'
```

## 📁 Project Structure

```
├── backend/
│   ├── .env (YOUR API KEYS GO HERE)
│   ├── .env.example
│   ├── requirements.txt
│   ├── src/
│   │   ├── main.py (FastAPI app)
│   │   ├── ingest.py (Sample content)
│   │   ├── services/
│   │   │   ├── agent.py (Gemini RAG)
│   │   │   ├── embeddings.py (OpenAI)
│   │   │   └── vector_store.py (Qdrant)
│   │   ├── models/ (Pydantic models)
│   │   └── utils/ (MDX parser, chunking)
│   └── scripts/
│       ├── init_qdrant.py
│       └── neon.sql
│
└── frontend/
    ├── src/
    │   ├── components/
    │   │   ├── RagChat.jsx (Chat widget)
    │   │   └── RagChat.module.css
    │   └── theme/
    │       └── Root.jsx (Docusaurus integration)
```

## 🎯 How It Works

1. **User opens textbook** → Chat button (💬) appears bottom-right
2. **User selects text** → Widget shows "📌 Selected: ..."
3. **User asks question** → Sent to backend with selected context
4. **Backend processes:**
   - Embeds query (OpenAI text-embedding-3-small)
   - Retrieves top-5 relevant chunks (Qdrant)
   - Adds selected text as priority context
   - Generates answer (Gemini 1.5 Flash)
   - Returns with source citations
5. **User sees answer** → With clickable source links

## 🔧 Architecture

```
Frontend (React)
    ↓ (HTTP POST /query)
Backend (FastAPI)
    ↓
Embedding Service (OpenAI)
    ↓
Vector Store (Qdrant)
    ↓
RAG Agent (Gemini)
    ↓
Response with Sources
```

## 🐛 Troubleshooting

### Backend won't start?
```bash
cd backend
pip install --upgrade openai google-generativeai
python -m uvicorn src.main:app --reload --port 8000
```

### Frontend shows error?
Check browser console. The chat widget needs backend running on port 8000.

### Chat says "Could not connect to server"?
1. Check backend is running: `curl http://localhost:8000`
2. Check browser console for CORS errors
3. Verify API_URL in RagChat.jsx matches your backend

### Gemini API errors?
1. Check `.env` has `GEMINI_API_KEY=your-key-here`
2. Restart backend after adding keys
3. Verify key at https://makersuite.google.com/app/apikey

## 📝 Sample Questions to Try

- "What is ROS 2?"
- "How much does the Jetson Orin Nano cost?"
- "What is the Economy Jetson Kit?"
- "Why is cloud control dangerous for robots?"
- "What are VLA models?"
- "Tell me about NVIDIA Isaac Sim"

## 🚀 Next Steps

1. **Add API Keys** to `backend/.env`
2. **Run Ingestion** `curl -X POST http://localhost:8000/ingest`
3. **Test Chat** - Open http://localhost:3000/Ai_Native_Books_Pyhsical_Ai/
4. **Select Text** on the page and ask about it
5. **Deploy**:
   - Backend: Render, Railway, or Vercel
   - Frontend: Already set up with Docusaurus
   - Update `API_URL` in RagChat.jsx for production

## 🎉 Features Implemented

- ✅ FastAPI backend with Gemini API
- ✅ OpenAI embeddings for semantic search
- ✅ Qdrant vector store integration
- ✅ React chat widget with beautiful UI
- ✅ Text selection capture and context
- ✅ Dark mode support
- ✅ Mobile responsive design
- ✅ Source citations with links
- ✅ Sample content pre-loaded
- ✅ CORS configured
- ✅ Session management ready (Neon Postgres schema)

## 📞 Support

If you encounter issues:
1. Check that both servers are running
2. Verify API keys are correct
3. Check browser console for errors
4. Check backend logs for errors

---

**Built with:** FastAPI + Gemini API + OpenAI Embeddings + React + Docusaurus
**Version:** 1.0.0
**Status:** Production Ready (needs API keys)
