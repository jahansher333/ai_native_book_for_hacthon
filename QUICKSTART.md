# 🚀 RAG Chatbot - Quick Start Guide

Get your RAG chatbot running in **5 minutes**!

## ✅ Prerequisites Configured

Your API keys are already set up:
- ✅ Gemini API: Configured
- ✅ Qdrant Cloud: Configured (cluster in us-east-1)
- ✅ Neon Postgres: Configured

## Step 1: Install Dependencies (2 minutes)

```bash
# Navigate to backend
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On Mac/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

Expected output: ~20 packages installed

## Step 2: Test Setup (30 seconds)

```bash
# Run setup test script
python test_setup.py
```

Expected output:
```
🚀 RAG Chatbot Setup Test
✅ Gemini API working! Embedding dimension: 768
✅ Qdrant Cloud connected!
✅ Neon Postgres connected!
🎉 All tests passed! Ready to start the server.
```

If you see any ❌ FAIL, check:
- Internet connection
- API keys in `.env` file
- Qdrant cluster is active
- Neon database is active

## Step 3: Start Backend Server (10 seconds)

```bash
# Start FastAPI server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
🚀 Starting RAG Chatbot Backend...
✅ Database tables created/verified
✅ Created collection: book_vectors
INFO:     Uvicorn running on http://0.0.0.0:8000
```

✅ **Verify**: Open http://localhost:8000 in your browser

You should see:
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0",
  "status": "running"
}
```

## Step 4: Check Health (5 seconds)

Open a **new terminal** and run:

```bash
curl http://localhost:8000/api/health
```

Expected output:
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

✅ All services should be "healthy"

## Step 5: Ingest Textbook Content (2-3 minutes)

**IMPORTANT**: The chatbot needs textbook content to answer questions!

```bash
# Ingest all Markdown files from docs/
curl -X POST http://localhost:8000/api/ingest
```

Expected output:
```json
{
  "status": "success",
  "files_processed": 29,
  "chunks_created": 482,
  "chunks_ingested": 482
}
```

⏱️ This takes **2-3 minutes** to:
- Load all .md/.mdx files from `docs/`
- Chunk content (1000 chars, 200 overlap)
- Generate embeddings with Gemini
- Upload to Qdrant

Watch the terminal for progress:
```
📚 Loading documents from: D:\...\docs
📄 Found 29 markdown files
  ✓ intro.md: 15 chunks
  ✓ module-1-ros2.md: 23 chunks
  ...
🔢 Total chunks created: 482
  Uploaded batch 1: 50/482 chunks
  Uploaded batch 2: 100/482 chunks
  ...
✅ Ingestion complete! 482 chunks uploaded to Qdrant
```

## Step 6: Test Query (5 seconds)

```bash
# Test a simple question
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What is ROS 2?\"}"
```

Expected response time: **< 3 seconds**

Expected output:
```json
{
  "answer": "ROS 2 is the Robot Operating System 2, the next generation of ROS...",
  "citations": [
    {
      "module": "ros2",
      "section": "introduction",
      "page": 8,
      "relevance_score": 0.92
    }
  ],
  "latency_ms": 2341,
  "query_id": "...",
  "timestamp": "2025-12-05T..."
}
```

✅ **Success!** Your backend is working!

## Step 7: Start Frontend (30 seconds)

Open a **new terminal**:

```bash
# Navigate to frontend
cd frontend

# Install dependencies (if not already done)
npm install

# Start Docusaurus
npm start
```

Expected output:
```
Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

## Step 8: Test the Widget (30 seconds)

1. Open http://localhost:3000 in your browser
2. Look for the **💬 chat button** in the bottom-right corner
3. Click to open the chat panel
4. Type: **"What is ROS 2?"**
5. Press Enter or click Send
6. Wait 2-3 seconds
7. ✅ You should see an answer with citations!

### Test Selected Text Mode:

1. Highlight any text on the page (e.g., "Jetson Orin Nano")
2. Notice the **📌 Selected:** badge in the chat
3. Type: **"Explain this hardware"**
4. Send the message
5. ✅ The answer should focus on your selected text!

## 🎉 You're Done!

Your RAG chatbot is now fully functional!

### What You Can Do Now:

1. **Ask Questions**:
   - "What is ROS 2?"
   - "How much does the Jetson Orin Nano cost?"
   - "What is URDF?"
   - "Explain sim-to-real transfer"

2. **Use Selected Text**:
   - Highlight any text on the page
   - Ask questions about it
   - Get focused answers

3. **View Chat History**:
   - Your conversations are saved
   - Navigate between pages
   - History persists in the same session

## 📊 Performance Metrics

After testing, you should see:
- ⚡ Query response time: **< 3 seconds**
- 🎯 Answers include **citations**
- 📚 Bot answers **only from textbook**
- 🚫 Refuses non-textbook questions

## 🐛 Troubleshooting

### Backend won't start?
```bash
# Check if port 8000 is in use
# On Windows:
netstat -ano | findstr :8000
# On Mac/Linux:
lsof -i :8000

# Kill the process if needed
# Then restart: uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### Health check fails?
```bash
# Re-run setup test
python test_setup.py

# Check .env file has correct values
cat .env  # Mac/Linux
type .env  # Windows
```

### No answer returned?
```bash
# Check if content was ingested
curl http://localhost:8000/api/health

# Re-ingest if needed
curl -X POST http://localhost:8000/api/ingest
```

### Widget not showing?
1. Check browser console for errors (F12)
2. Verify backend is running: http://localhost:8000
3. Check CORS settings in `.env`
4. Clear browser cache and reload

## 📚 Next Steps

### 1. Test Different Queries

Try these example questions:
```bash
# General questions
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What are the 4 modules in this course?\"}"

# Hardware pricing
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"How much does the Economy Jetson Kit cost?\"}"

# Technical concepts
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"Explain URDF in ROS 2\"}"
```

### 2. Test Selected Text Mode

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d "{
    \"question\": \"Explain this hardware\",
    \"mode\": \"selected\",
    \"selected_text\": \"Jetson Orin Nano: $249, 40 TOPS INT8\"
  }"
```

### 3. View API Documentation

Open in browser: **http://localhost:8000/docs**

This shows interactive API documentation where you can:
- Test all endpoints
- See request/response schemas
- Try different parameters

### 4. Monitor Logs

Watch the terminal where `uvicorn` is running to see:
- Incoming requests
- Query processing
- Embedding generation
- Vector search results
- Response times

### 5. Deploy to Production

When ready to deploy, follow the guide in:
**`README_RAG_DEPLOYMENT.md`**

Deployment options:
- **Backend**: Render (free tier)
- **Frontend**: GitHub Pages (free)

## 🎓 Understanding the System

### How It Works:

1. **User asks a question** → Frontend sends to backend
2. **Backend embeds question** → Gemini API generates embedding
3. **Vector search** → Qdrant finds top-5 similar chunks
4. **RAG Agent** → OpenAI Agents SDK uses function tool to search
5. **Gemini generates answer** → Using retrieved context
6. **Extract citations** → Parse source references
7. **Store in database** → Neon Postgres logs query
8. **Return to frontend** → Display answer with citations

### Architecture Diagram:

```
User Question
    ↓
FastAPI /query
    ↓
OpenAI Agents SDK
    ├─> @function_tool: search_textbook()
    │       ├─> Gemini Embedding
    │       └─> Qdrant Search (top-5 chunks)
    └─> Gemini Chat Completion
    ↓
Answer + Citations
    ├─> Store in Neon Postgres
    └─> Return to Frontend
```

## ✅ Checklist

- [ ] Backend dependencies installed
- [ ] Setup test passed (all ✅)
- [ ] Server started (port 8000)
- [ ] Health check passed
- [ ] Content ingested (450-500 chunks)
- [ ] Query test passed
- [ ] Frontend started (port 3000)
- [ ] Widget visible and working
- [ ] Test queries return answers
- [ ] Citations appear in responses
- [ ] Selected text mode works
- [ ] Chat history persists

## 🆘 Need Help?

1. **Check logs** in the terminal where uvicorn is running
2. **API docs**: http://localhost:8000/docs
3. **Health check**: http://localhost:8000/api/health
4. **Full guide**: `README_RAG_DEPLOYMENT.md`
5. **Backend docs**: `backend/README.md`

## 🎉 Success!

You now have a fully functional RAG chatbot that:
- ✅ Answers questions from the textbook
- ✅ Cites sources for all answers
- ✅ Supports selected text queries
- ✅ Persists chat history
- ✅ Works on mobile devices
- ✅ Refuses non-textbook questions

**Happy chatting! 🤖📚**

---

Last updated: 2025-12-05
Status: ✅ READY FOR USE
