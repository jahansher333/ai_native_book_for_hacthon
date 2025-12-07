# Testing Groq LiteLLM Agent

This guide shows how to test the Groq-powered RAG agent once API keys are configured.

---

## 🎯 What We're Testing

Your backend uses:
- **Groq API** with **LiteLLM** for chat responses
- **Model:** `groq/llama-3.3-70b-versatile`
- **Agent:** OpenAI Agents SDK with function tools
- **Tool:** `search_textbook` for RAG retrieval

---

## 📋 Prerequisites

Before testing, ensure these environment variables are set in Railway:

```bash
GROQ_API_KEY=gsk_...              # Get from https://console.groq.com/keys
GEMINI_API_KEY=...                # For embeddings
QDRANT_URL=https://....qdrant.io
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgresql://...
JWT_SECRET=...
```

---

## 🧪 Test 1: Simple Query (No Auth Required)

### cURL Test:
```bash
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS2?"
  }'
```

### Expected Response:
```json
{
  "answer": "ROS 2 is the next generation of the Robot Operating System... [Source: ROS 2 - Introduction, Page 5]",
  "citations": [
    {
      "module": "ROS 2",
      "section": "Introduction",
      "page": 5,
      "source_file": "ros-2/introduction.md",
      "chunk_id": "",
      "relevance_score": 0.85,
      "text_snippet": ""
    }
  ],
  "latency_ms": 2500,
  "query_id": "uuid-here",
  "timestamp": "2025-12-07T..."
}
```

---

## 🧪 Test 2: Query with Selected Text

### cURL Test:
```bash
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this concept",
    "mode": "selected",
    "selected_text": "ROS 2 uses DDS for communication between nodes"
  }'
```

### What Happens:
1. Agent receives question + selected text
2. Uses selected text as primary context
3. Searches textbook for additional context
4. Generates answer using Groq LLM
5. Returns response with citations

---

## 🧪 Test 3: Query with Session ID

### cURL Test:
```bash
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is Gazebo?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'
```

### What Happens:
- Query is logged to Neon database
- Retrievable via `/api/history/{session_id}`
- Session tracks conversation history

---

## 🧪 Test 4: Check Query History

### cURL Test:
```bash
curl https://web-production-d0418.up.railway.app/api/history/550e8400-e29b-41d4-a716-446655440000?limit=10
```

### Expected Response:
```json
[
  {
    "query_id": "...",
    "query_text": "What is Gazebo?",
    "response_text": "Gazebo is...",
    "citations": [...],
    "timestamp": "2025-12-07T...",
    "query_mode": "general"
  }
]
```

---

## 🧪 Test 5: Test Agent Tool (search_textbook)

The RAG agent has one tool: `search_textbook`

### What It Does:
1. Takes user's question
2. Converts to embedding (Gemini)
3. Searches Qdrant vector database
4. Returns top-k relevant chunks
5. Formats as context for Groq LLM

### Agent Instructions:
```python
# From rag_agent.py lines 97-115
CRITICAL RULES:
1. Answer ONLY using textbook information
2. MUST use search_textbook function first
3. Always cite sources: [Source: Module - Section, Page X]
4. Refuse non-textbook questions
5. Prioritize selected text if provided
6. Be concise (2-4 paragraphs max)
```

---

## 🧪 Test 6: Frontend Integration Test

### JavaScript/TypeScript:
```typescript
// Test in browser console or React component
const testQuery = async () => {
  const response = await fetch('https://web-production-d0418.up.railway.app/api/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      question: 'What is Isaac Sim?',
      mode: 'general'
    })
  });

  const data = await response.json();
  console.log('Answer:', data.answer);
  console.log('Citations:', data.citations);
  console.log('Latency:', data.latency_ms, 'ms');
};

testQuery();
```

---

## 🎯 Expected Agent Behavior

### Scenario 1: Textbook Question
**User:** "What is ROS2?"
**Agent:**
1. Calls `search_textbook("What is ROS2?")`
2. Gets relevant chunks from Qdrant
3. Uses Groq to generate answer
4. Returns answer with citations

### Scenario 2: Non-Textbook Question
**User:** "What's the weather today?"
**Agent:** "I can only answer questions about the Physical AI & Humanoid Robotics textbook content. Please ask about ROS 2, Gazebo, Unity, Isaac Sim, Vision-Language-Action models, or hardware requirements."

### Scenario 3: Rate Limit Error
**If Groq API hits rate limit:**
```json
{
  "answer": "⚠️ API rate limit exceeded. The Groq API has reached its quota limit. Please try again in a minute...",
  "citations": [],
  "confidence": 0.0
}
```

---

## 🧪 Test 7: Performance Benchmarks

### Expected Timings:
- **Embedding generation:** ~200-500ms (Gemini)
- **Vector search:** ~100-200ms (Qdrant)
- **LLM response:** ~1-3s (Groq)
- **Total latency:** ~2-4s

### Groq Model Performance:
- **Model:** `llama-3.3-70b-versatile`
- **Speed:** Fast inference (optimized hardware)
- **Quality:** High-quality responses
- **Cost:** 90% cheaper than Gemini

---

## 🧪 Test 8: Error Scenarios

### Test Missing API Key:
**Current State (without GROQ_API_KEY):**
```json
{
  "detail": "Failed to process query: 'NoneType' object is not callable"
}
```

### Test Timeout (>60s):
```json
{
  "answer": "The query took too long (>60s). Try a simpler question or try again later.",
  "citations": [],
  "confidence": 0.0
}
```

---

## 📊 Monitoring Agent Performance

### Check Logs in Railway:
1. Go to Railway Dashboard
2. Click Deployments
3. View Logs
4. Look for:
   - `*** LiteLLM RAG agent loaded ***`
   - Query processing logs
   - Tool invocations
   - Response times

---

## 🔍 Debugging Checklist

If queries fail:

1. **Check Health Endpoint:**
   ```bash
   curl https://web-production-d0418.up.railway.app/api/health
   ```
   Should show all services "healthy"

2. **Verify GROQ_API_KEY:**
   - Set in Railway Variables
   - Starts with `gsk_`
   - Not expired

3. **Verify GEMINI_API_KEY:**
   - Required for embeddings
   - Set in Railway Variables

4. **Verify Qdrant:**
   - Cluster is active
   - Collection exists: `book_vectors`
   - API key is correct

5. **Check Railway Logs:**
   - Any import errors?
   - API connection errors?
   - Timeout errors?

---

## 🚀 Once API Keys Are Set

### Health Check Should Show:
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

### Test Command:
```bash
# Simple test
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{"question":"Hello"}' | jq
```

### Should Return:
- Real AI-generated answer
- Source citations
- Query ID
- Latency metrics

---

## 📝 Testing Checklist

- [ ] GROQ_API_KEY set in Railway
- [ ] GEMINI_API_KEY set in Railway
- [ ] QDRANT credentials set
- [ ] Health endpoint shows "healthy"
- [ ] Simple query returns answer
- [ ] Citations are included
- [ ] Selected text mode works
- [ ] Session ID tracking works
- [ ] Query history retrievable
- [ ] Frontend can connect
- [ ] Error handling works
- [ ] Rate limiting is handled

---

**Ready to test once API keys are configured!** 🚀
