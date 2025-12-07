# Quick Fix Guide - Get Your Agent Working in 5 Minutes

**Current Status:** Backend is LIVE but agent needs API keys

---

## Problem
```
Error: "Failed to process query: 'NoneType' object is not callable"
```

## Cause
Missing API keys in Railway environment variables.

---

## Solution (5 minutes)

### 1. Get API Keys (3 minutes)

**Groq API Key** (PRIMARY - REQUIRED):
1. Go to: https://console.groq.com/keys
2. Sign up / Sign in
3. Click "Create API Key"
4. Copy key (starts with `gsk_`)

**Gemini API Key** (REQUIRED for embeddings):
1. Go to: https://aistudio.google.com/apikey
2. Sign in with Google
3. Click "Create API Key"
4. Copy key

**Qdrant Credentials** (REQUIRED for search):
1. Go to: https://cloud.qdrant.io/
2. Sign up / Sign in
3. Create cluster (free tier)
4. Copy URL and API key

---

### 2. Set in Railway (2 minutes)

1. Go to: https://railway.app/
2. Select your project
3. Click **Variables** tab
4. Add these 3 critical variables:

```bash
GROQ_API_KEY=gsk_your_key_here
GEMINI_API_KEY=your_gemini_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
```

5. Click **Save**
6. Wait for redeploy (~2 minutes)

---

### 3. Test (30 seconds)

```bash
curl -X POST https://web-production-d0418.up.railway.app/api/query \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS2?"}'
```

**Should return:** AI-generated answer with citations ✅

---

## Done!

Your LiteLLM RAG agent will now work perfectly.

**Agent Uses:**
- **Groq** (`llama-3.3-70b-versatile`) for chat responses
- **Gemini** for embeddings only
- **Qdrant** for textbook search

**Backend URL:** https://web-production-d0418.up.railway.app/

---

## Optional (for full features)

Add these for authentication and logging:

```bash
NEON_DATABASE_URL=postgresql://...
JWT_SECRET=random_32_char_string
CORS_ORIGINS=http://localhost:3000,https://your-frontend.vercel.app
```

---

## Need Help?

See detailed guides:
- `LIVE_TEST_RESULTS.md` - Full test report
- `TEST_GROQ_LITELLM.md` - Testing guide
- `CORRECT_ENV_VARS.md` - All environment variables
- `test-groq-agent.html` - Interactive testing UI
