# Railway Backend Setup - Complete Guide

## ✅ Your Backend is LIVE!

**Backend URL:** https://web-production-d0418.up.railway.app/

### Current Status:
- ✅ Container Running
- ✅ Uvicorn Server Active
- ✅ API Responding
- ⚠️ Services Need Configuration (environment variables not set)

---

## 🔧 URGENT: Set Environment Variables

Your backend is running but needs environment variables to be fully functional.

### Go to Railway Dashboard:
1. Open https://railway.app/dashboard
2. Click on your project: `Ai_Native_Books_Pyhsical_Ai`
3. Click on your service
4. Go to **"Variables"** tab
5. Add these variables:

### Required Environment Variables:

```bash
# Gemini API (Required for RAG)
GEMINI_API_KEY=your_gemini_api_key_here

# Groq API (Required for personalization/translation)
GROQ_API_KEY=your_groq_api_key_here

# Qdrant Vector Database (Required for RAG)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_vectors

# Neon PostgreSQL (Required for auth and sessions)
NEON_DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# JWT Authentication (Required)
JWT_SECRET=your_super_secret_key_minimum_32_characters_long
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7

# Application Settings
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO

# CORS Origins (IMPORTANT - Add your frontend URL)
CORS_ORIGINS=http://localhost:3000,https://ai-native-books-pyhsical-ai-kcpd.vercel.app,https://web-production-d0418.up.railway.app

# Model Configuration
MODEL_NAME=gemini-2.0-flash-exp
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# RAG Settings
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5
SIMILARITY_THRESHOLD=0.3
```

### After Adding Variables:
Railway will automatically redeploy your service with the new environment variables.

---

## ✅ Frontend Already Updated

The frontend configuration has been updated to point to your Railway backend:

**File:** `frontend/src/config.ts`
```typescript
export const API_BASE_URL = 'https://web-production-d0418.up.railway.app';
```

---

## 🧪 Test Your Endpoints

### 1. Root Endpoint (Working ✅)
```bash
curl https://web-production-d0418.up.railway.app/
```
**Response:**
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0",
  "status": "running"
}
```

### 2. Health Check (Needs env vars ⚠️)
```bash
curl https://web-production-d0418.up.railway.app/api/health
```
**Current Response:**
```json
{
  "status": "degraded",
  "services": {
    "qdrant": "unhealthy",
    "neon": "unhealthy",
    "gemini": "unhealthy"
  }
}
```

**After setting env vars, should show:**
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

### 3. API Documentation (Working ✅)
Visit: https://web-production-d0418.up.railway.app/docs

---

## 📋 Setup Checklist

- [x] Backend deployed to Railway
- [x] Container running successfully
- [x] API responding to requests
- [x] Frontend config updated
- [ ] **Set environment variables in Railway** ← DO THIS NOW
- [ ] Verify health endpoint shows "healthy"
- [ ] Push frontend changes to GitHub
- [ ] Redeploy frontend to Vercel
- [ ] Test end-to-end flow

---

## 🚀 Next Steps

### 1. Set Environment Variables (Priority!)
   - Go to Railway Variables tab
   - Add all required variables from above
   - Wait for automatic redeploy (~2-3 minutes)

### 2. Push Frontend Changes
   ```bash
   git add frontend/src/config.ts
   git commit -m "feat: Connect frontend to Railway backend"
   git push
   ```

### 3. Redeploy Frontend to Vercel
   - Vercel will auto-deploy on git push
   - Or manually trigger deployment in Vercel dashboard

### 4. Test Complete Flow
   - Visit your Vercel frontend URL
   - Test authentication (signup/signin)
   - Test RAG chatbot queries
   - Test personalization features

---

## 🔍 Troubleshooting

### Services Show "Unhealthy"
**Cause:** Environment variables not set
**Solution:** Add all required env vars in Railway Variables tab

### CORS Errors in Browser
**Cause:** Frontend URL not in CORS_ORIGINS
**Solution:** Add your Vercel URL to CORS_ORIGINS:
```bash
CORS_ORIGINS=http://localhost:3000,https://your-frontend.vercel.app,https://web-production-d0418.up.railway.app
```

### Authentication Not Working
**Cause:** NEON_DATABASE_URL not set or JWT_SECRET missing
**Solution:** Set both variables in Railway

### RAG Queries Failing
**Cause:** GEMINI_API_KEY, QDRANT_URL, or QDRANT_API_KEY not set
**Solution:** Set all three variables in Railway

---

## 📊 Current Architecture

```
Frontend (Vercel)
    ↓
    ↓ HTTPS Requests
    ↓
Backend (Railway) ← https://web-production-d0418.up.railway.app
    ↓
    ├─→ Qdrant Cloud (Vector Database)
    ├─→ Neon Postgres (User Database)
    ├─→ Gemini API (LLM for RAG)
    └─→ Groq API (Personalization)
```

---

## ✅ Success Criteria

Your setup is complete when:
1. ✅ Health endpoint shows all services "healthy"
2. ✅ Frontend connects to Railway backend without CORS errors
3. ✅ Users can sign up and sign in
4. ✅ RAG chatbot responds to queries
5. ✅ Personalization features work

---

## 🎉 You're Almost There!

Just add the environment variables and you'll have a fully functional deployment! 🚀
