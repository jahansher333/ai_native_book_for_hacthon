# AI Robotics & Physical AI Textbook

An interactive educational platform for learning about Physical AI, Humanoid Robotics, and ROS 2, powered by AI agents and RAG technology.

## Quick Deploy

### Deploy Frontend (Docusaurus)
[![Deploy Frontend with Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/yourusername/ai_robotics_book&project-name=ai-robotics-frontend&root-directory=frontend)

### Deploy Backend (FastAPI)
[![Deploy Backend with Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/yourusername/ai_robotics_book&project-name=ai-robotics-backend&root-directory=backend)

## Features

- **Interactive Textbook** - Modern Docusaurus-based documentation
- **RAG Chatbot** - AI-powered Q&A using OpenAI Agents SDK and Gemini
- **Chapter Personalization** - Adaptive learning with LiteLLM/Groq agents
- **Urdu Translation** - Technical translation with perfect terminology
- **Vector Search** - Qdrant-powered semantic search
- **Session Management** - Neon Postgres for chat history

## Architecture

```
Frontend (Docusaurus + React)
    ↓
Backend (FastAPI + OpenAI Agents SDK)
    ↓
Qdrant (Vector Store) + Neon (PostgreSQL) + Gemini/Groq (LLMs)
```

## Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x
- **UI**: React 18+, TypeScript
- **Deployment**: Vercel

### Backend
- **Framework**: FastAPI (Python 3.11+)
- **AI**: OpenAI Agents SDK, LiteLLM
- **LLMs**: Gemini 2.0 Flash, Groq (llama3-70b)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres
- **Deployment**: Vercel (Serverless)

## Local Development

### Prerequisites
- Node.js 18+
- Python 3.11+
- Git

### Frontend Setup

```bash
cd frontend
npm install
npm start
```

Frontend runs at http://localhost:3000

### Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Backend runs at http://localhost:8000

API docs at http://localhost:8000/docs

### Environment Variables

#### Backend (.env)
```bash
# Gemini API
GEMINI_API_KEY=your_gemini_api_key
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
MODEL_NAME=gemini-2.0-flash-exp

# Groq API (for personalization/translation)
GROQ_API_KEY=your_groq_api_key

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_vectors

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:password@host/db

# Application
ENVIRONMENT=development
DEBUG=True
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

#### Frontend (.env)
```bash
REACT_APP_API_URL=http://localhost:8000
REACT_APP_ENABLE_RAG=true
```

## Vercel Deployment

### Backend Deployment Steps

1. **Push to GitHub** (if not already)
   ```bash
   git add .
   git commit -m "Add Vercel backend deployment config"
   git push
   ```

2. **Click Deploy Button** (above) or manual setup:
   - Go to [Vercel Dashboard](https://vercel.com/dashboard)
   - Click "New Project"
   - Import your GitHub repository
   - Set **Root Directory** to `backend`
   - Configure environment variables (see Backend .env above)
   - Deploy!

3. **Set Environment Variables in Vercel**
   - Go to Project Settings → Environment Variables
   - Add all variables from Backend .env section
   - Mark sensitive keys as "Sensitive"

4. **Verify Deployment**
   ```bash
   curl https://your-backend.vercel.app/api/health
   ```

### Frontend Deployment Steps

1. **Click Deploy Button** (above) or manual setup:
   - Go to [Vercel Dashboard](https://vercel.com/dashboard)
   - Click "New Project"
   - Import your GitHub repository
   - Set **Root Directory** to `frontend`
   - Set **Framework Preset** to "Docusaurus"
   - Add environment variable: `REACT_APP_API_URL=https://your-backend.vercel.app`
   - Deploy!

2. **Update CORS in Backend**
   - Add your frontend URL to `CORS_ORIGINS` in backend env vars
   - Example: `CORS_ORIGINS=https://your-frontend.vercel.app`
   - Redeploy backend

## One-Click Batch Deployment

Use the provided batch file for quick deployment (Windows):

```bash
deploy-vercel.bat
```

This will:
1. Install Vercel CLI
2. Login to Vercel
3. Deploy frontend to production

## API Endpoints

### POST /api/query
Answer questions using RAG

### POST /api/personalize
Personalize chapter content to learning style

### POST /api/translate
Translate content to technical Urdu

### POST /api/ingest
Ingest textbook content (admin)

### GET /api/health
Health check

See [Backend README](./backend/README.md) for detailed API documentation.

## Project Structure

```
ai_robotics_book/
├── frontend/           # Docusaurus frontend
│   ├── docs/          # Textbook content (Markdown)
│   ├── src/           # React components
│   └── vercel.json    # Vercel config
├── backend/           # FastAPI backend
│   ├── src/           # Source code
│   │   ├── api/       # API endpoints
│   │   ├── services/  # Business logic
│   │   ├── models/    # Data models
│   │   └── ai_agents/ # LiteLLM agents
│   ├── api/           # Vercel serverless handler
│   ├── requirements.txt
│   └── vercel.json    # Vercel config
├── specs/             # Spec-Driven Development artifacts
├── history/           # PHRs and ADRs
└── README.md          # This file
```

## Key Features

### 1. RAG Chatbot
- Book-only retrieval with citations
- Session persistence
- Selected-text query support
- Gemini 2.0 Flash via OpenAI Agents SDK

### 2. Chapter Personalization
- Adaptive learning styles (visual, practical, theoretical)
- Interest-based content (ML, robotics, hardware)
- Goal-oriented (quick overview, deep dive, project-based)
- LiteLLM with Groq llama3-70b

### 3. Urdu Translation
- Technical terminology preservation
- Contextual translation
- Markdown formatting retention
- Per-chapter translation button

## Troubleshooting

### Backend Issues

**Import errors in Vercel**:
- Check `api/index.py` path handling
- Verify all dependencies in `requirements.txt`

**API timeouts**:
- Increase `maxDuration` in `vercel.json` (max 60s for Pro)
- Optimize vector search queries
- Use smaller embedding models

**Environment variable issues**:
- Verify all required env vars are set in Vercel dashboard
- Check variable names match exactly (case-sensitive)

### Frontend Issues

**CORS errors**:
- Update `CORS_ORIGINS` in backend to include frontend URL
- Redeploy backend after changes

**Build failures**:
- Clear node_modules and reinstall: `rm -rf node_modules && npm install`
- Check Node.js version (18+)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Follow Spec-Driven Development workflow (see `.specify/`)
4. Create PHR for all work
5. Submit pull request

## License

MIT License - See LICENSE file for details

## Support

- GitHub Issues: [Create issue](https://github.com/yourusername/ai_robotics_book/issues)
- Backend API Docs: https://your-backend.vercel.app/docs
- Frontend: https://your-frontend.vercel.app

## Acknowledgments

Built with:
- Docusaurus (Meta)
- FastAPI (Tiangolo)
- OpenAI Agents SDK
- LiteLLM
- Gemini AI (Google)
- Groq
- Qdrant
- Neon
- Vercel
