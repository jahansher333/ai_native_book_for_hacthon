# Implementation Plan: Integrated RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-04 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

## Summary

Build an intelligent RAG (Retrieval-Augmented Generation) chatbot embedded in the Physical AI textbook website using FastAPI backend, OpenAI Agents SDK for agentic retrieval, Qdrant Cloud for vector storage, Neon Postgres for session management, and OpenAI ChatKit.js widget for frontend integration. The chatbot must answer questions exclusively from textbook content with source citations, support selected-text queries, and deploy within free-tier constraints (Qdrant 1M vectors, Neon 0.5GB, Render 750 hours/month).

**Primary Requirement**: Enable readers to ask natural language questions about textbook content and receive accurate, cited answers within 3 seconds, with seamless Docusaurus integration via async-loaded ChatKit widget.

**Technical Approach**: Implement three-tier architecture: (1) Ingestion pipeline using LangChain text splitter + OpenAI embeddings → Qdrant, (2) FastAPI backend with `/ingest`, `/query`, `/history` endpoints using OpenAI Agents SDK for retrieval-augmented generation, (3) ChatKit.js widget with custom selection capture injected via Docusaurus plugin.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 4.5+ (frontend Docusaurus plugin)
**Primary Dependencies**:
- Backend: FastAPI 0.104+, OpenAI Python SDK 1.3+, Qdrant Client 1.6+, psycopg2-binary 2.9+, LangChain 0.1+, OpenAI Agents SDK (beta)
- Frontend: OpenAI ChatKit.js (CDN), Docusaurus 3.0+, React 18+

**Storage**:
- Vector DB: Qdrant Cloud Free Tier (1M vectors, 1GB, cosine similarity, HNSW index)
- Relational DB: Neon Postgres Serverless Free Tier (0.5GB, pgvector extension for metadata queries)

**Testing**:
- Backend: pytest + pytest-asyncio for FastAPI endpoints, manual gold Q&A validation (50 questions)
- Frontend: Manual testing of widget load, selection capture, theme compatibility

**Target Platform**:
- Backend: Render Free Tier (750 hours/month, Linux container, auto-sleep after 15 min)
- Frontend: GitHub Pages (static site with async widget injection)

**Project Type**: Web application (separate backend API + frontend static site)

**Performance Goals**:
- Query response latency: <3s p95 (1s embedding + 1.5s retrieval + 0.5s generation)
- Widget load time: <100ms impact on page render (async script loading)
- Concurrent users: 50 simultaneous chats (free tier limit)

**Constraints**:
- MUST use only textbook content (no external knowledge in answers)
- MUST cite sources in format `[Source: Module X - Title]`
- MUST stay within free-tier limits: Qdrant 1M vectors, Neon 0.5GB, OpenAI $5/month budget (~1000 queries)
- MUST NOT block static site render (widget loads asynchronously)
- MUST support selected-text queries (capture via `window.getSelection()`)

**Scale/Scope**:
- ~450 textbook chunks (29 MDX files × ~15 chunks each at 1000 chars/chunk, 200 overlap)
- Estimated 1000 queries/month (20 concurrent students, 50 questions each)
- Chat session retention: 7 days (auto-cleanup via CRON)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle III: Accuracy & Technical Truth
**Status**: ✅ PASS
- All hardware references use verified Economy Jetson Kit pricing ($249 Jetson, $349 RealSense)
- Technical claims about RAG architecture cite OpenAI Agents SDK docs, Qdrant docs
- Free-tier limits verified: Qdrant (1M vectors/1GB), Neon (0.5GB), Render (750 hours/month)

### Principle IV: Sim-to-Real First Philosophy
**Status**: ✅ PASS (N/A for this feature)
- RAG chatbot is a web service, not robot control system
- Answers emphasize edge deployment for robot inference (constitutional compliance in content)

### Principle V: Cost Transparency
**Status**: ✅ PASS
- Spec explicitly lists free-tier options (Qdrant, Neon, Render)
- OpenAI cost estimation: $0.0001/query (embedding + generation) ≈ $0.10/1000 queries
- Total monthly cost for 1000 queries: ~$0.10 (well within $5 budget)

### Principle VII: RAG Chatbot is First-Class Citizen
**Status**: ✅ PASS
- This feature implements the constitutional requirement for embedded RAG chatbot
- Answers only from textbook content (enforced via system prompt)
- Selected-text queries supported (frontend JS capture + backend context injection)
- Full deployment to GitHub Pages with async widget integration

### Principle X: Open Source & Accessible Forever
**Status**: ✅ PASS
- Backend code will be MIT licensed
- No paywalls (chatbot accessible to all readers without authentication)
- Free-tier services ensure permanent availability

### Principle XI: Latency Trap Rule
**Status**: ✅ PASS
- Chatbot answers will cite textbook warnings about cloud latency for robot control
- System prompt includes instruction to emphasize edge deployment when answering ROS/VLA questions

**GATE RESULT**: ✅ ALL CHECKS PASS - Proceed to Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (OpenAI Agents SDK, Qdrant setup, Neon integration)
├── data-model.md        # Phase 1 output (Qdrant schema, Neon tables, chunk structure)
├── quickstart.md        # Phase 1 output (Setup instructions, deployment guide)
├── contracts/           # Phase 1 output (OpenAPI spec for FastAPI endpoints)
│   ├── ingest.yaml      # POST /ingest contract
│   ├── query.yaml       # POST /query contract
│   └── history.yaml     # GET /history/{session_id} contract
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                    # FastAPI app entry point
│   ├── api/
│   │   ├── ingest.py              # POST /ingest endpoint
│   │   ├── query.py               # POST /query endpoint (RAG logic)
│   │   └── history.py             # GET /history/{session_id} endpoint
│   ├── services/
│   │   ├── embeddings.py          # OpenAI text-embedding-3-small wrapper
│   │   ├── vector_store.py        # Qdrant client + search logic
│   │   ├── session_store.py       # Neon Postgres queries (sessions, messages)
│   │   └── agent.py               # OpenAI Agents SDK integration
│   ├── models/
│   │   ├── chunks.py              # Pydantic model for textbook chunks
│   │   ├── queries.py             # Pydantic models for /query request/response
│   │   └── sessions.py            # Pydantic models for chat sessions
│   └── utils/
│       ├── chunking.py            # LangChain text splitter wrapper
│       └── mdx_parser.py          # MDX file loader (strip JSX, preserve content)
├── tests/
│   ├── unit/
│   │   ├── test_chunking.py
│   │   ├── test_embeddings.py
│   │   └── test_vector_store.py
│   ├── integration/
│   │   ├── test_ingest_endpoint.py
│   │   └── test_query_endpoint.py
│   └── contract/
│       └── test_gold_qa.py        # 50-question gold Q&A validation
├── scripts/
│   ├── ingest_textbook.py         # CLI script to run ingestion
│   └── cleanup_sessions.py        # CRON job to delete old sessions
├── requirements.txt
├── Dockerfile
└── render.yaml                    # Render deployment config

frontend/
├── src/
│   ├── plugins/
│   │   └── chatkit-plugin/
│   │       ├── index.ts           # Docusaurus plugin entry
│   │       ├── client.ts          # Browser-side ChatKit initialization
│   │       └── selection.ts       # window.getSelection() capture logic
│   └── css/
│       └── chatkit-theme.css      # Theme overrides for ChatKit widget
└── docusaurus.config.js           # Plugin registration

.env.example                       # Template for API keys
README.md                          # Root-level quickstart
```

**Structure Decision**: Web application structure selected because we have:
1. **Backend API** (FastAPI): Separate server deployed to Render
2. **Frontend Static Site** (Docusaurus): Deployed to GitHub Pages
3. **Clear separation of concerns**: Backend handles RAG logic, frontend handles UI/widget

This structure aligns with Constitution Principle VII (RAG chatbot as first-class citizen) by making the backend a standalone, testable service that can be independently deployed and scaled.

## Complexity Tracking

No violations detected. This feature strictly adheres to constitutional principles:
- Uses free-tier services (Principle V: Cost Transparency)
- Answers only from textbook (Principle VII: RAG requirements)
- No paywalls (Principle X: Open Source)
- Cites latency warnings (Principle XI: Latency Trap Rule)

---

## Phase 0: Research & Unknowns Resolution

**Objective**: Resolve all "NEEDS CLARIFICATION" items from Technical Context and document best practices for chosen technologies.

### Research Tasks

1. **OpenAI Agents SDK Integration**:
   - Research: Official OpenAI Agents SDK documentation (beta) for function tools
   - Unknown: How to register `search_textbook()` as a function tool
   - Unknown: How to pass selected text as additional context to agent
   - Deliverable: Code snippet showing agent initialization with search tool

2. **Qdrant Cloud Free Tier Setup**:
   - Research: Qdrant Cloud dashboard signup, API key generation, collection creation
   - Unknown: How to create HNSW index with optimal parameters for 450 chunks
   - Unknown: Best practices for metadata filtering (e.g., filter by module/chapter)
   - Deliverable: Python script to initialize Qdrant collection with correct schema

3. **Neon Postgres Integration**:
   - Research: Neon console setup, connection string format, pgvector extension usage
   - Unknown: How to enable pgvector for metadata queries (optional)
   - Unknown: Best practices for session cleanup (CRON vs. TTL triggers)
   - Deliverable: SQL schema with indexes for fast session/message queries

4. **ChatKit.js Docusaurus Integration**:
   - Research: ChatKit.js documentation (if available) or alternative React chat widget (e.g., react-chatbot-kit)
   - Unknown: Does OpenAI provide official ChatKit.js library? (may need alternative)
   - Unknown: How to inject widget asynchronously without blocking Docusaurus SSR
   - Deliverable: Docusaurus plugin code snippet for widget injection

5. **Selected Text Capture Best Practices**:
   - Research: MDN documentation for `window.getSelection()`, cross-browser compatibility
   - Unknown: How to handle large selections (>2000 chars) and edge cases (no text selected)
   - Deliverable: TypeScript function to safely capture and sanitize selected text

6. **LangChain Text Splitting for MDX**:
   - Research: LangChain `RecursiveCharacterTextSplitter` documentation
   - Unknown: How to strip MDX syntax (JSX components) while preserving content
   - Unknown: Optimal chunk size/overlap for technical documentation
   - Deliverable: Python utility function to parse MDX and split into chunks

**Output**: `research.md` document with findings, code snippets, and decisions for all 6 research tasks.

---

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete (all unknowns resolved)

### 1. Data Model (`data-model.md`)

**Entities**:

1. **TextbookChunk** (Qdrant vector):
   - `id`: UUID (unique chunk identifier)
   - `vector`: float[1536] (OpenAI text-embedding-3-small)
   - `payload`:
     - `source`: string (e.g., "Module 1 - ROS 2 Fundamentals")
     - `chapter`: string (e.g., "nodes.md")
     - `text`: string (chunk content, max 1000 chars)
     - `url`: string (e.g., "/docs/ros2/nodes")
     - `chunk_index`: int (position in original document)

2. **ChatSession** (Neon Postgres):
   - `id`: UUID (primary key, gen_random_uuid())
   - `created_at`: timestamp (default NOW())
   - `last_activity`: timestamp (default NOW(), updated on each message)

3. **ChatMessage** (Neon Postgres):
   - `id`: serial (primary key)
   - `session_id`: UUID (foreign key → chat_sessions.id)
   - `role`: enum('user', 'assistant')
   - `content`: text (question or answer)
   - `sources`: jsonb (array of {title, url, chunk_id}, nullable for user messages)
   - `created_at`: timestamp (default NOW())

**Validation Rules**:
- Chunk text max 1000 chars (enforced by LangChain splitter)
- Session expires after 7 days inactivity (cleanup CRON)
- Max 100 messages per session (delete older)

**Relationships**:
- ChatSession 1:N ChatMessage (one session has many messages)
- TextbookChunk references source chapter (no foreign key, metadata only)

### 2. API Contracts (`/contracts/`)

**Endpoints**:

1. **POST /ingest** (Admin-only, run manually):
   - Input: None (reads from `frontend/docs/` directory)
   - Output: `{"status": "success", "chunks_ingested": 450}`
   - OpenAPI spec: `contracts/ingest.yaml`

2. **POST /query** (Public, rate-limited 10/min):
   - Input:
     ```json
     {
       "question": "What is ROS 2?",
       "selected_text": null,
       "session_id": "uuid-v4"
     }
     ```
   - Output:
     ```json
     {
       "answer": "ROS 2 is... [Source: Module 1]",
       "sources": [{"title": "...", "url": "...", "chunk_id": "..."}],
       "confidence": 0.92
     }
     ```
   - OpenAPI spec: `contracts/query.yaml`

3. **GET /history/{session_id}** (Public):
   - Input: URL param `session_id`
   - Output:
     ```json
     {
       "messages": [
         {"role": "user", "content": "...", "created_at": "..."},
         {"role": "assistant", "content": "...", "sources": [...], "created_at": "..."}
       ]
     }
     ```
   - OpenAPI spec: `contracts/history.yaml`

### 3. Quickstart Guide (`quickstart.md`)

**Sections**:
1. Prerequisites (Python 3.11, Node 18, API keys)
2. Backend Setup (clone, install deps, set env vars, run FastAPI)
3. Ingestion (run `python scripts/ingest_textbook.py`)
4. Frontend Setup (install Docusaurus plugin, configure widget)
5. Deployment (Render backend, GitHub Pages frontend)
6. Testing (gold Q&A validation, manual widget testing)

### 4. Agent Context Update

Run agent context script to add new technologies:
- FastAPI
- Qdrant Cloud
- Neon Postgres
- OpenAI Agents SDK
- LangChain
- ChatKit.js

**Command**:
```bash
.specify/scripts/bash/update-agent-context.sh claude
```

---

## Phase 2: Task Breakdown (Deferred to `/sp.tasks`)

Task generation is handled by `/sp.tasks` command, which will create `tasks.md` with:
- Task 1: Setup Qdrant Cloud collection
- Task 2: Setup Neon Postgres schema
- Task 3: Implement ingestion script
- Task 4: Implement FastAPI /query endpoint with OpenAI Agents SDK
- Task 5: Implement ChatKit widget Docusaurus plugin
- Task 6: Deploy backend to Render
- Task 7: Validate gold Q&A set (50 questions)
- Task 8: Deploy frontend to GitHub Pages

---

## Implementation Milestones

### Milestone 1: Backend Core (Week 1)
- ✅ Qdrant Cloud collection created
- ✅ Neon Postgres schema deployed
- ✅ Ingestion script working (450 chunks uploaded)
- ✅ FastAPI `/query` endpoint returns answers with sources

### Milestone 2: Frontend Integration (Week 2)
- ✅ ChatKit widget embedded in Docusaurus
- ✅ Selected text capture working
- ✅ Widget loads without blocking page render (<100ms)
- ✅ Dark/light theme compatibility verified

### Milestone 3: Validation & Deployment (Week 3)
- ✅ Gold Q&A set 50/50 correct (100% accuracy)
- ✅ Response latency <3s p95 (backend logs)
- ✅ Backend deployed to Render free tier
- ✅ Frontend deployed to GitHub Pages with working chatbot

---

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| OpenAI API quota exceeded | Rate limiting (10 queries/min), fallback to gpt-3.5-turbo |
| Qdrant free tier full (1M vectors) | Monitor usage, optimize chunk size, upgrade to paid ($25/month) if needed |
| Neon Postgres storage full (0.5GB) | Implement session cleanup CRON (delete >7 days old) |
| ChatKit.js library unavailable | Use alternative React chat widget (react-chatbot-kit, fallback plan) |
| Render backend cold start (5-10s) | Accept first-query delay or upgrade to paid tier ($7/month) |
| Inaccurate answers (hallucination) | Strong system prompt, log incorrect answers, iteratively improve retrieval |

---

## Next Steps

1. ✅ This plan is complete
2. ⏭ Run research tasks to generate `research.md`
3. ⏭ Generate `data-model.md`, `contracts/`, `quickstart.md`
4. ⏭ Update agent context with new technologies
5. ⏭ Run `/sp.tasks` to break down into executable tasks
6. ⏭ Run `/sp.implement` to build the RAG chatbot

**Branch**: 002-rag-chatbot (to be created)
**Plan Path**: D:/New folder (6)/ai_robotics_book/specs/002-rag-chatbot/plan.md
