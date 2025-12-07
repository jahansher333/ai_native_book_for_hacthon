# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (technical architecture), spec.md (user stories), research.md (decisions), data-model.md (schemas), contracts/ (API specs), quickstart.md (setup guide)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/` (FastAPI), `frontend/` (Docusaurus)
- All paths are absolute from repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment configuration

- [ ] T001 Create backend directory structure: `backend/src/{api,services,models,utils}`, `backend/tests/{unit,integration,contract}`, `backend/scripts`
- [ ] T002 Create `backend/requirements.txt` with dependencies: `fastapi==0.104.1`, `uvicorn[standard]==0.24.0`, `openai==1.3.8`, `qdrant-client==1.6.9`, `psycopg[binary]==3.1.15`, `langchain==0.1.0`, `python-dotenv==1.0.0`, `pydantic==2.5.2`
- [ ] T003 [P] Create `backend/.env.example` template with: `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `DATABASE_URL`, `ADMIN_API_KEY`, `ENVIRONMENT`
- [ ] T004 [P] Install Python dependencies: `cd backend && python -m venv venv && source venv/bin/activate && pip install -r requirements.txt`
- [ ] T005 [P] Add `.gitignore` entries: `backend/.env`, `backend/venv/`, `backend/__pycache__/`, `**/*.pyc`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until database setup (Qdrant + Neon) is complete

### Database Setup

- [ ] T006 [P] Create `backend/scripts/init_qdrant.py`: Initialize Qdrant collection `textbook_embeddings` with schema: 1536-dim vectors, COSINE distance, HNSW config (m=32, ef_construct=200, ef=128)
- [ ] T007 [P] Create `backend/scripts/init_postgres.py`: Initialize Neon Postgres schema with tables: `chat_sessions` (id UUID, created_at, last_activity, expires_at, metadata JSONB), `chat_messages` (id SERIAL, session_id FK, role CHECK, content TEXT, sources JSONB, created_at); create indexes: `idx_sessions_expires_at`, `idx_messages_session`
- [ ] T008 Run Qdrant initialization: `python backend/scripts/init_qdrant.py` (verify collection created with correct schema)
- [ ] T009 Run Postgres initialization: `python backend/scripts/init_postgres.py` (verify tables and indexes created)

### Core Backend Infrastructure

- [ ] T010 Create `backend/src/models/chunks.py`: Pydantic model `TextbookChunk` with fields: id, text, module, chapter, section, source_url, chunk_index, created_at
- [ ] T011 [P] Create `backend/src/models/queries.py`: Pydantic models `QueryRequest` (question, selected_text, session_id, module_filter) and `QueryResponse` (answer, sources, confidence, processing_time_ms)
- [ ] T012 [P] Create `backend/src/models/sessions.py`: Pydantic models `ChatSession` (id UUID, created_at, last_activity, expires_at, metadata) and `ChatMessage` (id, session_id, role, content, sources, created_at), `Source` (title, url, chunk_id, score)
- [ ] T013 Create `backend/src/utils/mdx_parser.py`: Function `strip_mdx_syntax(text: str) -> str` that removes JSX/MDX imports, exports, components using regex patterns from research.md
- [ ] T014 [P] Create `backend/src/utils/chunking.py`: Class `MDXTextSplitter` with `RecursiveCharacterTextSplitter` (separators: `["\n## ", "\n### ", "\n\n", "\n", " ", ""]`, chunk_size=1024, chunk_overlap=256); method `split_mdx_file()` returns list of chunk dicts

**Checkpoint**: Foundation ready - Qdrant and Neon databases initialized, core models defined, text processing utilities ready

---

## Phase 3: User Story 1 - General Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask natural language questions about textbook content and receive accurate answers with source citations within 3 seconds

**Independent Test**:
1. Ingest textbook content to Qdrant (450 chunks)
2. Start backend: `uvicorn backend.src.main:app --reload`
3. Send query: `curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d '{"question": "What is ROS 2?", "selected_text": null, "session_id": "test-uuid"}'`
4. Verify response: answer contains "ROS 2" content, sources array populated with module/chapter citations, confidence >0.7

### Implementation for User Story 1

- [ ] T015 [US1] Create `backend/src/services/embeddings.py`: Class `EmbeddingService` with method `embed_text(text: str) -> List[float]` using OpenAI `text-embedding-3-small` model (import openai, handle API errors)
- [ ] T016 [US1] Create `backend/src/services/vector_store.py`: Class `VectorStoreService` with methods: `search(query_embedding: List[float], limit: int = 5, module_filter: str = "all") -> List[dict]` using Qdrant client search with score_threshold=0.7; `upsert_chunk(chunk: TextbookChunk, embedding: List[float])`
- [ ] T017 [US1] Create `backend/src/services/session_store.py`: Class `SessionStoreService` with async methods: `create_session() -> UUID`, `get_session(session_id: UUID) -> ChatSession`, `add_message(session_id: UUID, role: str, content: str, sources: List[dict])`, `get_history(session_id: UUID, limit: int = 20) -> List[ChatMessage]`, `update_activity(session_id: UUID)`
- [ ] T018 [US1] Create `backend/scripts/ingest_textbook.py`: Load MDX files from `../../frontend/docs/`, use `MDXTextSplitter` to chunk, generate embeddings with `EmbeddingService`, upsert to Qdrant with `VectorStoreService`; batch upsert every 50 chunks; print progress (Processing: {filename}, Uploaded {count} chunks)
- [ ] T019 [US1] Run ingestion script: `python backend/scripts/ingest_textbook.py` (verify 450 chunks ingested, ~2-3 min processing time)
- [ ] T020 [US1] Create `backend/src/services/agent.py`: Function `run_rag_agent(question: str, selected_text: str, session_id: UUID) -> dict` implementing OpenAI function calling pattern from research.md section 1 (define `search_textbook` tool schema, agentic loop with tool_choice="auto", handle tool_calls, return {answer, sources, confidence})
- [ ] T021 [US1] Create `backend/src/api/query.py`: FastAPI router with `POST /query` endpoint using `QueryRequest`/`QueryResponse` models; call `run_rag_agent()`, store conversation with `SessionStoreService.add_message()`, update activity, return response; add system prompt: "Answer ONLY from textbook content. Cite sources as [Source: Module X - Title]. Refuse non-textbook questions politely."
- [ ] T022 [US1] Create `backend/src/api/history.py`: FastAPI router with `GET /history/{session_id}` endpoint returning last 20 messages; handle 404 if session not found; return `{"session_id": UUID, "messages": List[ChatMessage], "total_messages": int, "has_more": bool}`
- [ ] T023 [US1] Create `backend/src/main.py`: FastAPI app with CORS middleware (allow_origins: `["https://jahansher333.github.io", "http://localhost:3000"]`); include routers: `query`, `history`; root endpoint `GET /` returns `{"status": "ok", "message": "RAG Chatbot API"}`
- [ ] T024 [US1] Manual test - General questions: Start backend, test 5 questions from spec.md gold Q&A set (e.g., "What is ROS 2?", "How much does Jetson cost?"); verify answers cite correct modules, response <3s, non-textbook questions refused
- [ ] T025 [US1] Manual test - Chat history: Send 3 queries with same session_id, call `GET /history/{session_id}`, verify all messages returned in chronological order with sources

**Checkpoint**: Backend RAG query endpoint functional - users can ask questions and receive accurate cited answers from textbook content

---

## Phase 4: User Story 2 - Selected Text Contextual Queries (Priority: P1)

**Goal**: Enable readers to select text on any page and query the chatbot about that specific highlighted content

**Independent Test**:
1. Frontend running at `http://localhost:3000`
2. Select text on any docs page (e.g., hardware table row)
3. Open chat widget, verify "Ask about selection" button appears
4. Click button, type question about selection
5. Verify backend receives `selected_text` parameter and prioritizes it in answer

### Implementation for User Story 2

- [ ] T026 [US2] Install frontend dependencies: `cd frontend && npm install dompurify && npm install --save-dev @types/dompurify`
- [ ] T027 [US2] Create `frontend/src/utils/textSelection.ts`: Functions `getSelectedTextSafely() -> {text, module, chapter, isValid}` using `window.getSelection()`, DOMPurify sanitization (ALLOWED_TAGS: [], ALLOWED_ATTR: []), truncate to 2000 chars; `sanitizeForBackend(text: string) -> string`; `isSelectionSafe(text: string) -> boolean` checking for `<script`, `on*=`, `javascript:`, `<iframe`, `<object`, `<embed>` patterns (see research.md section 5 for full implementation)
- [ ] T028 [US2] Create `frontend/src/hooks/useTextSelection.ts`: React hook that listens to `mouseup` and `keyup` events, calls `getSelectedTextSafely()` and `isSelectionSafe()`, returns sanitized text via `useState`
- [ ] T029 [US2] Update `backend/src/services/agent.py`: Modify `run_rag_agent()` to check if `selected_text` provided; if yes, prepend to context with higher weight in search_textbook function parameters (add `selected_text` field to tool schema)
- [ ] T030 [US2] Update `backend/src/api/query.py`: Ensure `QueryRequest.selected_text` is passed to `run_rag_agent()`; log when selected_text is present for debugging
- [ ] T031 [US2] Manual test - Selected text queries: Select hardware table row "$249 Jetson Orin Nano", ask "Explain this hardware", verify answer focuses on selected context and cites Hardware Requirements source; test with empty selection (fallback to general query)

**Checkpoint**: Selected text feature working - users can highlight content and get focused answers about their selection

---

## Phase 5: User Story 3 - Seamless Docusaurus Integration (Priority: P2)

**Goal**: Embed chatbot widget seamlessly into Docusaurus site with async loading, theme support, and no performance impact

**Independent Test**:
1. Frontend dev server running: `npm start`
2. Open any docs page, verify chat button appears bottom-right
3. Click button, chat window opens 400×500px
4. Type question, verify API call to backend
5. Toggle Docusaurus dark/light theme, verify widget styling adapts
6. Check Lighthouse performance score, verify <100ms widget load impact

### Implementation for User Story 3

- [ ] T032 [US3] Create `frontend/src/components/ChatWidget/ChatWidgetClient.tsx`: React component with state for `[isOpen, messages, inputValue, isLoading]`; `handleSendMessage()` function calling `fetch('/api/query')` with question and `getSelectedText()`; message list with user/bot roles; input field with Enter key handler; loading indicator during API call (see research.md section 4 for full implementation ~200 lines)
- [ ] T033 [US3] Create `frontend/src/components/ChatWidget/index.tsx`: Wrapper component using dynamic import `dynamic(() => import('./ChatWidgetClient'), {ssr: false, loading: () => null})` to prevent SSR hydration errors
- [ ] T034 [US3] Create `frontend/src/components/ChatWidget/ChatWidget.module.css`: Styles for floating button (56×56px, bottom-right, z-index: 1000), chat window (400×500px, white background, box-shadow), message bubbles (user: blue right-aligned, bot: gray left-aligned), input container (flex, gap: 8px), mobile responsive (@media max-width: 768px: fullscreen) (see research.md section 4 for full styles ~150 lines)
- [ ] T035 [US3] Swizzle Docusaurus Root: `npm run swizzle @docusaurus/theme-classic Root -- --eject` or create `frontend/src/theme/Root.tsx`: Import `ChatWidget`, render alongside {children}
- [ ] T036 [US3] Update `frontend/docusaurus.config.js`: Set `baseUrl` for API proxy or update `ChatWidgetClient.tsx` to use `process.env.NODE_ENV === 'production' ? 'https://textbook-rag-backend.onrender.com' : 'http://localhost:8000'` for API_URL constant
- [ ] T037 [US3] Add data attributes to docs pages: Update MDX wrapper to include `<div data-module="..." data-chapter="...">` around content for context extraction in `getSelectedTextSafely()`
- [ ] T038 [US3] Manual test - Widget integration: Start frontend dev server, verify widget appears on all pages (intro, ROS 2, Gazebo, Isaac, VLA), test opening/closing, verify no console errors, test message send/receive
- [ ] T039 [US3] Manual test - Theme compatibility: Toggle dark/light theme in Docusaurus navbar, verify chat widget colors adapt (use CSS custom properties: `var(--ifm-color-primary)`, `var(--ifm-background-color)`), text remains readable
- [ ] T040 [US3] Manual test - Performance: Run Lighthouse audit on docs page with widget, verify Performance score >90, widget load time <100ms (check Network tab for async script loading)

**Checkpoint**: Chat widget fully integrated into Docusaurus - users can access chatbot on any page with seamless UX

---

## Phase 6: Deployment & Validation

**Purpose**: Deploy backend to production and validate end-to-end functionality

- [ ] T041 Create `backend/Dockerfile`: Multi-stage build with Python 3.11 base, install dependencies, copy src/, expose port 8000, CMD `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- [ ] T042 Create `backend/render.yaml`: Render.com config with service type: web, env: python, buildCommand: `pip install -r requirements.txt`, startCommand: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`, envVars from secret
- [ ] T043 [P] Deploy backend to Render Free Tier: Push code to GitHub, connect repo to Render, add environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, ADMIN_API_KEY), trigger deployment
- [ ] T044 [P] Update `frontend/src/components/ChatWidget/ChatWidgetClient.tsx`: Set production API_URL to Render backend URL (e.g., `https://textbook-rag-backend.onrender.com`)
- [ ] T045 Deploy frontend to GitHub Pages: `cd frontend && npm run build && npm run deploy`
- [ ] T046 Test production deployment: Visit `https://jahansher333.github.io/Ai_Native_Books_Pyhsical_Ai/`, open chat widget, send test questions, verify answers with sources, test selected text feature
- [ ] T047 Validate with gold Q&A set: Test 10 questions from spec.md Appendix A (e.g., "What is ROS 2?", "How much does Jetson cost?", "Why is cloud control dangerous?"), verify 100% accuracy, all answers cite correct sources
- [ ] T048 Setup session cleanup CRON: Connect to Neon Postgres, run `CREATE EXTENSION IF NOT EXISTS pg_cron; SELECT cron.schedule('cleanup-expired-sessions', '0 0 * * *', 'DELETE FROM chat_sessions WHERE expires_at < CURRENT_TIMESTAMP');` or configure external cleanup via GitHub Actions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T005) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (T006-T014) - Backend RAG query functionality
- **User Story 2 (Phase 4)**: Depends on User Story 1 (T015-T025) - Selected text feature extends query functionality
- **User Story 3 (Phase 5)**: Depends on User Story 1 backend (T015-T023) - Frontend widget integration
- **Deployment (Phase 6)**: Depends on all user stories being functional

### User Story Dependencies

- **User Story 1 (P1)**: Foundational → Ingestion → RAG Agent → Query API (T006-T025)
- **User Story 2 (P1)**: User Story 1 backend → Frontend text selection → Updated agent (T026-T031)
- **User Story 3 (P2)**: User Story 1 backend → Frontend widget → Integration (T032-T040)

### Critical Path

```
T001-T005 (Setup) →
T006-T014 (Foundational) →
T018-T019 (Ingestion) →
T020-T023 (RAG API) →
T024-T025 (US1 Tests) →
T032-T040 (US3 Widget) →
T041-T048 (Deployment)
```

### Parallel Opportunities

- **Setup Phase**: T003 (env template), T004 (pip install), T005 (gitignore) can run in parallel
- **Foundational Phase**: T006 (Qdrant init script) and T007 (Postgres init script) can run in parallel; T010, T011, T012 (Pydantic models) can run in parallel; T013 (MDX parser) and T014 (chunking) can run in parallel
- **US1 Implementation**: T015 (embeddings), T016 (vector store), T017 (session store) can run in parallel (different services)
- **US2 + US3**: Once US1 backend (T015-T023) is complete, US2 frontend (T026-T028) and US3 widget (T032-T036) can run in parallel
- **Deployment**: T043 (backend deploy) and T044 (frontend update) can start in parallel

---

## Parallel Example: Foundational Phase

```bash
# Launch model creation tasks together (different files):
Task T010: "Create TextbookChunk model in backend/src/models/chunks.py"
Task T011: "Create QueryRequest/QueryResponse models in backend/src/models/queries.py"
Task T012: "Create ChatSession/ChatMessage models in backend/src/models/sessions.py"

# Launch utility creation tasks together (different files):
Task T013: "Create MDX parser in backend/src/utils/mdx_parser.py"
Task T014: "Create MDXTextSplitter in backend/src/utils/chunking.py"
```

---

## Parallel Example: User Story 1 Services

```bash
# Launch service layer tasks together (different files):
Task T015: "Create EmbeddingService in backend/src/services/embeddings.py"
Task T016: "Create VectorStoreService in backend/src/services/vector_store.py"
Task T017: "Create SessionStoreService in backend/src/services/session_store.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T014) - CRITICAL
3. Complete Phase 3: User Story 1 (T015-T025)
4. **STOP and VALIDATE**: Test backend RAG queries independently
5. Deploy backend to Render (T041-T043)
6. Verify production API with curl

**MVP Scope**: Backend RAG API functional - can answer textbook questions with source citations

### Full Delivery (All User Stories)

1. MVP First (Setup + Foundational + US1)
2. Add Phase 4: User Story 2 (T026-T031) - Selected text feature
3. Add Phase 5: User Story 3 (T032-T040) - Frontend widget
4. Complete Phase 6: Deployment (T041-T048)
5. **VALIDATE**: Test complete end-to-end flow on production

**Full Scope**: Complete chatbot embedded in Docusaurus with selected text support

### Parallel Team Strategy

With 3 developers:

1. **Together**: Complete Setup (Phase 1) + Foundational (Phase 2)
2. **Split work after T014**:
   - **Developer A**: User Story 1 backend (T015-T023) - RAG agent and API
   - **Developer B**: User Story 2 frontend (T026-T028) - Text selection utils (can start in parallel with A)
   - **Developer C**: User Story 3 widget (T032-T036) - Chat widget component (can start in parallel with A)
3. **Integration**: Developer A finishes backend → B and C integrate their frontends
4. **Together**: Testing (T024-T025, T031, T038-T040) and Deployment (Phase 6)

---

## Notes

- **[P] tasks**: Different files, no shared dependencies - safe for parallel execution
- **[US1], [US2], [US3] labels**: Map tasks to user stories for traceability
- **Tests**: User requested specific test scenarios (curl /query, selected text query) - integrated as manual test tasks T024, T025, T031, T038-T040, T046, T047
- **Dependencies**: Foundational phase (T006-T014) MUST complete before any user story work
- **Ingestion CRITICAL**: T018-T019 must complete before RAG agent testing (T020-T025)
- **File paths**: All paths are absolute from repository root
- **Commit strategy**: Commit after each task or logical group (e.g., all Pydantic models T010-T012)
- **Checkpoints**: Stop at any checkpoint to validate story independently
- **Avoid**: Vague tasks, same-file conflicts, cross-story dependencies that break independence

---

## Total Task Count

- **Setup**: 5 tasks
- **Foundational**: 9 tasks
- **User Story 1**: 11 tasks
- **User Story 2**: 6 tasks
- **User Story 3**: 9 tasks
- **Deployment**: 8 tasks
- **TOTAL**: 48 tasks

**Parallel Opportunities**: 15+ tasks can run in parallel across phases

**MVP Scope**: 25 tasks (Setup + Foundational + US1)

**Suggested First Increment**: Complete T001-T025 (MVP backend) before frontend work
