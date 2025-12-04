---
id: 0001
title: RAG Chatbot Feature Specification
stage: spec
date: 2025-12-04
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot
branch: 001-docusaurus-textbook
user: jahansher333
command: /sp.specify
labels: ["rag", "chatbot", "openai", "qdrant", "fastapi", "chatkit"]
links:
  spec: D:/New folder (6)/ai_robotics_book/specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/spec.md
tests:
  - Specification completeness check (PASS)
---

## Prompt

Feature: Integrated RAG Chatbot for Physical AI Textbook

User Stories:
- As a reader, I can open a chatbot widget on any book page to ask questions about the content (e.g., "What is ROS 2?"), and get accurate answers from the 4 modules/hardware sections only, with sources cited.
- As a user, I can select text on a page (e.g., hardware table) and query the bot about just that selection (e.g., "Explain this Jetson kit"), so it uses only selected context for precise responses.
- As a deployer, the chatbot embeds seamlessly in Docusaurus via ChatKit widget, with backend on FastAPI serving RAG via OpenAI Agents SDK, without slowing the static site.

Functional Requirements:
- Backend: FastAPI app with /ingest (chunk MDX files from /docs/, embed with OpenAI text-embedding-3-small, store in Qdrant Cloud Free Tier collection "book_vectors"; use Neon Postgres for chat sessions/history via pgvector for metadata).
- RAG Pipeline: OpenAI Agents SDK for agentic retrieval (query → embed → search Qdrant top-5 chunks → augment prompt → generate with gpt-4o-mini); handle selected-text mode by passing extra context param.
- Frontend: Embed OpenAI ChatKit.js widget in Docusaurus (swizzle Footer or plugin inject script); capture selected text via JS (window.getSelection()), send to backend as /query?text=selected&question=msg.
- Ingestion: Script to load book content (intro + 4 modules: ROS2, Gazebo/Unity, Isaac, VLA/capstone; hardware $700 kit table, latency warnings) into Qdrant (chunk size 1000, overlap 200).
- Security: API keys in env; Neon/Qdrant free tier; answers book-only (prompt: "Answer only from provided context, no external knowledge").
- Deploy: Backend on Render/Vercel; widget loads async, no paywalls.

Non-Functional Requirements:
- Accurate: 100% from book (e.g., "$249 Jetson Orin Nano, 40 TOPS"); cite sources (chunk IDs).
- Scalable: Qdrant free tier (1M vectors); Neon for sessions (serverless autoscaling).
- Selected Text: Frontend JS hook on button "Ask about selection" to grab text and query.
- Constitution Obey: Sim-to-real emphasis in prompts; Urdu/personalization hooks (future: query param for lang/profile).

Goal: Embed production-ready RAG bot in book site, unlocking 30 base + 10 bonus points.

## Response snapshot

Created comprehensive specification document (647 lines) for RAG chatbot feature integration.

**Key Components Specified**:

1. **User Stories (3)**:
   - US1: General Question Answering (P1) - 8 acceptance criteria
   - US2: Selected Text Contextual Queries (P1) - 7 acceptance criteria
   - US3: Seamless Docusaurus Integration (P2) - 8 acceptance criteria

2. **Functional Requirements (6)**:
   - FR-001: Backend RAG Pipeline (FastAPI + /ingest + /query + /history endpoints)
   - FR-002: Frontend ChatKit Integration (async widget + selection capture)
   - FR-003: Vector Database Setup (Qdrant Cloud schema with metadata)
   - FR-004: Session Storage (Neon Postgres schema with chat_sessions/messages tables)
   - FR-005: Content Ingestion Script (Python script with chunking + embedding)
   - FR-006: OpenAI Agents SDK Integration (agentic retrieval with function tools)

3. **Non-Functional Requirements (6)**:
   - NFR-001: 95%+ answer accuracy (textbook-only, source citations)
   - NFR-002: <3s p95 response latency
   - NFR-003: Free-tier scalability (Qdrant 1M vectors, Neon 0.5GB)
   - NFR-004: Security (env vars, CORS, HTTPS, input validation)
   - NFR-005: Constitution compliance (sim-to-real emphasis, accurate prices)
   - NFR-006: Maintainability (docs, versioning, logging)

4. **Technical Architecture**:
   - Backend: FastAPI + OpenAI Agents SDK + LangChain
   - Vector DB: Qdrant Cloud Free Tier (1M vectors, 1GB)
   - Relational DB: Neon Postgres Serverless (0.5GB)
   - Frontend: OpenAI ChatKit.js widget
   - Deployment: Render Free Tier (750 hours/month)
   - Models: text-embedding-3-small (embeddings) + gpt-4o-mini (generation)

5. **Success Criteria**:
   - 100% accuracy on 50 gold Q&A test set
   - <3s response latency (p95)
   - 100% source citation rate
   - <100ms widget load impact
   - Free tier usage within limits

6. **Deliverables**:
   - 50-question gold Q&A evaluation set
   - Technical architecture Mermaid diagram
   - Risk mitigation matrix (8 risks)
   - 20-item acceptance checklist
   - 13-point Definition of Done
   - Ingestion script example code
   - OpenAI Agents SDK integration example

**Out of Scope** (10 items):
- Multi-language support, voice interface, user accounts, code execution, image understanding, personalized learning, homework help, real-time collaboration, analytics dashboard, external links

## Outcome

- ✅ Impact: Comprehensive 647-line specification document defining complete RAG chatbot feature with 3 user stories, 6 functional requirements, 6 non-functional requirements, technical architecture, success criteria, and 50-question gold Q&A set
- 🧪 Tests: Specification completeness validated (all sections present: user stories, requirements, success criteria, out-of-scope, risks, acceptance checklist)
- 📁 Files: Created specs/002-rag-chatbot/spec.md (647 lines)
- 🔁 Next prompts: Run /sp.plan to design technical implementation architecture; run /sp.tasks to break down into executable tasks
- 🧠 Reflection: Specification balances free-tier constraints (Qdrant 1M vectors, Neon 0.5GB, Render 750 hours) with production-ready features (selected text queries, source citations, <3s latency); gold Q&A set provides clear acceptance criteria for 95%+ accuracy target

## Evaluation notes (flywheel)

- Failure modes observed: None - specification successfully created with all required sections
- Graders run and results (PASS/FAIL): Completeness check - PASS (user stories, functional/non-functional requirements, success criteria, out-of-scope, risks, acceptance checklist all present)
- Prompt variant (if applicable): Direct user request with detailed technical requirements (FastAPI, Qdrant, Neon, ChatKit, OpenAI Agents SDK)
- Next experiment (smallest change to try): Create technical plan (/sp.plan) to design backend architecture, database schemas, and deployment workflow
