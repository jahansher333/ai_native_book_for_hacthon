---
id: 0001
title: Urdu Translation Planning
stage: plan
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 006-urdu-translation
branch: 006-urdu-translation
user: system
command: /sp.plan
labels: ["architecture", "planning", "urdu", "translation", "i18n", "accessibility"]
links:
  spec: specs/006-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-urdu-translation/research.md
  - specs/006-urdu-translation/data-model.md
  - specs/006-urdu-translation/contracts/translate-api.yaml
  - specs/006-urdu-translation/quickstart.md
  - specs/006-urdu-translation/plan.md
  - CLAUDE.md
tests:
  - Manual constitution compliance check (all principles passed)
  - Technical context validation (all NEEDS CLARIFICATION resolved)
---

## Prompt

Execute `/sp.plan` command to generate implementation plan for feature 006-urdu-translation.

**Context**: User requested Urdu translation feature via `/sp.specify` in previous session:
```
Feature: Per-Chapter "اردو میں پڑھیں" Button

User Stories:
- As a logged-in user (or even guest), I see a button "اردو میں پڑھیں" at the top of every chapter
- When I click → entire chapter instantly becomes perfect technical Urdu
- Urdu is natural, readable, and technically accurate
- Uses @urdu-translator subagent

Requirements:
- Button visible to everyone (no login needed for Urdu)
- On click → send chapter text → @urdu-translator → replace content
- Loading spinner + badge "اردو میں دکھایا جا رہا ہے"
- Toggle back to English with "English میں واپس"
- Works on all chapters
```

**Input Files**:
- specs/006-urdu-translation/spec.md (feature specification - validated ✅)
- .specify/memory/constitution.md (project principles)

**Command**: `/sp.plan`

**Expected Outputs**:
- research.md (technical decisions)
- data-model.md (entity definitions)
- contracts/translate-api.yaml (API specification)
- quickstart.md (developer guide)
- plan.md (complete architecture)

## Response snapshot

**Phase 0: Research** ✅ Completed

Generated `research.md` with 8 research questions answered:
1. **@urdu-translator Architecture**: Follow OpenAI Agents SDK pattern (same as personalization agent)
2. **RTL Support**: Use CSS `direction: rtl` for Urdu content, keep code blocks LTR
3. **Technical Term Strategy**: 3-tier glossary (full translation / transliteration / hybrid)
4. **Markdown Preservation**: Regex validation (heading count, code blocks, links)
5. **Component Reuse**: Create separate Urdu components (different auth requirements vs personalization)
6. **Caching Strategy**: localStorage with 7-day TTL, chapter-based cache keys
7. **Error Handling**: Bilingual error messages (English + Urdu)
8. **Loading State**: Simple spinner with bilingual text, no progress bar

**Key Technology Decisions**:
- Backend: Python 3.11, FastAPI, OpenAI Agents SDK, Gemini 2.0 Flash API
- Frontend: TypeScript/React, DOMPurify, localStorage caching
- Translation: Gemini API (justified over DeepL/NLLB - better robotics domain knowledge)
- RTL: Browser-native CSS `direction: rtl`

---

**Phase 1: Design & Contracts** ✅ Completed

Generated `data-model.md` with 7 entities:
1. TranslationRequest (API input)
2. TranslationResponse (API output)
3. TranslationCacheEntry (localStorage structure)
4. TranslationState (React state machine)
5. TranslationError (bilingual error codes)
6. UrduSkillInput (backend skill input)
7. UrduSkillOutput (backend skill output)

Generated `contracts/translate-api.yaml` (OpenAPI 3.0 spec):
- POST /api/translate/chapter (main translation endpoint)
- GET /api/translate/health (health check)
- Request/response schemas with validation rules
- Error responses (422, 429, 500, 504)
- Example requests and responses

Generated `quickstart.md` (developer guide):
- Setup instructions (backend + frontend)
- Gemini API key configuration
- Complete implementation code samples for all 9 tasks
- Testing checklist (backend curl tests, frontend E2E manual tests)
- Troubleshooting guide (quota errors, timeout, markdown issues)

---

**Phase 1: Architecture Plan** ✅ Completed

Generated `plan.md` with complete architecture:

**Constitution Compliance**:
- ✅ All 11 principles evaluated
- ⚠️ 1 justified deviation: No authentication required (Principle VIII) - justified under Principle X (accessibility)
- ✅ Overall: PASS

**Technical Context**:
- Language: Python 3.11, TypeScript 4.9+, React 18
- Dependencies: FastAPI, OpenAI Agents SDK, DOMPurify
- Storage: localStorage only (no backend persistence)
- Performance: < 10s translation, < 1s cached load
- Scale: ~50-100 chapters, low concurrency

**Architecture Decisions**:
1. Separate Urdu components (not reusing Personalize components)
2. Frontend-only caching (backend stateless)
3. Gemini API via OpenAI-compatible endpoint
4. Agent pattern: urdu_translator_agent.py + translate_to_urdu_skill.py
5. Three-tier technical glossary for translation accuracy

**Project Structure**:
- Backend: `src/agents/`, `src/api/translate.py`
- Frontend: `src/components/Urdu/`, `src/hooks/useUrduTranslation.ts`, `src/services/urduTranslationService.ts`
- Integration: Docusaurus theme swizzle (already done for personalization)

**Risk Assessment**:
- HIGH: Gemini API quota exhaustion (mitigation: 7-day cache)
- MEDIUM: Translation quality (mitigation: glossary + validation)
- MEDIUM: Large chapter timeout (mitigation: 50K char limit)

**Success Metrics**:
- API latency: p95 < 10 seconds
- Cache hit rate: > 60%
- Markdown validation: 100% pass rate
- XSS vulnerability: 0 (DOMPurify)

---

**Agent Context Update** ✅ Completed

Ran `.specify/scripts/bash/update-agent-context.sh claude` to update CLAUDE.md with new feature information.

## Outcome

- ✅ Impact: Complete implementation plan generated for Urdu translation feature. All NEEDS CLARIFICATION items resolved. Constitution compliance verified (all gates passed).
- 🧪 Tests: Constitution compliance check (11/11 passed with 1 justified deviation), technical context validation (all resolved), OpenAPI spec generated
- 📁 Files: 5 artifacts generated (research.md, data-model.md, contracts/translate-api.yaml, quickstart.md, plan.md) + CLAUDE.md updated
- 🔁 Next prompts: Run `/sp.tasks` to generate task breakdown for implementation
- 🧠 Reflection: Planning phase successfully completed all required artifacts. Architecture leverages existing personalization infrastructure while maintaining separation of concerns (no auth required for Urdu vs auth required for personalization). Key innovation is three-tier technical glossary approach for translation quality.

## Evaluation notes (flywheel)

- Failure modes observed: None. All constitution gates passed. One intentional deviation (no authentication) was properly justified and documented.
- Graders run and results (PASS/FAIL): Constitution Check PASS (11/11 principles), Technical Context PASS (all NEEDS CLARIFICATION resolved), Markdown Structure Validation PASS (regex patterns defined)
- Prompt variant (if applicable): Standard /sp.plan workflow followed exactly per template
- Next experiment (smallest change to try): Consider adding backend Redis caching layer for shared translations (out of scope for MVP, future enhancement)
