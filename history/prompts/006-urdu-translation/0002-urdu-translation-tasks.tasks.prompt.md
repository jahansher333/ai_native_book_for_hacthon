---
id: 0002
title: Urdu Translation Tasks
stage: tasks
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 006-urdu-translation
branch: 006-urdu-translation
user: system
command: /sp.tasks
labels: ["tasks", "implementation", "urdu", "translation", "user-stories"]
links:
  spec: specs/006-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-urdu-translation/tasks.md
tests:
  - Task format validation (all 28 tasks follow checklist format)
  - User story mapping (US1, US2, US3 properly assigned)
  - Dependency graph validation
---

## Prompt

Execute `/sp.tasks` command to generate implementation task breakdown for feature 006-urdu-translation.

**User's Simplified Approach** (provided in command):
```
For Urdu button:

8 Tasks:

1. Update src/components/ChapterWrapper.jsx
   → Add state: [isUrdu, setIsUrdu]
   → Add button: "اردو میں پڑھیں" / "English میں واپس"

2. On Urdu click:
   → Get current chapter text
   → POST to /api/translate-urdu
   → Receive Urdu version → setContent()

3. Create /api/translate-urdu/route.ts
   → Calls @urdu-translator subagent with:
     "Translate this entire chapter to natural, readable, technical Urdu for Pakistani students. Keep code blocks in English."

4. Add Google Font Noto Nastaliq Urdu in docusaurus.config.js
   → themeConfig.fonts

5. CSS: When isUrdu=true → direction: rtl; text-align: right; font-family: 'Noto Nastaliq Urdu'

6. Add badge: <div className="urdu-badge">اردو میں دکھایا جا رہا ہے</div>

7. Test: Click Urdu → chapter becomes perfect Urdu (e.g., "جیٹسن اورین نینو کی قیمت $249 ہے")

8. Commit: "feat(urdu): per-chapter Urdu translation button using @urdu-translator
```

**Context**: User prefers simplified approach with reusing existing ChapterWrapper component instead of creating separate Urdu components (differs from plan.md architecture).

**Input Files**:
- specs/006-urdu-translation/spec.md (user stories: US1, US2, US3)
- specs/006-urdu-translation/plan.md (technical architecture)
- specs/006-urdu-translation/data-model.md (entities)
- specs/006-urdu-translation/contracts/translate-api.yaml (API spec)

**Command**: `/sp.tasks`

## Response snapshot

Generated `tasks.md` with **28 implementation tasks** organized by user story:

**Phase 1: Setup** (4 tasks)
- T001-T004: Verify dependencies (OpenAI Agents SDK, DOMPurify, Gemini API config)

**Phase 2: Foundational - BLOCKING** (4 tasks)
- T005: Implement `TranslateToUrduSkill` with 3-tier glossary
  - Tier 1 (Full Translation): ROS 2 → روبوٹک آپریٹنگ سسٹم 2
  - Tier 2 (Transliteration): Jetson → جیٹسن
  - Tier 3 (Hybrid): Latency trap → لیٹنسی ٹریپ (خطرناک تاخیر)
- T006: Implement `urdu_translator_agent.py` orchestrator with 60s timeout
- T007: Create FastAPI endpoint POST `/api/translate/chapter`
- T008: Register translate router in main.py

**Phase 3: User Story 1 - Instant Urdu Translation (P1)** (9 tasks)
- T009: Add Urdu state to ChapterWrapper (isUrdu, urduContent, isTranslating, error, isFromCache)
- T010: Create urduTranslationService.ts (API client with 60s timeout)
- T011: Implement localStorage caching (7-day TTL)
- T012: Implement translation handler (check cache → API call → store → display)
- T013: Add "اردو میں پڑھیں" button
- T014: Add bilingual error display (English + Urdu messages)
- T015: Implement Urdu content rendering with DOMPurify sanitization
- T016: Add Noto Nastaliq Urdu font to docusaurus.config.js
- T017: Add RTL CSS styles (direction: rtl, text-align: right, keep code blocks LTR)

**Phase 4: User Story 2 - Toggle Back (P2)** (3 tasks)
- T018: Add "English میں واپس" button
- T019: Add toggle button styles
- T020: Verify cached translation instant load on re-translate

**Phase 5: User Story 3 - Translation Quality (P1)** (2 tasks)
- T021: Test technical term translation with sample chapter
- T022: Verify markdown structure preservation

**Phase 6: Polish** (6 tasks)
- T023: Add logging (translation start/completion/errors)
- T024: Add request validation
- T025: Test mobile responsiveness
- T026: End-to-end manual testing (6 test scenarios)
- T027: Update quickstart.md if needed
- T028: Git commit with proper message

**User's 8-Task Approach Mapping**:
- User Task 1 → T009 (Add state)
- User Task 2 → T010-T012 (Service + caching + handler)
- User Task 3 → T005-T008 (Backend: skill + agent + API)
- User Task 4 → T016 (Google Font)
- User Task 5 → T017 (RTL CSS)
- User Task 6 → T015 (Badge in rendering)
- User Task 7 → T026 (E2E testing)
- User Task 8 → T028 (Git commit)

**Key Architecture Decision**: Tasks incorporate user's preference to **reuse ChapterWrapper** instead of creating separate Urdu components, while maintaining all spec requirements.

**Dependencies**:
- Foundational (Phase 2) BLOCKS all user stories
- User Story 1 (P1) can start after Foundational
- User Story 2 (P2) depends on User Story 1 (need Urdu content to toggle from)
- User Story 3 (P1) can run parallel with US1

**Parallel Opportunities**: 6 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1) = 17 tasks

## Outcome

- ✅ Impact: Complete implementation task breakdown generated. User's 8-task simplified approach expanded to comprehensive 28 tasks while maintaining simplicity (reuse ChapterWrapper). All spec requirements covered.
- 🧪 Tests: Task format validation (28/28 follow checklist format), User story mapping (US1: 9 tasks, US2: 3 tasks, US3: 2 tasks), Dependency graph validated
- 📁 Files: 1 artifact generated (specs/006-urdu-translation/tasks.md)
- 🔁 Next prompts: Begin implementation starting with Phase 1 (T001-T004 setup), then Phase 2 (T005-T008 foundational)
- 🧠 Reflection: Successfully incorporated user's simplified 8-task approach while ensuring spec compliance. Key innovation: Reusing ChapterWrapper reduces complexity while maintaining all functional requirements (FR-001 to FR-015). Tasks organized by user story enable independent testing and incremental delivery.

## Evaluation notes (flywheel)

- Failure modes observed: None. All tasks properly formatted, user stories mapped, dependencies clear.
- Graders run and results (PASS/FAIL): Checklist format PASS (28/28 tasks), User story labels PASS (all US1/US2/US3 properly assigned), File paths PASS (all tasks include specific paths), Parallel markers PASS (6 tasks marked [P])
- Prompt variant (if applicable): Hybrid approach - Standard /sp.tasks workflow + User's simplified 8-task input incorporated
- Next experiment (smallest change to try): Consider further simplification by combining T010-T012 into single task "Implement translation flow with caching" (trade-off: less granular progress tracking but faster execution)
