# Tasks: Chapter Urdu Translation Button

**Input**: Design documents from `/specs/006-urdu-translation/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/translate-api.yaml

**Tests**: Not explicitly requested in feature specification - focusing on implementation tasks

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**User's Simplified Approach**: Reuse existing ChapterWrapper component, add Urdu toggle functionality

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Backend files: Python (FastAPI, OpenAI Agents SDK)
- Frontend files: TypeScript/React (Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Backend agent infrastructure initialization + frontend dependencies

- [ ] T001 Verify backend agents directory exists: `backend/src/agents/` and `backend/src/agents/skills/`
- [ ] T002 [P] Verify OpenAI Agents SDK is installed: Check `backend/requirements.txt` for `openai-agents`
- [ ] T003 [P] Verify DOMPurify is installed: Check `frontend/package.json` for `dompurify` and `@types/dompurify`
- [ ] T004 Verify Gemini API configuration in `backend/.env`: Ensure `GEMINI_API_KEY`, `BASE_URL`, and `MODEL_NAME` are set

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core @urdu-translator agent infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Implement `TranslateToUrduSkill` class in `backend/src/agents/skills/translate_to_urdu_skill.py`:
  - Create Skill class with `name="translateToUrduSkill"`
  - Implement `execute()` method signature: `async def execute(self, chapter_content: str, chapter_id: str) -> str`
  - Implement `_build_translation_prompt()` method with 3-tier technical glossary:
    - Tier 1 (Full Translation): ROS 2 → روبوٹک آپریٹنگ سسٹم 2, URDF → یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ
    - Tier 2 (Transliteration): Jetson → جیٹسن, NVIDIA → اینویڈیا
    - Tier 3 (Hybrid): Latency trap → لیٹنسی ٹریپ (خطرناک تاخیر)
  - Implement `_get_system_prompt()` method: "You are an expert technical translator specializing in robotics. Translate English to natural, readable Urdu. Preserve ALL markdown structure. Keep code blocks unchanged."
  - Implement `_validate_markdown_structure()` method to check:
    - Heading count (original vs translated)
    - Code block count (must match exactly)
    - Link count (±10% tolerance)
  - Configure OpenAI client with Gemini endpoint: `base_url=settings.base_url`, `api_key=settings.gemini_api_key`
  - Call `chat.completions.create()` with `model="gemini-2.0-flash"`, `temperature=0.7`, `max_tokens=8192`

- [ ] T006 Implement `urdu_translator_agent.py` orchestrator in `backend/src/agents/urdu_translator_agent.py`:
  - Import `Agent` and `Runner` from `openai_agents`
  - Import `TranslateToUrduSkill` from `skills.translate_to_urdu_skill`
  - Initialize OpenAI client: `AsyncOpenAI(api_key=settings.gemini_api_key, base_url=settings.base_url)`
  - Create skill instance: `translate_skill = TranslateToUrduSkill(client)`
  - Initialize agent: `Agent(name="Urdu Translator", instructions="You translate technical robotics content from English to Urdu.", skills=[translate_skill], model="gemini-2.0-flash")`
  - Implement `async def translate_chapter_content(original_content: str, chapter_id: str) -> str`
  - Use `asyncio.wait_for(runner.run(...), timeout=60.0)` for 60-second timeout
  - Return translated content or raise TimeoutError

- [ ] T007 Create FastAPI endpoint in `backend/src/api/translate.py`:
  - Define `TranslationRequest` model: `chapterId` (str, pattern `^[a-z0-9-]+$`), `originalContent` (str, 100-50000 chars)
  - Define `TranslationResponse` model: `chapterId`, `translatedContent`, `timestamp`, `cached` (bool)
  - Implement `POST /api/translate/chapter` endpoint:
    - Validate request (chapterId format, content length)
    - Call `translate_chapter_content(request.originalContent, request.chapterId)`
    - Return `TranslationResponse` with translated content and timestamp
    - Handle errors: 422 (validation), 429 (quota), 500 (internal), 504 (timeout)
  - Implement `GET /api/translate/health` endpoint:
    - Return `{"status": "healthy", "gemini_api_available": bool, "timestamp": int}`

- [ ] T008 Register translate router in `backend/src/main.py`:
  - Add import: `from .api import translate`
  - Add router registration: `app.include_router(translate.router)`

**Checkpoint**: Agent infrastructure ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Instant Urdu Translation (Priority: P1) 🎯 MVP Core

**Goal**: Any user (guest or logged-in) can click "اردو میں پڑھیں" button and see chapter instantly translated to Urdu

**Independent Test**: Navigate to any chapter, click "اردو میں پڑھیں", verify entire chapter becomes Urdu within 10 seconds

### Implementation for User Story 1

**Backend is complete** (Phase 2) - Focus on frontend integration

- [ ] T009 [US1] Add Urdu state to `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add state: `const [isUrdu, setIsUrdu] = useState(false)`
  - Add state: `const [urduContent, setUrduContent] = useState<string | null>(null)`
  - Add state: `const [isTranslating, setIsTranslating] = useState(false)`
  - Add state: `const [translationError, setTranslationError] = useState<string | null>(null)`
  - Add state: `const [isFromCache, setIsFromCache] = useState(false)`

- [ ] T010 [US1] Create translation service in `frontend/src/services/urduTranslationService.ts`:
  - Export `async function translateChapter(chapterId: string, originalContent: string): Promise<{translatedContent: string, timestamp: number}>`
  - Implement fetch to `http://localhost:8000/api/translate/chapter` with POST method
  - Set headers: `Content-Type: application/json`
  - Set body: `JSON.stringify({chapterId, originalContent})`
  - Use AbortController with 60-second timeout: `setTimeout(() => controller.abort(), 60000)`
  - Handle errors: Network (NETWORK_ERROR), 429 (QUOTA_EXCEEDED), 504 (TIMEOUT), 500 (API_ERROR)
  - Return JSON response

- [ ] T011 [US1] Implement localStorage caching in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add function `getCachedTranslation(chapterId: string): string | null`
    - Cache key format: `urdu_translation_${chapterId}_v1`
    - Check age < 7 days (604800000 ms)
    - Return cached content if valid, null otherwise
  - Add function `cacheTranslation(chapterId: string, content: string)`
    - Store: `{translatedContent: string, timestamp: number, version: 1}`
    - Save to localStorage with cache key

- [ ] T012 [US1] Implement translation handler in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add `const handleUrduClick = async () => { ... }`
  - Check cache first: `const cached = getCachedTranslation(chapterId); if (cached) { setUrduContent(cached); setIsFromCache(true); setIsUrdu(true); return; }`
  - If cache miss: `setIsTranslating(true); setTranslationError(null);`
  - Call `translateChapter(chapterId, originalContent)`
  - On success: `setUrduContent(response.translatedContent); cacheTranslation(chapterId, response.translatedContent); setIsUrdu(true); setIsFromCache(false); window.scrollTo({top: 0, behavior: 'smooth'});`
  - On error: `setTranslationError(error.message);`
  - Finally: `setIsTranslating(false);`

- [ ] T013 [US1] Add Urdu button to `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add button JSX before PersonalizeButton:
  ```tsx
  {!isUrdu && (
    <button
      onClick={handleUrduClick}
      disabled={isTranslating}
      className="urdu-translate-button"
      aria-label="Translate to Urdu"
    >
      {isTranslating ? 'Translating... / ترجمہ ہو رہا ہے' : 'اردو میں پڑھیں'}
    </button>
  )}
  ```

- [ ] T014 [US1] Add bilingual error display in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add error mapping object:
  ```tsx
  const ERROR_MESSAGES = {
    TIMEOUT: {en: "Translation took too long. Please try a shorter chapter.", ur: "ترجمہ میں زیادہ وقت لگ گیا۔ براہ کرم چھوٹا باب آزمائیں۔"},
    API_ERROR: {en: "Something went wrong. Please retry.", ur: "کچھ غلط ہو گیا۔ براہ کرم دوبارہ کوشش کریں۔"},
    NETWORK_ERROR: {en: "Network error. Check your connection.", ur: "نیٹ ورک کی خرابی۔ اپنا کنکشن چیک کریں۔"},
    QUOTA_EXCEEDED: {en: "Translation service temporarily unavailable. Try again in 24 hours.", ur: "ترجمے کی سروس عارضی طور پر دستیاب نہیں۔ 24 گھنٹوں میں دوبارہ کوشش کریں۔"}
  };
  ```
  - Add error display JSX:
  ```tsx
  {translationError && (
    <div className="translation-error">
      <p>{ERROR_MESSAGES[translationError]?.en || translationError}</p>
      <p className="urdu-text">{ERROR_MESSAGES[translationError]?.ur || ""}</p>
      <button onClick={handleUrduClick}>Retry / دوبارہ کوشش کریں</button>
    </div>
  )}
  ```

- [ ] T015 [US1] Implement Urdu content rendering with DOMPurify in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add conditional rendering:
  ```tsx
  {isUrdu && urduContent ? (
    <div className="urdu-content-wrapper">
      {/* Badge */}
      <div className="urdu-badge">
        {isFromCache ? '✓ Cached / محفوظ' : '✓ اردو میں'}
      </div>
      {/* Urdu Content */}
      <div
        className="urdu-content markdown"
        dangerouslySetInnerHTML={{
          __html: DOMPurify.sanitize(urduContent, {
            ALLOWED_TAGS: ['p', 'br', 'strong', 'em', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'ul', 'ol', 'li', 'a', 'code', 'pre', 'blockquote'],
            ALLOWED_ATTR: ['href', 'class', 'id']
          })
        }}
      />
    </div>
  ) : (
    children  // Original English content
  )}
  ```

- [ ] T016 [US1] Add Noto Nastaliq Urdu font to `frontend/docusaurus.config.js`:
  - In `themeConfig` section, add custom CSS that imports Google Font:
  ```javascript
  customCss: [
    require.resolve('./src/css/custom.css'),
  ],
  ```
  - Or add to head tags:
  ```javascript
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap',
      },
    },
  ],
  ```

- [ ] T017 [US1] Add RTL CSS styles to `frontend/src/css/custom.css`:
  ```css
  /* Urdu Translation Button */
  .urdu-translate-button {
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    color: white;
    border: none;
    padding: 0.75rem 1.5rem;
    border-radius: 8px;
    font-size: 1rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.3s ease;
    margin-bottom: 1rem;
  }

  .urdu-translate-button:hover:not(:disabled) {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
  }

  .urdu-translate-button:disabled {
    opacity: 0.6;
    cursor: not-allowed;
  }

  /* Urdu Content Wrapper */
  .urdu-content-wrapper {
    direction: rtl;
    text-align: right;
  }

  /* Urdu Content Typography */
  .urdu-content {
    font-family: 'Noto Nastaliq Urdu', Arial, sans-serif;
    line-height: 2;
    direction: rtl;
    text-align: right;
  }

  /* Keep code blocks LTR */
  .urdu-content pre,
  .urdu-content code {
    direction: ltr;
    text-align: left;
    font-family: 'Courier New', monospace;
  }

  /* Urdu Badge */
  .urdu-badge {
    background: #10b981;
    color: white;
    padding: 0.5rem 1rem;
    border-radius: 20px;
    font-size: 0.875rem;
    font-weight: 500;
    display: inline-block;
    margin-bottom: 1rem;
  }

  /* Error Display */
  .translation-error {
    background: #fee2e2;
    border: 2px solid #ef4444;
    border-radius: 8px;
    padding: 1rem;
    margin-bottom: 1rem;
  }

  .translation-error p {
    margin: 0.5rem 0;
    color: #991b1b;
  }

  .translation-error .urdu-text {
    direction: rtl;
    text-align: right;
    font-family: 'Noto Nastaliq Urdu', Arial, sans-serif;
  }

  .translation-error button {
    margin-top: 0.5rem;
    background: #ef4444;
    color: white;
    border: none;
    padding: 0.5rem 1rem;
    border-radius: 6px;
    cursor: pointer;
  }

  .translation-error button:hover {
    background: #dc2626;
  }
  ```

**Checkpoint**: User Story 1 complete - Users can now translate chapters to Urdu

---

## Phase 4: User Story 2 - Toggle Back to English (Priority: P2)

**Goal**: Users can switch back to original English content after viewing Urdu translation

**Independent Test**: After translating chapter to Urdu, click "English میں واپس" button and verify original English content is restored immediately

### Implementation for User Story 2

- [ ] T018 [US2] Add toggle-back button in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add button JSX (shown when `isUrdu === true`):
  ```tsx
  {isUrdu && (
    <button
      onClick={() => {
        setIsUrdu(false);
        setUrduContent(null);
        setTranslationError(null);
      }}
      className="english-toggle-button"
      aria-label="Switch back to English"
    >
      English میں واپس
    </button>
  )}
  ```

- [ ] T019 [US2] Add English toggle button styles to `frontend/src/css/custom.css`:
  ```css
  .english-toggle-button {
    background: #6b7280;
    color: white;
    border: none;
    padding: 0.5rem 1rem;
    border-radius: 6px;
    cursor: pointer;
    margin-left: 0.5rem;
  }

  .english-toggle-button:hover {
    background: #4b5563;
  }
  ```

- [ ] T020 [US2] Verify cached translation loads instantly on re-translation:
  - Test flow: Translate → Toggle back → Translate again
  - Verify: Second translation loads from cache (< 1 second, badge shows "✓ Cached / محفوظ")

**Checkpoint**: User Story 2 complete - Users can toggle between English and Urdu

---

## Phase 5: User Story 3 - Technical Term Translation Quality (Priority: P1)

**Goal**: Urdu translations use natural, readable Urdu while maintaining technical precision

**Independent Test**: Review translated chapter and verify technical terms are handled appropriately (e.g., "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2", "Jetson" → "جیٹسن")

### Implementation for User Story 3

**Note**: This story's implementation is primarily in Phase 2 (T005 - translation skill with glossary). Tasks here are for validation.

- [ ] T021 [US3] Test technical term translation with sample chapter:
  - Create test chapter with terms: ROS 2, URDF, Jetson, NVIDIA, Latency trap
  - Translate chapter via API: `curl -X POST http://localhost:8000/api/translate/chapter -H "Content-Type: application/json" -d '{"chapterId": "test-chapter", "originalContent": "# Test\\n\\nROS 2 is a framework. URDF is a format. Jetson Orin Nano costs $249. NVIDIA provides Isaac Sim. Latency trap is dangerous."}'`
  - Verify output contains:
    - "روبوٹک آپریٹنگ سسٹم 2" (ROS 2 full translation)
    - "یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ" (URDF full translation)
    - "جیٹسن" (Jetson transliteration)
    - "اینویڈیا" (NVIDIA transliteration)
    - "لیٹنسی ٹریپ (خطرناک تاخیر)" (Latency trap hybrid)

- [ ] T022 [US3] Verify markdown structure preservation:
  - Translate a chapter with complex markdown (headings, lists, code blocks, links)
  - Verify: All headings preserved, code blocks unchanged, links intact, list structure maintained
  - Check validation logic in `translate_to_urdu_skill.py` catches any mismatches

**Checkpoint**: User Story 3 complete - Translation quality validated

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T023 [P] Add logging to `backend/src/agents/urdu_translator_agent.py`:
  - Log translation start: `logger.info(f"Translation request: chapter={chapter_id}, length={len(original_content)}")`
  - Log translation completion: `logger.info(f"Translation completed: chapter={chapter_id}, time={duration}s")`
  - Log errors: `logger.error(f"Translation failed: {error}", exc_info=True)`

- [ ] T024 [P] Add request validation to `backend/src/api/translate.py`:
  - Validate `chapterId` matches pattern: `^[a-z0-9-]+$` (already in Pydantic model)
  - Validate `originalContent` length: 100-50000 characters (already in Pydantic model)
  - Add custom validation for empty content: `if not request.originalContent.strip(): raise ValueError("Content cannot be empty")`

- [ ] T025 [P] Test mobile responsiveness of Urdu button:
  - Test on mobile viewport (375px width)
  - Test on tablet viewport (768px width)
  - Verify button remains visible and accessible
  - Adjust CSS if needed (e.g., reduce padding, font size on mobile)

- [ ] T026 End-to-end manual testing:
  - **Test 1**: Guest user translates chapter → Verify Urdu content displays with RTL layout
  - **Test 2**: Toggle back to English → Verify original content restored
  - **Test 3**: Translate again → Verify cache loads instantly (< 1 second)
  - **Test 4**: Clear localStorage → Translate → Verify fresh translation (5-10 seconds)
  - **Test 5**: Test with long chapter (>5000 words) → Verify completes within 60 seconds
  - **Test 6**: Test error scenarios (stop backend, invalid API key) → Verify bilingual error messages

- [ ] T027 [P] Update `specs/006-urdu-translation/quickstart.md` if any implementation patterns changed:
  - Verify code samples match actual implementation
  - Update file paths if any changed
  - Add any new troubleshooting scenarios discovered during testing

- [ ] T028 Commit work:
  - Stage all changes: `git add backend/src/agents/ backend/src/api/translate.py backend/src/main.py frontend/src/components/Personalize/ChapterWrapper.tsx frontend/src/services/urduTranslationService.ts frontend/src/css/custom.css frontend/docusaurus.config.js`
  - Commit with message:
  ```
  feat(urdu): per-chapter Urdu translation button using @urdu-translator

  - Add @urdu-translator subagent with 3-tier glossary (translate/transliterate/hybrid)
  - Integrate Urdu toggle in ChapterWrapper (no auth required for accessibility)
  - Add Noto Nastaliq Urdu font with RTL text support
  - Implement localStorage caching (7-day TTL) for instant re-translation
  - Add bilingual error messages (English + Urdu)
  - Preserve markdown structure with validation
  - Success criteria: <10s translation, <1s cached load, 100% markdown preservation

  Technical stack:
  - Backend: OpenAI Agents SDK + Gemini 2.0 Flash API
  - Frontend: React + DOMPurify + localStorage
  - Translation quality: Tier 1 (ROS 2→روبوٹک آپریٹنگ سسٹم 2), Tier 2 (Jetson→جیٹسن), Tier 3 (Latency trap→لیٹنسی ٹریپ خطرناک تاخیر)

  Closes #006-urdu-translation

  🤖 Generated with Claude Code (https://claude.com/claude-code)

  Co-Authored-By: Claude Opus 4.5 <noreply@anthropic.com>
  ```

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Instant Urdu translation
- **User Story 2 (Phase 4)**: Depends on US1 - Toggle back (can't toggle if no Urdu content)
- **User Story 3 (Phase 5)**: Depends on Foundational - Quality validation (can run parallel with US1 if desired)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Instant translation - No dependencies (depends only on Foundational)
- **User Story 2 (P2)**: Toggle back - Depends on US1 (need Urdu content to toggle from)
- **User Story 3 (P1)**: Translation quality - No dependencies (depends only on Foundational)

**Note**: US1 and US3 are both P1 priority and can be developed in parallel after Foundational phase.

### Within Each User Story

- **US1 Tasks**: T009-T017 must be done mostly sequentially (each builds on previous)
  - T009 (state) → T010 (service) → T011 (caching) → T012 (handler) → T013 (button) → T014 (errors) → T015 (rendering) → T016/T017 (styling can be parallel)
- **US2 Tasks**: T018-T020 are sequential but quick (simple toggle logic)
- **US3 Tasks**: T021-T022 are validation tests, can be done in parallel

### Parallel Opportunities

- **Phase 1 Setup**: T002, T003 can run in parallel (different package managers)
- **Phase 2 Foundational**: T005 (skill) must complete before T006 (agent), then T007 (API), then T008 (registration) - all sequential
- **Phase 6 Polish**: T023, T024, T025, T027 can all run in parallel (different files)

---

## Parallel Example: Phase 1 Setup

```bash
# Launch setup tasks in parallel:
Task T002: "Verify OpenAI Agents SDK"
Task T003: "Verify DOMPurify"
# These touch different package managers, can run simultaneously
```

---

## Implementation Strategy

### MVP First (Critical Path)

1. **Complete Phase 1**: Setup (T001-T004) - Verify existing dependencies
2. **Complete Phase 2**: Foundational (T005-T008) - Backend agent infrastructure
3. **Complete Phase 3**: User Story 1 (T009-T017) - Full translation flow
4. **STOP and VALIDATE**: Test translation end-to-end
5. **Optional Phase 4**: User Story 2 (T018-T020) - Toggle functionality
6. **Optional Phase 5**: User Story 3 (T021-T022) - Quality validation
7. **Optional Phase 6**: Polish (T023-T028)

**Total Estimated Tasks**: 28

### Incremental Delivery

1. **After Phase 2**: Backend agent ready, can test with curl/Postman
2. **After Phase 3**: Full Urdu translation working in UI (MVP!)
3. **After Phase 4**: Toggle functionality added
4. **After Phase 5**: Translation quality validated
5. **After Phase 6**: Production-ready with logging, validation, testing

### User's Simplified 8-Task Approach (Mapped to Full Tasks)

Your original 8 tasks map to our comprehensive tasks as follows:

1. **"Update ChapterWrapper.jsx - Add state"** → T009 (Add Urdu state)
2. **"On Urdu click - POST to /api/translate-urdu"** → T010-T012 (Service + caching + handler)
3. **"Create /api/translate-urdu/route.ts"** → T005-T008 (Full backend: skill + agent + API + registration)
4. **"Add Google Font Noto Nastaliq Urdu"** → T016 (Font configuration)
5. **"CSS: RTL + font-family"** → T017 (RTL CSS styles)
6. **"Add badge"** → T015 (Badge in Urdu content rendering)
7. **"Test: Click Urdu"** → T026 (E2E manual testing)
8. **"Commit"** → T028 (Git commit with proper message)

**Additional tasks in our plan** ensure:
- Proper error handling (T014 - bilingual errors)
- Toggle back functionality (T018-T020 - US2)
- Translation quality validation (T021-T022 - US3)
- Production readiness (T023-T025 - logging, validation, mobile)

---

## Task Summary

- **Total Tasks**: 28
- **Setup**: 4 tasks (T001-T004)
- **Foundational**: 4 tasks (T005-T008) - BLOCKING
- **User Story 1**: 9 tasks (T009-T017) - Instant translation
- **User Story 2**: 3 tasks (T018-T020) - Toggle back
- **User Story 3**: 2 tasks (T021-T022) - Quality validation
- **Polish**: 6 tasks (T023-T028)

**Parallel Opportunities**: 6 tasks marked [P] can run in parallel within their phases

**Critical Path**: T001 → T004 → T005 → T006 → T007 → T008 → T009 → T010 → T011 → T012 → T013 → T015 → T026 → T028

---

## Notes

- User requested simplified approach: **Reuse existing ChapterWrapper** instead of creating separate Urdu components
- This aligns with user's 8-task plan while ensuring all spec requirements (FR-001 to FR-015) are covered
- **No authentication required** per spec.md FR-002 - Urdu translation is accessible to all users
- Feature leverages EXISTING personalization infrastructure (ChapterWrapper, DOMPurify)
- Backend uses EXISTING Gemini API configuration from feature 005 (personalization)
- Tasks are marked with [US1], [US2], [US3] labels for traceability to user stories
- Each phase has clear "Checkpoint" description for validation
- User Story 1 is the MVP - can stop here for initial release
- User Stories 2 and 3 are enhancements but recommended for complete feature

---

## Ready to Begin Implementation

All tasks are defined with exact file paths and specific implementation details. Start with Phase 1 (Setup) to verify dependencies, then proceed to Phase 2 (Foundational) to build the backend agent infrastructure.

**Next step**: Begin implementation starting with T001 (verify agents directory exists).
