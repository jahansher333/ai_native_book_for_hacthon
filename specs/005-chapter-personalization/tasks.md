# Tasks: Chapter Personalization

**Input**: Design documents from `/specs/005-chapter-personalization/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/personalize-api.yaml

**Tests**: Not explicitly requested in feature specification - focusing on implementation tasks

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

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

**Purpose**: Backend agent infrastructure initialization

- [X] T001 Create backend agents directory structure: `backend/src/agents/` and `backend/src/agents/skills/`
- [X] T002 [P] Install OpenAI Agents SDK dependency in backend: Add `openai-agents` to `backend/requirements.txt`
- [X] T003 [P] Install DOMPurify dependency in frontend: `npm install dompurify @types/dompurify` in `frontend/`
- [X] T004 Verify Gemini API configuration in `backend/.env`: Ensure `GEMINI_API_KEY`, `BASE_URL`, and `MODEL_NAME` are set

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core agent infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Implement `PersonalizeChapterSkill` class in `backend/src/agents/skills/personalize_chapter_skill.py`:
  - Create Skill class with `name="personalizeChapterSkill"`
  - Implement `execute()` method signature: `async def execute(self, chapter_content: str, user_profile: dict, chapter_id: str) -> str`
  - Implement `_build_prompt()` method that adapts prompt based on experience (beginner/intermediate/advanced) and hardware (hasRTX, hasJetson, hasRobot)
  - Implement `_get_system_prompt()` method returning expert technical writer system prompt
  - Implement `_validate_personalization()` method to check length (±50%) and markdown structure preservation
  - Configure OpenAI client with Gemini endpoint: `base_url=settings.base_url`, `api_key=settings.gemini_api_key`
  - Call `chat.completions.create()` with `model="gemini-2.0-flash-exp"`, `temperature=0.7`, `max_tokens=8192`

- [X] T006 Implement `personalizer_agent.py` orchestrator in `backend/src/agents/personalizer_agent.py`:
  - Import `Agent` and `Runner` from `openai_agents`
  - Import `personalizeChapterSkill` from `skills.personalize_chapter_skill`
  - Initialize `personalizer_agent` with name="Chapter Personalizer", skills=[personalizeChapterSkill], model="gemini-2.0-flash-exp"
  - Implement `async def personalize_chapter_content(original_content: str, user_profile: dict, chapter_id: str) -> str`
  - Use `asyncio.wait_for(runner.run(...), timeout=60.0)` for 60-second timeout
  - Return `result.personalized_content`

- [X] T007 Fix import in `backend/src/api/personalize.py`:
  - Change line 62 to: `from ..agents.personalizer_agent import personalize_chapter_content`
  - Verify endpoint calls `personalize_chapter_content()` correctly with chapter content and user profile

**Checkpoint**: Agent infrastructure ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 3 - Personalize Button Visibility (Priority: P1) 🎯 MVP Component 1

**Goal**: Display "Personalize Chapter" button at the top of every chapter, visible to all users

**Independent Test**: Navigate to any chapter and verify button appears consistently at the top

### Implementation for User Story 3

- [X] T008 [US3] Verify `PersonalizeButton.tsx` exists and renders correctly in `frontend/src/components/Personalize/PersonalizeButton.tsx` (CREATED - full implementation with loading, error, and personalized states)

- [X] T009 [US3] Verify `ChapterWrapper.tsx` integration in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Confirm it wraps Docusaurus chapters ✓
  - Confirm it renders PersonalizeButton at the top ✓
  - Confirm it passes `onPersonalize`, `isLoading`, `error` props ✓
  - CREATED with full integration to usePersonalization hook and UserProfileContext

- [X] T010 [US3] Verify Docusaurus theme swizzling in `frontend/src/theme/DocItem/Layout/index.tsx`:
  - Check if ChapterWrapper is imported and wrapping the chapter content ✓
  - ALREADY INTEGRATED - ChapterWrapper wraps all DocItemContent
  - Created styles.module.css for layout component

**Checkpoint**: Button should now be visible on all chapters for all users (logged-in and anonymous)

---

## Phase 4: User Story 2 - Anonymous User Auth Check (Priority: P2) 🎯 MVP Component 2

**Goal**: Show "Please login to personalize chapters" message when anonymous users click the button

**Independent Test**: Log out, navigate to any chapter, click "Personalize Chapter", verify login prompt appears

### Implementation for User Story 2

- [X] T011 [US2] Verify auth check in `frontend/src/hooks/usePersonalization.ts`:
  - Confirm `personalizeContent()` checks `isAuthenticated` prop ✓
  - Confirm it sets error "Please log in to personalize content" when `!isAuthenticated` ✓
  - VERIFIED - Displays "Please log in to personalize content" for anonymous users

- [X] T012 [US2] Verify error display in `frontend/src/components/Personalize/PersonalizeButton.tsx`:
  - Confirm error message renders when `error` prop is set ✓
  - Confirm message shows "Please login to personalize chapters" ✓
  - VERIFIED - Error UI renders correctly with proper styling

- [ ] T013 [US2] (OPTIONAL) Add login redirect logic in `frontend/src/hooks/usePersonalization.ts`:
  - If user is not authenticated and clicks button, optionally redirect to `/signin` with return URL
  - Implementation: `window.location.href = `/signin?returnTo=${window.location.pathname}``
  - This is optional enhancement - not required for MVP

**Checkpoint**: Anonymous users should see clear login prompt instead of attempting personalization

---

## Phase 5: User Story 1 - Logged-in User Personalizes Chapter (Priority: P1) 🎯 MVP Core

**Goal**: Logged-in users can personalize chapters and see personalized content replace original DOM

**Independent Test**: Log in, navigate to any chapter, click "Personalize Chapter", verify personalized content displays

### Implementation for User Story 1

- [X] T014 [P] [US1] Create `personalizationService.ts` API client in `frontend/src/services/personalizationService.ts`:
  - Export `async function personalizeChapter(request: PersonalizeChapterRequest): Promise<PersonalizeChapterResponse>`
  - Get JWT token from localStorage: `const token = localStorage.getItem('auth_token')` or from UserProfileContext
  - Make POST request to `/api/personalize/chapter` with headers: `Authorization: Bearer ${token}`, `Content-Type: application/json`
  - Include request body: `{ chapterId, originalContent, userProfile }`
  - Set 60-second timeout using AbortController: `setTimeout(() => controller.abort(), 60000)`
  - Parse and return JSON response
  - Handle errors (401, 403, 429, 500, 504) with appropriate error messages

- [X] T015 [US1] Integrate `personalizationService` into `frontend/src/hooks/usePersonalization.ts`:
  - Import `personalizeChapter` from `../services/personalizationService`
  - In `personalizeContent()` function, call `await personalizeChapter({ chapterId, originalContent, userProfile })`
  - Extract `personalizedContent` from response
  - Cache result in localStorage with key: `personalized_${chapterId}_${userEmail}_${profileHash}`
  - Update state: `setPersonalizedContent(result.personalizedContent)`
  - Scroll to top after successful personalization

- [X] T016 [US1] Verify DOM replacement in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Confirm `personalizedContent` from `usePersonalization` hook replaces chapter DOM
  - Confirm DOMPurify is used to sanitize HTML before insertion: `import DOMPurify from 'dompurify'`
  - Sanitization: `const sanitized = DOMPurify.sanitize(personalizedContent)`
  - DOM replacement: Replace article content with sanitized HTML
  - Verify original markdown files are NOT modified (session-based only)

- [ ] T017 [US1] Verify loading state in `frontend/src/components/Personalize/LoadingOverlay.tsx`:
  - Confirm loading overlay displays during personalization (when `isPersonalizing` is true)
  - Confirm spinner or loading animation is visible
  - Existing component should handle this - verify behavior

- [ ] T018 [US1] Verify PersonalizationBadge in `frontend/src/components/Personalize/PersonalizationBadge.tsx`:
  - Confirm badge shows "Personalized" or "From Cache" after successful personalization
  - Confirm badge has clear UI indicating personalized state
  - Existing component should handle this - verify behavior

- [ ] T019 [US1] Add error handling in `frontend/src/hooks/usePersonalization.ts`:
  - Catch network errors, timeouts, and API errors
  - Set appropriate error messages for different error codes:
    - 401: "Session expired, please log in again"
    - 429: "Too many requests. Please wait before trying again."
    - 504: "Personalization took too long. Try a shorter chapter."
    - 500: "Something went wrong. Please retry."
  - Ensure retry button appears in PersonalizeButton error UI

**Checkpoint**: Logged-in users should be able to fully personalize chapters end-to-end

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T020 [P] Add rate limiting in `backend/src/api/personalize.py`:
  - Implement rate limiter: 10 requests per minute per user
  - Return 429 Too Many Requests when limit exceeded
  - Use user_email from JWT for rate limit key

- [ ] T021 [P] Add logging in `backend/src/agents/personalizer_agent.py`:
  - Log personalization start: `logger.info(f"Personalization request: user={user_email}, chapter={chapter_id}")`
  - Log personalization completion: `logger.info(f"Personalization completed: time={duration}s")`
  - Log errors: `logger.error(f"Personalization failed: {error}", exc_info=True)`

- [ ] T022 [P] Add request validation in `backend/src/api/personalize.py`:
  - Validate `chapterId` matches pattern: `^[a-z0-9-]+$`
  - Validate `originalContent` length: 100-50000 characters
  - Validate `userProfile` has all required fields
  - Return 422 Validation Error with specific messages

- [ ] T023 [P] Verify UserProfileContext in `frontend/src/contexts/UserProfileContext.tsx`:
  - Confirm user profile (experience, hasRTX, hasJetson, hasRobot) is available
  - Confirm user_email is accessible from context
  - This is EXISTING infrastructure - verify it provides required data

- [ ] T024 [P] Add cache invalidation button in `frontend/src/components/Personalize/ChapterWrapper.tsx`:
  - Add "Restore Original" button that calls `resetToDefault()` from usePersonalization hook
  - Button should clear localStorage cache and display original chapter
  - Optional enhancement for better UX

- [ ] T025 [P] Verify mobile responsiveness of PersonalizeButton:
  - Test button on mobile, tablet, desktop viewports
  - Ensure button remains visible and accessible on all screen sizes
  - Adjust CSS/Tailwind classes if needed in `PersonalizeButton.tsx`

- [ ] T026 End-to-end manual testing:
  - Test all three user stories independently
  - Test with different user profiles (beginner/intermediate/advanced, various hardware)
  - Test edge cases: very long chapters, complex formatting, multiple rapid clicks
  - Test error scenarios: backend down, timeout, invalid JWT
  - Verify caching works correctly (7-day TTL)

- [ ] T027 Update `specs/005-chapter-personalization/quickstart.md` with actual implementation details if any patterns changed during implementation

- [ ] T028 Performance testing:
  - Test with 50 concurrent personalization requests
  - Verify response times under load
  - Monitor Gemini API rate limits and costs

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 3 (Phase 3)**: Depends on Foundational - Button visibility
- **User Story 2 (Phase 4)**: Depends on Foundational and US3 - Auth check on button click
- **User Story 1 (Phase 5)**: Depends on Foundational, US3, US2 - Full personalization flow
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 3 (P1)**: Button visibility - Can start after Foundational (Phase 2)
- **User Story 2 (P2)**: Auth check - Depends on US3 (button must exist)
- **User Story 1 (P1)**: Full personalization - Depends on US3 (button) and US2 (auth check)

**Note**: US1 and US3 are both P1 priority. US3 must come first because US1 requires the button to exist.

### Within Each User Story

- Frontend and backend tasks can often be done in parallel
- API client must be created before hook integration
- Agent infrastructure (Foundational) must be complete before any API calls work
- DOM replacement depends on API integration being complete

### Parallel Opportunities

- **Phase 1 Setup**: T002, T003 can run in parallel (different dependency managers)
- **Phase 2 Foundational**: T005 (skill) and T006 (agent) are sequential (agent depends on skill)
- **Phase 5 User Story 1**: T014 (API client) can run in parallel with T016-T018 (UI verifications)
- **Phase 6 Polish**: T020, T021, T022, T023, T024, T025 can all run in parallel (different files)

---

## Parallel Example: Phase 1 Setup

```bash
# Launch setup tasks in parallel:
Task T002: "Install OpenAI Agents SDK dependency"
Task T003: "Install DOMPurify dependency"
# These touch different package managers, can run simultaneously
```

---

## Parallel Example: Phase 5 User Story 1

```bash
# Launch in parallel:
Task T014: "Create personalizationService.ts API client"
Task T016: "Verify DOM replacement in ChapterWrapper"
Task T017: "Verify loading state in LoadingOverlay"
Task T018: "Verify PersonalizationBadge"

# Then sequentially:
Task T015: "Integrate personalizationService into usePersonalization hook" (depends on T014)
Task T019: "Add error handling" (depends on T015)
```

---

## Implementation Strategy

### MVP First (Critical Path)

1. **Complete Phase 1**: Setup (T001-T004) - ~30 minutes
2. **Complete Phase 2**: Foundational (T005-T007) - ~4 hours
   - T005: Implement PersonalizeChapterSkill - ~2 hours
   - T006: Implement personalizer_agent orchestrator - ~1 hour
   - T007: Fix API import - ~15 minutes
3. **Complete Phase 3**: User Story 3 - Button Visibility (T008-T010) - ~1 hour
4. **Complete Phase 4**: User Story 2 - Auth Check (T011-T013) - ~1 hour
5. **Complete Phase 5**: User Story 1 - Full Personalization (T014-T019) - ~4 hours
6. **STOP and VALIDATE**: Test end-to-end personalization flow
7. **Optional Phase 6**: Polish (T020-T028) - ~3 hours

**Total Estimated Time**: ~13-15 hours for core MVP

### Incremental Delivery

1. **After Phase 2**: Agent infrastructure ready, can test with curl/Postman
2. **After Phase 3**: Button visible on all chapters
3. **After Phase 4**: Anonymous users see login prompt
4. **After Phase 5**: Full personalization working end-to-end (MVP!)
5. **After Phase 6**: Production-ready with rate limiting, logging, validation

### Parallel Team Strategy

With 2 developers:

1. **Together**: Complete Setup + Foundational (Phases 1-2)
2. **Split**:
   - Developer A: Frontend tasks (T008-T010, T011-T012, T014-T019)
   - Developer B: Backend tasks (T005-T007, T020-T022)
3. **Together**: Integration testing and polish (T026-T028)

---

## Task Summary

- **Total Tasks**: 28
- **Setup**: 4 tasks
- **Foundational**: 3 tasks (BLOCKING)
- **User Story 3**: 3 tasks
- **User Story 2**: 3 tasks
- **User Story 1**: 6 tasks
- **Polish**: 9 tasks

**Parallel Opportunities**: 15 tasks marked [P] can run in parallel within their phases

**Critical Path**: T001 → T004 → T005 → T006 → T007 → T008 → T011 → T014 → T015 → T016

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- **Many frontend components already exist** - focus on verification and integration
- **Backend agent infrastructure is NEW** - primary implementation effort
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Feature leverages EXISTING Better-Auth JWT authentication - no auth implementation needed
- Feature leverages EXISTING user profile context - no profile model changes needed
