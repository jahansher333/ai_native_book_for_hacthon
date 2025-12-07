# Implementation Plan: Chapter Personalization

**Branch**: `005-chapter-personalization` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-chapter-personalization/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable logged-in users to personalize any chapter by clicking a button at the chapter start. The system sends chapter content and user profile to the backend, which invokes an OpenAI Agents SDK skill (`personalizeChapterSkill`) using Gemini API to rewrite the chapter tailored to the user's experience level and hardware setup. The personalized content replaces the original chapter DOM without modifying markdown source files.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 18+ (frontend)
**Primary Dependencies**:
- Backend: FastAPI, OpenAI Agents SDK, Gemini API (via OpenAI-compatible endpoint), Neon Postgres, Better-Auth
- Frontend: Docusaurus v3, React 18, TailwindCSS
**Storage**: Neon Postgres (user profiles, personalization cache), LocalStorage (frontend cache)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Web application (GitHub Pages deployment)
**Project Type**: Web (backend + frontend)
**Performance Goals**:
- Personalization response < 60 seconds for typical chapter (under 5000 words)
- Support 50 concurrent personalization requests
- Frontend response time < 1 second for auth check
**Constraints**:
- DOM-only replacement (no markdown file modification)
- Maintain 100% content accuracy during personalization
- Session-based personalization (non-persistent on refresh)
**Scale/Scope**:
- Expected load: 100-500 concurrent users
- Chapter count: 20-50 chapters
- Average chapter length: 2000-5000 words

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle VI: Urdu + Personalization Ready ✅

**Status**: PASS - This feature IS the personalization implementation

This feature directly implements the "Personalize for me" button requirement from Constitution Principle VI. The architecture includes:
- User profile collection via Better-Auth (experience level, hardware access)
- Per-chapter personalization button
- Content rewriting based on user context
- DOM-only replacement (no markdown modification)

**Compliance**:
- User profile fields align with Constitution requirements: hardware access (RTX_GPU, JETSON, REAL_ROBOT), experience level, learning style
- API endpoint `/api/personalize/chapter` defined with clear contracts
- Personalization infrastructure baked into architecture from start

### Principle VIII: Authentication & User Profiles (Better-Auth) ✅

**Status**: PASS - Leverages existing Better-Auth integration

This feature extends existing Better-Auth implementation to enable personalized experiences. Required profile data (hardware, experience) is already collected at signup.

**Compliance**:
- JWT authentication required for personalization
- Profile data drives personalization logic
- Anonymous users can still read content (login enhances, doesn't gate)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── agents/
│   │   ├── personalizer_agent.py      # NEW: OpenAI Agents SDK agent
│   │   └── skills/
│   │       └── personalize_chapter_skill.py  # NEW: Personalization skill
│   ├── api/
│   │   └── personalize.py             # EXISTING: API endpoint (needs agent import fix)
│   ├── models/
│   │   └── user.py                    # EXISTING: User model with profile
│   └── services/
│       └── auth_service.py            # EXISTING: JWT auth
└── tests/
    ├── test_personalizer_agent.py     # NEW: Agent tests
    └── test_personalize_api.py        # NEW: API integration tests

frontend/
├── src/
│   ├── components/
│   │   ├── Personalize/
│   │   │   ├── PersonalizeButton.tsx  # EXISTING: Button component
│   │   │   ├── ChapterWrapper.tsx     # EXISTING: DOM wrapper
│   │   │   ├── LoadingOverlay.tsx     # EXISTING: Loading UI
│   │   │   └── PersonalizationBadge.tsx # EXISTING: Status badge
│   │   └── Auth/
│   │       ├── AuthGuard.tsx          # EXISTING: Auth check
│   │       └── SigninForm.tsx         # EXISTING: Login form
│   ├── hooks/
│   │   └── usePersonalization.ts      # EXISTING: Personalization logic
│   ├── services/
│   │   └── personalizationService.ts  # NEW: API client
│   ├── contexts/
│   │   └── UserProfileContext.tsx     # EXISTING: User profile state
│   └── theme/
│       ├── Root.tsx                   # EXISTING: Global providers
│       └── DocItem/
│           └── Layout/index.tsx       # MAY NEED UPDATE: Chapter layout integration
└── tests/
    ├── PersonalizeButton.test.tsx     # NEW: Component tests
    └── usePersonalization.test.ts     # NEW: Hook tests
```

**Structure Decision**: Web application structure (Option 2)

This is a full-stack web application with:
- **Backend**: FastAPI service handling personalization requests via OpenAI Agents SDK
- **Frontend**: Docusaurus-based React application with personalization UI components
- **Key Integration Points**:
  - `backend/src/agents/personalizer_agent.py` orchestrates the OpenAI Agents SDK Runner
  - `backend/src/agents/skills/personalize_chapter_skill.py` implements the personalization logic
  - `frontend/src/components/Personalize/ChapterWrapper.tsx` wraps chapters and manages DOM replacement

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations. This feature aligns with Constitution Principles VI and VIII.

---

## Summary

This implementation plan provides a comprehensive architecture for enabling logged-in users to personalize textbook chapters using OpenAI Agents SDK with Gemini API. The feature integrates seamlessly with existing Better-Auth authentication and Docusaurus frontend infrastructure.

**Key Implementation Files Created**:
- ✅ `research.md` - Technical decisions and alternatives analysis
- ✅ `data-model.md` - Entity definitions and data flow
- ✅ `contracts/personalize-api.yaml` - OpenAPI specification
- ✅ `quickstart.md` - Developer onboarding guide

**Critical Path**:
1. Implement `backend/src/agents/personalizer_agent.py` (OpenAI Agents SDK orchestration)
2. Implement `backend/src/agents/skills/personalize_chapter_skill.py` (Gemini API personalization)
3. Fix import in `backend/src/api/personalize.py`
4. Create `frontend/src/services/personalizationService.ts` (API client)
5. End-to-end testing

**Ready for**: `/sp.tasks` command to generate detailed implementation tasks

