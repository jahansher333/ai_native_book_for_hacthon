# Data Model: Chapter Urdu Translation Button

**Branch**: `006-urdu-translation` | **Date**: 2025-12-07
**Input**: Feature specification and research from `/specs/006-urdu-translation/`

## Purpose

This document defines all entities, their relationships, validation rules, and state transitions for the Urdu translation feature.

---

## Entities

### 1. TranslationRequest

**Description**: Request payload sent from frontend to backend to translate a chapter.

**Schema**:
```typescript
interface TranslationRequest {
  chapterId: string;        // Unique chapter identifier (e.g., "ros2-intro")
  originalContent: string;  // Full chapter markdown content
}
```

**Validation Rules**:
- `chapterId`: Required, matches pattern `^[a-z0-9-]+$`, length 3-100 characters
- `originalContent`: Required, length 100-50,000 characters (prevents empty or excessively large requests)

**Source**: Extracted from Docusaurus chapter DOM by frontend

**Usage**: POST `/api/translate/chapter` request body

---

### 2. TranslationResponse

**Description**: Response payload returned from backend after successful translation.

**Schema**:
```typescript
interface TranslationResponse {
  chapterId: string;          // Echo of request chapterId
  translatedContent: string;  // Urdu markdown content
  timestamp: number;          // Unix timestamp (Date.now())
  cached: boolean;            // Whether response came from backend cache (false for MVP)
}
```

**Validation Rules**:
- `translatedContent`: Required, non-empty, valid UTF-8 Urdu text
- `timestamp`: Required, Unix epoch milliseconds
- `cached`: Boolean, default false

**Usage**: Returned to frontend, stored in localStorage cache

---

### 3. TranslationCacheEntry

**Description**: Cached translation stored in browser localStorage.

**Schema**:
```typescript
interface TranslationCacheEntry {
  translatedContent: string;  // Full Urdu markdown
  timestamp: number;          // Cache creation time (Date.now())
  version: number;            // Cache version (currently 1)
}
```

**Storage Key Format**:
```typescript
const cacheKey = `urdu_translation_${chapterId}_v${version}`;
// Example: "urdu_translation_ros2-intro_v1"
```

**Validation Rules**:
- `translatedContent`: Required, non-empty
- `timestamp`: Required, used for TTL calculation (7 days)
- `version`: Required, incremented when translation algorithm changes

**TTL**: 7 days (604,800,000 milliseconds)

**Cache Invalidation**:
- Age > 7 days: Automatic expiration
- Version mismatch: Manual invalidation (on version bump)
- User action: "Restore Original" button clears cache

**Usage**: Stored in `localStorage` after successful translation

---

### 4. TranslationState

**Description**: Frontend state machine tracking current translation status.

**Schema**:
```typescript
type TranslationStatus = 'idle' | 'translating' | 'translated' | 'error';

interface TranslationState {
  status: TranslationStatus;
  translatedContent: string | null;
  error: string | null;
  isFromCache: boolean;
}
```

**State Transitions**:

```
[idle]
  ↓ (user clicks "اردو میں پڑھیں")
[translating] (loading spinner, disabled button)
  ↓ (translation succeeds)
[translated] (show Urdu content, badge "اردو میں", "English میں واپس" button)
  ↓ (user clicks "Restore Original")
[idle] (original English content)

Alternative path from [translating]:
  ↓ (translation fails)
[error] (show bilingual error message, retry button)
  ↓ (user clicks retry)
[translating]
```

**Validation Rules**:
- `status`: Required, one of enum values
- `translatedContent`: Null when status is 'idle', 'translating', or 'error'; non-null when 'translated'
- `error`: Null except when status is 'error'
- `isFromCache`: Boolean, true if loaded from localStorage

**Usage**: Managed by `useUrduTranslation` React hook

---

### 5. TranslationError

**Description**: Error object for translation failures.

**Schema**:
```typescript
interface TranslationError {
  code: 'TIMEOUT' | 'API_ERROR' | 'NETWORK_ERROR' | 'QUOTA_EXCEEDED' | 'VALIDATION_ERROR';
  messageEn: string;  // English error message
  messageUr: string;  // Urdu error message
}
```

**Error Codes**:

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `TIMEOUT` | 504 | Translation took > 60 seconds |
| `API_ERROR` | 500 | Backend internal error |
| `NETWORK_ERROR` | N/A | Fetch failed (no response) |
| `QUOTA_EXCEEDED` | 429 | Gemini API quota exhausted |
| `VALIDATION_ERROR` | 422 | Invalid request payload |

**Bilingual Messages**:
```typescript
const ERROR_MESSAGES: Record<TranslationError['code'], { en: string, ur: string }> = {
  TIMEOUT: {
    en: "Translation took too long. Please try a shorter chapter.",
    ur: "ترجمہ میں زیادہ وقت لگ گیا۔ براہ کرم چھوٹا باب آزمائیں۔"
  },
  API_ERROR: {
    en: "Something went wrong. Please retry.",
    ur: "کچھ غلط ہو گیا۔ براہ کرم دوبارہ کوشش کریں۔"
  },
  NETWORK_ERROR: {
    en: "Network error. Check your connection.",
    ur: "نیٹ ورک کی خرابی۔ اپنا کنکشن چیک کریں۔"
  },
  QUOTA_EXCEEDED: {
    en: "Translation service temporarily unavailable. Try again in 24 hours.",
    ur: "ترجمے کی سروس عارضی طور پر دستیاب نہیں۔ 24 گھنٹوں میں دوبارہ کوشش کریں۔"
  },
  VALIDATION_ERROR: {
    en: "Invalid chapter content. Please refresh the page.",
    ur: "غلط باب کا مواد۔ براہ کرم صفحہ تازہ کریں۔"
  }
};
```

**Usage**: Displayed in `UrduButton.tsx` error UI

---

### 6. UrduSkillInput

**Description**: Input passed to `translateToUrduSkill.execute()` method on backend.

**Schema**:
```python
@dataclass
class UrduSkillInput:
    chapter_content: str  # Original English markdown
    chapter_id: str       # Chapter identifier for logging
```

**Validation Rules** (Python):
```python
assert len(chapter_content) >= 100, "Content too short"
assert len(chapter_content) <= 50000, "Content too long"
assert re.match(r'^[a-z0-9-]+$', chapter_id), "Invalid chapter ID"
```

**Usage**: Input to OpenAI Agents SDK skill

---

### 7. UrduSkillOutput

**Description**: Output returned from `translateToUrduSkill.execute()` method.

**Schema**:
```python
@dataclass
class UrduSkillOutput:
    translated_content: str  # Urdu markdown
    validation_passed: bool  # Markdown structure validation result
    validation_message: str  # Error message if validation failed
```

**Validation Rules**:
- `translated_content`: Must be non-empty UTF-8 string
- `validation_passed`: True if markdown structure matches original
- `validation_message`: Empty string if validation passed

**Usage**: Returned to agent orchestrator, then to API endpoint

---

## Relationships

```
┌─────────────────┐
│  Frontend       │
│  (React)        │
└────────┬────────┘
         │ POST /api/translate/chapter
         │ TranslationRequest
         ↓
┌─────────────────┐
│  Backend        │
│  FastAPI        │
└────────┬────────┘
         │ call agent
         ↓
┌─────────────────┐
│ Urdu Translator │
│ Agent           │
└────────┬────────┘
         │ execute skill
         ↓
┌─────────────────┐
│ TranslateToUrdu │
│ Skill           │
└────────┬────────┘
         │ Gemini API
         ↓
┌─────────────────┐
│ Gemini 2.0      │
│ (via OpenAI)    │
└────────┬────────┘
         │ Urdu markdown
         ↓
         (returns up stack)
         ↓
┌─────────────────┐
│ TranslationResp │
│ + Cache in LS   │
└─────────────────┘
```

**Key Relationships**:
1. `TranslationRequest` → `Backend API` → `UrduSkillInput`
2. `UrduSkillOutput` → `Backend API` → `TranslationResponse`
3. `TranslationResponse` → `Frontend` → `TranslationCacheEntry` (localStorage)
4. `TranslationCacheEntry` → `Frontend` → `TranslationState` (React state)

---

## State Machines

### Frontend Translation Flow

```
User clicks "اردو میں پڑھیں"
  ↓
Check localStorage cache
  ├─ Cache hit (valid TTL) → [translated] state (instant load)
  └─ Cache miss → [translating] state
       ↓
       Fetch POST /api/translate/chapter
         ├─ Success (200) → Cache in localStorage → [translated] state
         ├─ Timeout (504) → [error] state (TIMEOUT)
         ├─ Rate limit (429) → [error] state (QUOTA_EXCEEDED)
         └─ Other error → [error] state (API_ERROR/NETWORK_ERROR)

User clicks "English میں واپس"
  ↓
[idle] state (clear translatedContent, show original)

User clicks "Restore Original" (when error)
  ↓
Clear cache → [idle] state
```

### Backend Agent Flow

```
Receive TranslationRequest
  ↓
Validate request (chapterId, content length)
  ├─ Invalid → 422 ValidationError
  └─ Valid → Continue
       ↓
       Initialize UrduTranslatorAgent
         ↓
         Create Runner with 60s timeout
           ↓
           Execute translateToUrduSkill
             ├─ Success → Validate markdown → Return TranslationResponse (200)
             ├─ Timeout → Return 504 Gateway Timeout
             ├─ API Error (429) → Return 429 Too Many Requests
             └─ Other error → Return 500 Internal Server Error
```

---

## Validation Rules Summary

### Request Validation (Backend)
```python
# In backend/src/api/translate.py
def validate_translation_request(request: TranslationRequest) -> None:
    if not re.match(r'^[a-z0-9-]+$', request.chapterId):
        raise ValueError("Invalid chapterId format")

    if len(request.originalContent) < 100:
        raise ValueError("Content too short (min 100 chars)")

    if len(request.originalContent) > 50000:
        raise ValueError("Content too long (max 50,000 chars)")

    if not request.originalContent.strip():
        raise ValueError("Content cannot be empty")
```

### Markdown Structure Validation (Backend Skill)
```python
# In backend/src/agents/skills/translate_to_urdu_skill.py
def _validate_markdown_structure(self, original: str, translated: str) -> tuple[bool, str]:
    """
    Ensure translated content preserves markdown structure.
    Returns (is_valid, error_message)
    """
    # Heading count
    original_headings = len(re.findall(r'^#{1,6}\s', original, re.MULTILINE))
    translated_headings = len(re.findall(r'^#{1,6}\s', translated, re.MULTILINE))
    if original_headings != translated_headings:
        return False, f"Heading mismatch: {original_headings} vs {translated_headings}"

    # Code block count
    original_code_blocks = original.count('```')
    translated_code_blocks = translated.count('```')
    if original_code_blocks != translated_code_blocks:
        return False, f"Code block mismatch: {original_code_blocks} vs {translated_code_blocks}"

    # Link count (±10% tolerance)
    original_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', original))
    translated_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', translated))
    if abs(original_links - translated_links) > max(1, original_links * 0.1):
        return False, f"Link count mismatch: {original_links} vs {translated_links}"

    # Length check (±50% tolerance for Urdu script expansion)
    length_ratio = len(translated) / len(original)
    if length_ratio < 0.5 or length_ratio > 1.5:
        return False, f"Length ratio out of bounds: {length_ratio:.2f}"

    return True, ""
```

### Cache Validation (Frontend)
```typescript
// In frontend/src/hooks/useUrduTranslation.ts
function isValidCache(entry: TranslationCacheEntry, currentVersion: number): boolean {
  const age = Date.now() - entry.timestamp;
  const TTL = 7 * 24 * 60 * 60 * 1000; // 7 days in milliseconds

  if (age > TTL) return false;
  if (entry.version !== currentVersion) return false;
  if (!entry.translatedContent || entry.translatedContent.length === 0) return false;

  return true;
}
```

---

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────┐
│ Frontend (React)                                             │
│                                                              │
│  ┌─────────────┐  User clicks button                        │
│  │ UrduButton  │──────────────────────────┐                 │
│  └─────────────┘                          ↓                 │
│                              ┌────────────────────────┐      │
│                              │ useUrduTranslation     │      │
│                              │ Hook                   │      │
│                              └────────────────────────┘      │
│                                     ↓                        │
│                         Check localStorage cache             │
│                                ↙         ↘                   │
│                          Hit               Miss              │
│                           ↓                 ↓                │
│                    Load from cache    Fetch API              │
│                           ↓                 ↓                │
│                      ┌─────────────────────────┐            │
│                      │ TranslationState        │            │
│                      │ status: 'translated'    │            │
│                      └─────────────────────────┘            │
│                                     ↓                        │
│                      ┌─────────────────────────┐            │
│                      │ UrduChapterWrapper      │            │
│                      │ (replaces DOM)          │            │
│                      └─────────────────────────┘            │
└──────────────────────────────────────────────────────────────┘
                              ↑
                              │ TranslationResponse
                              │
┌──────────────────────────────────────────────────────────────┐
│ Backend (FastAPI)                                            │
│                                                              │
│  ┌──────────────────────────────┐                           │
│  │ POST /api/translate/chapter  │                           │
│  └──────────────────────────────┘                           │
│               ↓                                              │
│     Validate TranslationRequest                              │
│               ↓                                              │
│  ┌──────────────────────────────┐                           │
│  │ urdu_translator_agent.py     │                           │
│  │ (OpenAI Agents SDK)          │                           │
│  └──────────────────────────────┘                           │
│               ↓                                              │
│  ┌──────────────────────────────┐                           │
│  │ translate_to_urdu_skill.py   │                           │
│  │ (Skill.execute())            │                           │
│  └──────────────────────────────┘                           │
│               ↓                                              │
│     Call Gemini API (OpenAI-compatible endpoint)            │
│               ↓                                              │
│  ┌──────────────────────────────┐                           │
│  │ Validate markdown structure  │                           │
│  └──────────────────────────────┘                           │
│               ↓                                              │
│     Return TranslationResponse                               │
└──────────────────────────────────────────────────────────────┘
```

---

## Performance Considerations

### Cache Hit Rate Estimation
- **Assumption**: 70% of users view same popular chapters (ROS 2 intro, Jetson setup, etc.)
- **7-day TTL**: Amortizes translation cost over multiple users
- **Expected hit rate**: 60-80% after initial warm-up period

### API Cost Estimation
- **Gemini 2.0 Flash pricing**: ~$0.00005 per 1K input tokens, ~$0.00015 per 1K output tokens
- **Average chapter**: 5,000 words ≈ 6,500 tokens input, 8,000 tokens output (Urdu expansion)
- **Cost per translation**: ~$0.00032 + ~$0.0012 = ~$0.0015 per chapter
- **With 70% cache hit rate**: Effective cost ~$0.00045 per view

### Storage Estimation
- **localStorage limit**: 5-10 MB per domain (browser-dependent)
- **Average translation**: 15-20 KB per chapter (Urdu UTF-8)
- **Capacity**: ~250-500 cached chapters (sufficient for entire textbook)

---

## Security Considerations

### XSS Prevention
- **DOMPurify sanitization** applied before rendering translated content
- Allowed tags: `p`, `h1-h6`, `ul`, `ol`, `li`, `a`, `code`, `pre`, `blockquote`, etc.
- Disallowed: `<script>`, `<iframe>`, `on*` attributes

### Rate Limiting (Future Enhancement)
- **Not implemented in MVP**: No authentication required, so per-IP rate limiting complex
- **Mitigation**: Aggressive caching reduces API calls
- **Future**: Implement IP-based rate limiting (10 translations/hour per IP)

### Cache Poisoning
- **Not a risk**: Cache is local to user's browser
- No shared cache between users in MVP

---

## Testing Scenarios

### Unit Tests
1. Cache validation logic (TTL expiration, version mismatch)
2. Markdown structure validation (heading/code block counts)
3. Error message bilingual formatting

### Integration Tests
1. Full translation flow (request → agent → skill → response)
2. Cache hit/miss scenarios
3. Timeout handling (60-second limit)
4. Error responses (429, 500, 504)

### E2E Tests
1. User clicks button → sees loading spinner → sees Urdu content
2. Cache hit shows instant load (no spinner)
3. User toggles back to English → sees original content
4. Error state shows bilingual message + retry button

---

**Data Model Version**: 1.0
**Last Updated**: 2025-12-07
**Status**: Ready for implementation ✅
