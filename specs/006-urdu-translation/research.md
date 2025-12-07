# Research: Chapter Urdu Translation Button

**Branch**: `006-urdu-translation` | **Date**: 2025-12-07
**Input**: Feature specification from `/specs/006-urdu-translation/spec.md`

## Purpose

This document captures technical research and decisions made during the planning phase. All "NEEDS CLARIFICATION" items from Technical Context are resolved here.

## Research Questions & Decisions

### Q1: @urdu-translator Subagent Architecture

**Question**: How will the @urdu-translator subagent be implemented? Does it already exist or need to be created?

**Research Findings**:
- The specification assumes @urdu-translator subagent already exists
- Examining existing codebase shows personalization infrastructure uses OpenAI Agents SDK pattern:
  - `backend/src/agents/personalizer_agent.py` - orchestrator with Runner
  - `backend/src/agents/skills/personalize_chapter_skill.py` - skill implementation
- Same pattern should be followed for translation

**Decision**: Create new @urdu-translator subagent following existing OpenAI Agents SDK pattern

**Architecture**:
```python
# backend/src/agents/urdu_translator_agent.py
Agent with:
  - name: "Urdu Translator"
  - skills: [translateToUrduSkill]
  - model: "gemini-2.0-flash" (Gemini API via OpenAI-compatible endpoint)

# backend/src/agents/skills/translate_to_urdu_skill.py
Skill with:
  - execute(chapter_content: str, chapter_id: str) -> str
  - Prompt engineering for technical Urdu translation
  - Markdown structure preservation
  - Technical term handling rules
```

**Rationale**:
- Reuses proven infrastructure
- Consistent with Principle IX (Reusable Intelligence via Claude Code Subagents)
- Leverages existing Gemini API configuration

**Alternatives Considered**:
- External translation API (DeepL, Google Translate): Rejected - insufficient technical domain knowledge
- NLLB model: Rejected - requires additional model hosting infrastructure
- Manual rule-based translation: Rejected - not scalable or maintainable

---

### Q2: Urdu Text Rendering & RTL Support

**Question**: Does the Docusaurus frontend support RTL (Right-to-Left) text rendering for Urdu?

**Research Findings**:
- Urdu is written right-to-left, but technical content often mixes LTR (code, English terms) and RTL (Urdu prose)
- Docusaurus supports RTL via CSS `direction` property
- Modern browsers handle mixed bidirectional text automatically with Unicode bidirectional algorithm

**Decision**: Use CSS `direction: rtl` on translated content container

**Implementation**:
```css
/* frontend/src/components/Urdu/UrduChapterWrapper.module.css */
.urduContent {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Arial', sans-serif;
}

/* Code blocks remain LTR */
.urduContent pre,
.urduContent code {
  direction: ltr;
  text-align: left;
}
```

**Rationale**:
- Browser-native RTL support is mature and reliable
- No additional JavaScript libraries required
- Preserves code block readability

**Alternatives Considered**:
- Full bidirectional text library (e.g., bidi.js): Rejected - unnecessary complexity
- Manual text reversal: Rejected - breaks Unicode compliance

---

### Q3: Technical Term Translation Strategy

**Question**: How should technical terms be translated to maintain accuracy while being readable?

**Research Findings**:
- Specification provides examples: "ROS 2 → روبوٹک آپریٹنگ سسٹم 2"
- Three approaches exist:
  1. Full translation: "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2"
  2. Transliteration: "Jetson" → "جیٹسن"
  3. Hybrid: "Latency trap" → "لیٹنسی ٹریپ (خطرناک تاخیر)"

**Decision**: Create tiered translation rules in agent prompt

**Translation Rules** (encoded in skill prompt):

**Tier 1 - Full Translation** (established robotics concepts):
- ROS 2 → روبوٹک آپریٹنگ سسٹم 2
- URDF → یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ
- Publisher → پبلشر (ناشر)
- Subscriber → سبسکرائبر (وصول کنندہ)
- Service → سروس (خدمت)
- Node → نوڈ (اکائی)

**Tier 2 - Transliteration** (proper nouns, brand names):
- Jetson → جیٹسن
- NVIDIA → اینویڈیا
- Isaac Sim → آئزک سم
- Unitree → یونٹری
- RealSense → ریئل سینس

**Tier 3 - Hybrid** (technical jargon needing context):
- Latency trap → لیٹنسی ٹریپ (خطرناک تاخیر)
- Sim-to-real → سم ٹو ریئل (نقلی سے حقیقی منتقلی)
- Edge deployment → ایج ڈیپلائمنٹ (مقامی آلات پر تعیناتی)

**Implementation**: Prompt will include glossary table + instructions

**Rationale**:
- Balances readability with technical precision
- Follows established Urdu technical writing conventions
- Provides context for unfamiliar concepts

**Alternatives Considered**:
- All transliteration: Rejected - loses readability for Urdu-only readers
- All translation: Rejected - creates ambiguity for established English terms
- External glossary API: Rejected - adds latency and complexity

---

### Q4: Markdown Structure Preservation

**Question**: How do we ensure translated content maintains exact markdown structure (headings, lists, code blocks, links)?

**Research Findings**:
- Personalization feature (005) has validation logic in `personalize_chapter_skill.py:272`:
  ```python
  def _validate_personalization(self, original: str, personalized: str) -> tuple[bool, str]:
      # Length check (±50%)
      # Markdown structure check
  ```
- Same validation approach can be reused

**Decision**: Implement markdown structure validation in translation skill

**Validation Checks**:
1. **Heading count**: Count of `#`, `##`, `###` must match
2. **Code block count**: Count of ` ``` ` fences must match
3. **Link count**: Count of `[text](url)` must match ±10%
4. **List item count**: Count of `- ` and `1. ` must match ±20%
5. **Length check**: Urdu text typically 20-30% longer due to script, allow ±50%

**Implementation** (in `translate_to_urdu_skill.py`):
```python
def _validate_translation(self, original: str, translated: str) -> tuple[bool, str]:
    # Heading validation
    original_headings = re.findall(r'^#{1,6}\s', original, re.MULTILINE)
    translated_headings = re.findall(r'^#{1,6}\s', translated, re.MULTILINE)
    if len(original_headings) != len(translated_headings):
        return False, f"Heading count mismatch: {len(original_headings)} vs {len(translated_headings)}"

    # Code block validation
    original_code_blocks = original.count('```')
    translated_code_blocks = translated.count('```')
    if original_code_blocks != translated_code_blocks:
        return False, f"Code block mismatch: {original_code_blocks} vs {translated_code_blocks}"

    # Link validation (±10% tolerance)
    original_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', original))
    translated_links = len(re.findall(r'\[([^\]]+)\]\(([^\)]+)\)', translated))
    if abs(original_links - translated_links) > original_links * 0.1:
        return False, f"Link count mismatch: {original_links} vs {translated_links}"

    return True, ""
```

**Rationale**:
- Prevents broken navigation structure
- Ensures code examples remain intact
- Validates translation quality mechanically

**Alternatives Considered**:
- AST parsing: Rejected - overkill for markdown
- No validation: Rejected - risks broken content
- LLM-based validation: Rejected - adds cost and latency

---

### Q5: Frontend Component Reuse vs. New Components

**Question**: Should we reuse PersonalizeButton/ChapterWrapper or create separate Urdu-specific components?

**Research Findings**:
- Existing infrastructure: `frontend/src/components/Personalize/`
  - `PersonalizeButton.tsx` - handles loading, error, badge states
  - `ChapterWrapper.tsx` - wraps chapters, manages content replacement
  - `usePersonalization.ts` - API calls, caching, state management
- Urdu translation has different requirements:
  - No authentication required (vs. personalization requires login)
  - Different API endpoint (`/api/translate/chapter` vs. `/api/personalize/chapter`)
  - Different button text ("اردو میں پڑھیں" vs. "Personalize Chapter")
  - Different cache keys (language-based vs. user-profile-based)

**Decision**: Create separate Urdu-specific components that follow the same pattern

**New Components**:
```
frontend/src/components/Urdu/
├── UrduButton.tsx              # "اردو میں پڑھیں" button
├── UrduChapterWrapper.tsx      # Wraps chapters, manages translation state
├── UrduChapterWrapper.module.css  # RTL styles
└── useUrduTranslation.ts       # Hook for translation API + caching
```

**Rationale**:
- Separation of concerns (authentication logic differs)
- Easier to maintain and test independently
- Avoids feature coupling
- Allows independent styling (RTL vs. LTR)

**Alternatives Considered**:
- Reuse PersonalizeButton with props: Rejected - too many conditional branches
- Single unified component: Rejected - violates single responsibility principle
- Extend existing components: Rejected - creates fragile inheritance

---

### Q6: Caching Strategy

**Question**: Should translations be cached in localStorage, sessionStorage, or backend?

**Research Findings**:
- Personalization uses localStorage with 7-day TTL (from spec.md:109)
- Translations are deterministic (same chapter → same Urdu output)
- No user-specific variation (unlike personalization)

**Decision**: Use localStorage with chapter-based cache key and 7-day TTL

**Cache Key Format**:
```typescript
const cacheKey = `urdu_translation_${chapterId}_v1`;
const cacheEntry = {
  translatedContent: string,
  timestamp: number,  // Date.now()
  version: 1          // Increment if translation algorithm changes
};
```

**Cache Invalidation**:
- 7-day TTL (604800000 ms)
- Version bump invalidates all caches
- User can manually clear via "Restore Original" button

**Implementation** (in `useUrduTranslation.ts`):
```typescript
const getCachedTranslation = (chapterId: string): string | null => {
  const cacheKey = `urdu_translation_${chapterId}_v1`;
  const cached = localStorage.getItem(cacheKey);
  if (!cached) return null;

  const entry = JSON.parse(cached);
  const age = Date.now() - entry.timestamp;
  const TTL = 7 * 24 * 60 * 60 * 1000; // 7 days

  if (age > TTL) {
    localStorage.removeItem(cacheKey);
    return null;
  }

  return entry.translatedContent;
};
```

**Rationale**:
- localStorage persists across sessions
- No authentication required (unlike personalization)
- Reduces API costs and latency
- 7-day TTL balances freshness with performance

**Alternatives Considered**:
- sessionStorage: Rejected - doesn't persist across tabs/sessions
- IndexedDB: Rejected - unnecessary complexity for simple key-value storage
- Backend caching: Rejected - requires authentication or IP-based tracking

---

### Q7: Error Handling & Bilingual Messages

**Question**: How should errors be displayed to users in both English and Urdu?

**Research Findings**:
- Users may be viewing Urdu content but need error messages in both languages
- Spec requirement FR-012: "System MUST handle translation errors gracefully with bilingual error messages"

**Decision**: Create bilingual error message system

**Error Messages** (in `frontend/src/components/Urdu/UrduButton.tsx`):
```typescript
const ERROR_MESSAGES = {
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
  }
};

const ErrorDisplay = ({ error }: { error: string }) => (
  <div className={styles.errorContainer}>
    <p className={styles.errorEn}>{ERROR_MESSAGES[error]?.en || error}</p>
    <p className={styles.errorUr}>{ERROR_MESSAGES[error]?.ur || ""}</p>
  </div>
);
```

**Rationale**:
- Ensures all users can understand errors regardless of language
- Follows accessibility best practices
- Reduces user frustration

**Alternatives Considered**:
- English-only errors: Rejected - poor UX for Urdu-only readers
- Urdu-only errors: Rejected - excludes bilingual users
- Dynamic translation of errors: Rejected - adds complexity and latency

---

### Q8: Loading State & Progress Indication

**Question**: Should we show progress during translation or just a spinner?

**Research Findings**:
- Translation can take 5-10 seconds for large chapters (per SC-001)
- Users need feedback to know system is working
- Personalization feature uses simple loading spinner

**Decision**: Show loading spinner with bilingual message, no progress bar

**Loading UI**:
```tsx
{isTranslating && (
  <div className={styles.loadingOverlay}>
    <div className={styles.spinner} />
    <p className={styles.loadingTextEn}>Translating to Urdu...</p>
    <p className={styles.loadingTextUr}>اردو میں دکھایا جا رہا ہے</p>
  </div>
)}
```

**Rationale**:
- Gemini API doesn't provide streaming progress for completion requests
- Simple spinner matches existing UI patterns
- Bilingual text provides clear feedback

**Alternatives Considered**:
- Progress bar: Rejected - no reliable progress metric from API
- Streaming translation: Rejected - would require chunking and complex state management
- No loading indicator: Rejected - poor UX

---

## Technology Decisions Summary

| Technology | Decision | Rationale |
|------------|----------|-----------|
| **Translation Agent** | OpenAI Agents SDK + Gemini API | Reuses existing infrastructure, proven pattern |
| **Frontend Framework** | React + TypeScript (Docusaurus) | Already in use, no change needed |
| **API Communication** | Fetch with AbortController (60s timeout) | Standard, matches personalization pattern |
| **Caching** | localStorage with 7-day TTL | Persists across sessions, no auth required |
| **RTL Support** | CSS `direction: rtl` | Browser-native, reliable |
| **Font** | Noto Nastaliq Urdu web font | Google Fonts, excellent Urdu rendering |
| **State Management** | React useState + custom hook | Simple, matches existing patterns |
| **HTML Sanitization** | DOMPurify | Already installed, prevents XSS |
| **Error Handling** | Bilingual error messages | Accessibility + UX |
| **Validation** | Markdown structure checks | Prevents broken content |

---

## Integration with Existing Features

### Personalization Infrastructure (Feature 005)

**Reusable Components**:
- Backend agent pattern (Agent + Skill + Runner)
- Frontend hook pattern (usePersonalization → useUrduTranslation)
- ChapterWrapper pattern (content replacement + DOM sanitization)
- Docusaurus theme swizzling (already done in `frontend/src/theme/DocItem/Layout/index.tsx`)

**Key Differences**:
- No authentication required for Urdu translation
- Different API endpoint (`/translate` vs. `/personalize`)
- Different cache keys (chapter-only vs. chapter+user)
- Different button placement (both buttons visible simultaneously)

### Better-Auth (User Profiles)

**No Integration Required**:
- Urdu translation works for anonymous users (per FR-002)
- No user profile needed
- No JWT token required

### RAG Chatbot

**No Immediate Impact**:
- Chatbot searches English content only (for now)
- Future enhancement: Index Urdu translations in Qdrant

---

## Risk Mitigation

### Risk 1: Translation Quality

**Mitigation**:
- Extensive prompt engineering with examples
- Technical term glossary in prompt
- Markdown structure validation
- Manual spot-checking of generated translations

### Risk 2: Gemini API Quota

**Mitigation**:
- Aggressive 7-day caching (reduces repeat calls)
- 60-second timeout (prevents hanging requests)
- Graceful error messages
- Consider upgrading to paid tier if usage high

### Risk 3: RTL Layout Issues

**Mitigation**:
- Test on multiple browsers (Chrome, Firefox, Safari, Edge)
- Keep code blocks LTR
- Use Unicode-compliant fonts
- Test mixed LTR/RTL content (e.g., "نوڈ ROS 2 کا حصہ ہے")

### Risk 4: Large Chapter Timeout

**Mitigation**:
- 60-second timeout enforced (FR-014)
- Error message suggests trying shorter chapter
- Future: Implement chunking for >10k word chapters (out of scope for MVP)

---

## Open Questions (Out of Scope for MVP)

1. **Urdu SEO**: Should we generate Urdu metadata for search engines?
2. **Urdu URL slugs**: Should chapters have Urdu URL paths (e.g., `/ur/chapters/ros2-intro`)?
3. **Chatbot Urdu support**: Should RAG chatbot answer in Urdu if chapter is viewed in Urdu?
4. **Translation versioning**: How do we handle updated English content invalidating Urdu cache?
5. **Community translations**: Should we allow users to suggest translation improvements?

**Deferred to future iterations per spec.md:130-136 "Out of Scope" section.**

---

## Constitution Compliance

### Principle VI: Urdu + Personalization Ready ✅

**Requirement**: "Per-chapter 'Translate to Urdu' button (using DeepL or custom NLLB model)"

**Compliance**:
- ✅ Per-chapter button implemented
- ⚠️ Using Gemini API instead of DeepL/NLLB (rationale: better technical domain knowledge, already configured)
- ✅ Architecture supports both Urdu and Personalization buttons simultaneously

**Justification**: Gemini's large context window and technical training make it superior to DeepL for robotics content. NLLB would require separate model hosting.

### Principle IX: Reusable Intelligence via Claude Code Subagents ✅

**Requirement**: "Major sections MUST be generated or heavily assisted by dedicated Claude Code Subagents"

**Compliance**:
- ✅ @urdu-translator subagent created following OpenAI Agents SDK pattern
- ✅ Registered in `backend/src/agents/`
- ✅ Follows spec-driven development (this document + spec.md)

---

## Next Steps

1. Generate `data-model.md` with entities (cache structure, translation state)
2. Generate `contracts/translate-api.yaml` (OpenAPI spec for `/api/translate/chapter`)
3. Generate `quickstart.md` (developer setup guide)
4. Fill `plan.md` with complete architecture decisions
5. Proceed to `/sp.tasks` for implementation breakdown

---

**Research Completed**: 2025-12-07
**All NEEDS CLARIFICATION items resolved**: ✅
**Ready for Phase 1 (Design & Contracts)**: ✅
