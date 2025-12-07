# Research: Chapter Personalization

**Feature**: 005-chapter-personalization
**Date**: 2025-12-06

## Research Questions

### 1. OpenAI Agents SDK Integration with Gemini API

**Question**: How to use OpenAI Agents SDK with Gemini API via OpenAI-compatible endpoint?

**Decision**: Use Gemini 2.0 Flash via OpenAI-compatible endpoint with OpenAI Agents SDK

**Rationale**:
- Gemini API provides OpenAI-compatible endpoint at `https://generativelanguage.googleapis.com/v1beta/openai/`
- OpenAI Agents SDK can be configured with custom base_url
- Gemini 2.0 Flash (`gemini-2.0-flash-exp`) offers fast, cost-effective generation
- Existing project already uses Gemini (see `backend/.env`)

**Implementation Pattern**:
```python
from openai import OpenAI

client = OpenAI(
    api_key=settings.gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

response = await client.chat.completions.create(
    model="gemini-2.0-flash-exp",
    messages=[...],
    temperature=0.7
)
```

**Alternatives Considered**:
- Anthropic Claude API: Higher cost, different SDK
- Direct OpenAI API: Higher cost than Gemini
- Google Gemini native SDK: Less standardized, requires different agent framework

---

### 2. Docusaurus Chapter Content Extraction

**Question**: How to extract rendered chapter content from Docusaurus for personalization?

**Decision**: Extract from DOM using `document.querySelector('article')`

**Rationale**:
- Docusaurus renders markdown to HTML in `<article>` tags
- DOM extraction captures exactly what user sees (including transformations)
- Avoids re-implementing Docusaurus markdown processing
- Handles Docusaurus plugins (admonitions, tabs, etc.)

**Implementation**:
```typescript
// Extract chapter content
const articleElement = document.querySelector('article');
const originalContent = articleElement?.innerHTML || '';

// Send to backend
await personalizeChapter({
  chapterId: 'chapter-1',
  originalContent,
  userProfile
});
```

**Challenges**:
- Need to preserve markdown structure for backend processing
- Solution: Backend converts HTML back to markdown-friendly format

**Alternatives Considered**:
- Read markdown source files: Doesn't capture Docusaurus transformations
- Use Docusaurus API: No public API for content extraction
- Server-side extraction: Violates "no markdown modification" constraint

---

### 3. DOM Replacement Strategy

**Question**: How to replace chapter content without modifying markdown files?

**Decision**: Client-side DOM replacement with `innerHTML` and DOMPurify sanitization

**Rationale**:
- Fast (no page reload)
- Preserves Docusaurus navigation and state
- No server-side file writes (session-based personalization)
- Security via DOMPurify HTML sanitization

**Implementation**:
```typescript
import DOMPurify from 'dompurify';

// Sanitize and replace
const sanitized = DOMPurify.sanitize(personalizedContent);
const contentContainer = document.querySelector('.markdown');
if (contentContainer) {
  contentContainer.innerHTML = sanitized;
}
```

**Security Considerations**:
- Always sanitize HTML before insertion (XSS prevention)
- Validate backend response structure
- Rate limit personalization requests

**Alternatives Considered**:
- React component replacement: More complex, state management issues
- Markdown re-parsing: Performance overhead
- Server-side pre-rendering: Requires file writes (violates constraint)

---

### 4. Personalization Prompt Engineering

**Question**: How to structure prompts for educational content personalization?

**Decision**: Context-aware prompts based on user profile dimensions

**Prompt Structure**:
```
System: "You are an expert technical writer specializing in educational content..."

User:
"Personalize this chapter for a {experience} student with {hardware}.

Guidelines:
1. Experience Level:
   - Beginner: More explanations, avoid jargon
   - Intermediate: Balance theory/practice
   - Advanced: Focus on edge cases, optimizations

2. Hardware:
   - hasRTX: Mention cloud alternatives
   - hasJetson: Emphasize edge deployment
   - hasRobot: Include safety warnings
   - No hardware: Focus on simulation

3. Constraints:
   - Preserve ALL information
   - Maintain markdown formatting
   - Keep code accurate

Original: {chapter_content}
Personalized:"
```

**Rationale**:
- Clear instructions ensure consistent personalization
- Constraint reminders prevent content loss
- Hardware-specific adaptations increase relevance

**Best Practices**:
- Use temperature 0.7 (balance creativity and consistency)
- Validate output length (±50% of original)
- Check markdown structure preservation

---

### 5. Authentication & Authorization

**Question**: How to protect personalization endpoint?

**Decision**: JWT-based authentication with Better-Auth

**Rationale**:
- Better-Auth already integrated (Constitution Principle VIII)
- JWT tokens enable stateless authentication
- Horizontal scaling without session storage

**Flow**:
1. User logs in via Better-Auth → JWT issued
2. Frontend stores JWT (localStorage/cookie)
3. Frontend includes JWT in `Authorization: Bearer {token}` header
4. Backend verifies JWT signature and expiration
5. Backend extracts user_email from JWT payload
6. Backend proceeds with personalization

**Error Handling**:
```python
async def get_current_user(authorization: Optional[str] = Header(None)):
    if not authorization:
        raise HTTPException(401, "Not authenticated")

    token = authorization.replace("Bearer ", "")
    payload = AuthService.verify_token(token)

    if not payload:
        raise HTTPException(401, "Invalid token")

    return payload.get("email")
```

---

### 6. Caching Strategy

**Question**: Should personalized content be cached?

**Decision**: Client-side caching with 7-day TTL

**Rationale**:
- Reduces backend load
- Faster UX for repeated chapter visits
- LocalStorage sufficient for MVP
- Per-user, per-chapter, per-profile cache key

**Implementation**:
```typescript
const cacheKey = `personalized_${chapterId}_${userEmail}_${profileHash}`;

// Check cache
const cached = localStorage.getItem(cacheKey);
if (cached) {
  const { content, timestamp } = JSON.parse(cached);
  const age = Date.now() - timestamp;
  if (age < 7 * 24 * 60 * 60 * 1000) {  // 7 days
    return content;
  }
}

// Store in cache
localStorage.setItem(cacheKey, JSON.stringify({
  content: personalizedContent,
  timestamp: Date.now()
}));
```

**Future Enhancements**:
- Backend caching with Redis (for shared caching across users with same profile)
- Database persistence (for permanent personalization)

---

### 7. Timeout Configuration

**Question**: What timeout threshold for personalization requests?

**Decision**: 60 seconds (from spec clarification)

**Rationale**:
- Typical chapter (2000-5000 words) personalizes in 20-40 seconds
- 60 seconds provides buffer for longer chapters
- Gemini 2.0 Flash is fast enough for this constraint
- Aligns with success criteria SC-002

**Implementation**:
```python
# Backend
async with asyncio.timeout(60):
    result = await runner.run(agent, skill_call)

# Frontend
const controller = new AbortController();
setTimeout(() => controller.abort(), 60000);
await fetch('/api/personalize/chapter', { signal: controller.signal });
```

**Error Handling**:
- 504 Gateway Timeout → "Personalization took too long"
- Offer retry for shorter chapters
- Log timeout incidents for monitoring

---

## Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| Backend Framework | FastAPI | Latest | Existing choice, async support |
| AI Orchestration | OpenAI Agents SDK | Latest | Structured agent framework |
| LLM | Gemini 2.0 Flash | gemini-2.0-flash-exp | Fast, cost-effective, existing setup |
| Authentication | Better-Auth (JWT) | Latest | Existing integration |
| Frontend Framework | Docusaurus | v3.x | Existing choice |
| Frontend UI | React 18 + TailwindCSS | Latest | Existing choices |
| State Management | React Hooks + Context | Built-in | Sufficient for feature |
| HTTP Client | fetch API | Native | No additional dependency |
| HTML Sanitization | DOMPurify | Latest | XSS prevention |
| Testing (Backend) | pytest | Latest | Python standard |
| Testing (Frontend) | Jest + React Testing Library | Latest | React standard |

---

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Gemini API timeout | Medium | High | 60s timeout, retry mechanism |
| Content accuracy loss | Low | Critical | Validation checks in skill |
| XSS via malicious content | Low | High | DOMPurify sanitization |
| Rate limit abuse | Medium | Medium | Per-user rate limiting (10/min) |
| Cache poisoning | Low | Low | User-specific cache keys |
| JWT token theft | Low | High | HTTPS only, secure cookie flags |

---

## Open Research Questions

1. **Should we implement backend caching (Redis)?**
   - Pro: Reduces Gemini API calls for common profiles
   - Con: Additional infrastructure complexity
   - **Decision**: Defer to post-MVP

2. **Should personalization persist across sessions?**
   - Pro: Better UX, no re-personalization needed
   - Con: Database storage, versioning complexity
   - **Decision**: Session-based for MVP (per spec)

3. **How to handle chapter updates after personalization?**
   - Issue: Original chapter may be edited, personalized version outdated
   - **Decision**: Cache invalidation via timestamp check (future enhancement)

---

## References

- [OpenAI Agents SDK Documentation](https://platform.openai.com/docs/agents)
- [Gemini API OpenAI Compatibility](https://ai.google.dev/gemini-api/docs/openai)
- [Docusaurus Theming](https://docusaurus.io/docs/swizzling)
- [DOMPurify Documentation](https://github.com/cure53/DOMPurify)
- [Better-Auth Documentation](https://better-auth.com/docs)
