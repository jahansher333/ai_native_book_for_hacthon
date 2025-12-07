# Quickstart: Chapter Personalization

**Feature**: 005-chapter-personalization
**Date**: 2025-12-06

## Overview

This guide helps developers understand and implement the Chapter Personalization feature in 10 minutes.

---

## What This Feature Does

Allows logged-in users to personalize any textbook chapter by clicking a button. The system:
1. Extracts the chapter content from Docusaurus DOM
2. Sends content + user profile to backend
3. Backend calls OpenAI Agents SDK with Gemini API to rewrite chapter
4. Frontend replaces chapter DOM with personalized content
5. No markdown files are modified (session-based personalization)

**User Flow**:
```
User clicks "Personalize for Me"
  ↓
Check if logged in
  ├─ Not logged in → Show "Please log in" message
  └─ Logged in → Send request to backend
       ↓
Backend personalizes with OpenAI Agents SDK + Gemini
       ↓
Frontend displays personalized chapter
```

---

## Prerequisites

- Backend running on `http://localhost:8000`
- Frontend running on `http://localhost:3000`
- User account created with completed profile (experience, hardware)
- Gemini API key configured in `backend/.env`

---

## Key Files

### Backend
- `backend/src/agents/personalizer_agent.py` - Agent orchestrator
- `backend/src/agents/skills/personalize_chapter_skill.py` - Personalization logic
- `backend/src/api/personalize.py` - API endpoint
- `backend/src/models/user.py` - User profile model

### Frontend
- `frontend/src/components/Personalize/PersonalizeButton.tsx` - Button UI
- `frontend/src/components/Personalize/ChapterWrapper.tsx` - DOM wrapper
- `frontend/src/hooks/usePersonalization.ts` - Personalization logic
- `frontend/src/services/personalizationService.ts` - API client

---

## Quick Test

### 1. Start Backend

```bash
cd backend
source venv/bin/activate  # or venv\Scripts\activate on Windows
uvicorn src.main:app --reload
```

### 2. Start Frontend

```bash
cd frontend
npm start
```

### 3. Test with curl

```bash
# Get JWT token first (log in via frontend or use existing token)
export TOKEN="your-jwt-token-here"

# Test personalization endpoint
curl -X POST http://localhost:8000/api/personalize/chapter \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "test-chapter",
    "originalContent": "# Test Chapter\n\nThis is a test chapter about ROS 2...",
    "userProfile": {
      "experience": "beginner",
      "hasRTX": false,
      "hasJetson": true,
      "hasRobot": false
    }
  }'
```

Expected response:
```json
{
  "success": true,
  "personalizedContent": "# Test Chapter (Personalized for You)\n\n...",
  "metadata": {
    "generationTime": 12.5,
    "cached": false
  }
}
```

### 4. Test in Browser

1. Navigate to `http://localhost:3000`
2. Log in (or sign up if no account)
3. Go to any chapter
4. Click "Personalize for Me" button at the top
5. Wait for personalization (loading overlay appears)
6. Personalized content replaces original

---

## Implementation Checklist

### Backend Tasks

- [ ] Create `backend/src/agents/` directory
- [ ] Implement `personalizer_agent.py`:
  - Initialize OpenAI Agents SDK Agent
  - Configure Gemini API endpoint
  - Create Runner for agent execution
- [ ] Implement `personalizeChapterSkill`:
  - Build personalization prompt
  - Call Gemini API
  - Validate output
- [ ] Fix import in `api/personalize.py`:
  ```python
  from ..agents.personalizer_agent import personalize_chapter_content
  ```
- [ ] Add tests:
  - `tests/test_personalizer_agent.py`
  - `tests/test_personalize_api.py`

### Frontend Tasks

- [ ] Create `personalizationService.ts`:
  ```typescript
  export async function personalizeChapter(request: PersonalizeChapterRequest) {
    const token = localStorage.getItem('auth_token');
    const response = await fetch('/api/personalize/chapter', {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(request)
    });
    return response.json();
  }
  ```
- [ ] Verify `usePersonalization` hook works with API
- [ ] Verify `ChapterWrapper` is integrated in all chapters
- [ ] Add tests:
  - `tests/PersonalizeButton.test.tsx`
  - `tests/usePersonalization.test.ts`

---

## Common Issues & Solutions

### Issue: "Not authenticated" error

**Symptom**: 401 response from `/api/personalize/chapter`

**Solution**:
1. Check JWT token is present: `localStorage.getItem('auth_token')`
2. Check token format: Should be `Bearer <token>`
3. Verify token hasn't expired (7-day default)
4. Re-login if needed

### Issue: Personalization times out

**Symptom**: 504 Gateway Timeout after 60 seconds

**Solution**:
1. Check chapter length (should be < 5000 words)
2. Try shorter chapter first
3. Check Gemini API key is valid
4. Check network connectivity

### Issue: Personalized content looks broken

**Symptom**: Missing formatting, garbled HTML

**Solution**:
1. Check DOMPurify is sanitizing HTML
2. Verify markdown structure in response
3. Check browser console for errors
4. Test with simple chapter first

### Issue: Button doesn't appear on chapter

**Symptom**: No "Personalize for Me" button visible

**Solution**:
1. Verify `ChapterWrapper` is wrapping chapters
2. Check Docusaurus theme swizzling is correct
3. Inspect DOM for PersonalizeButton component
4. Check CSS/Tailwind classes are loading

---

## Testing Strategy

### Manual Testing

**Test Case 1: Happy Path**
1. Log in as user with profile
2. Navigate to any chapter
3. Click "Personalize for Me"
4. Verify loading state appears
5. Verify personalized content displays
6. Verify badge shows "Personalized"

**Test Case 2: Not Logged In**
1. Log out (or use incognito)
2. Navigate to chapter
3. Click "Personalize for Me"
4. Verify "Please log in" message appears

**Test Case 3: Cache Hit**
1. Personalize a chapter
2. Navigate away and return
3. Verify personalized content loads instantly
4. Verify badge shows "From Cache"

**Test Case 4: Error Handling**
1. Stop backend server
2. Click "Personalize for Me"
3. Verify error message displays
4. Verify retry button appears

### Automated Testing

**Backend Tests** (`pytest`):
```bash
cd backend
pytest tests/test_personalizer_agent.py -v
pytest tests/test_personalize_api.py -v
```

**Frontend Tests** (Jest):
```bash
cd frontend
npm test usePersonalization
npm test PersonalizeButton
```

---

## Performance Tips

1. **Enable Caching**: LocalStorage cache reduces API calls
2. **Rate Limiting**: Max 10 requests/minute per user
3. **Timeout**: 60-second timeout prevents hung requests
4. **Monitoring**: Watch logs for personalization times

---

## Environment Variables

Add to `backend/.env`:
```bash
# Gemini API
GEMINI_API_KEY=your_gemini_api_key_here
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
MODEL_NAME=gemini-2.0-flash-exp

# JWT Auth
JWT_SECRET=your_jwt_secret_here
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7
```

---

## Next Steps

After basic implementation works:

1. **Add Monitoring**: Track personalization requests, errors, latency
2. **Optimize Prompts**: Refine personalization prompts based on user feedback
3. **Add Analytics**: Track which chapters get personalized most
4. **Consider Backend Caching**: Redis for frequently personalized chapters
5. **A/B Testing**: Compare engagement for personalized vs. original content

---

## Resources

- [OpenAI Agents SDK Docs](https://platform.openai.com/docs/agents)
- [Gemini API Docs](https://ai.google.dev/gemini-api/docs)
- [Docusaurus Theming Guide](https://docusaurus.io/docs/swizzling)
- [Better-Auth Integration](https://better-auth.com/docs)

---

## Support

For issues or questions:
- Check logs: `backend/logs/` and browser console
- Review error messages carefully
- Test with minimal example first
- Consult full implementation plan: `specs/005-chapter-personalization/plan.md`
