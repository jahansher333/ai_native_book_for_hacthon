# Implementation Plan: Per-Chapter "Personalize for Me" Button

**Feature ID:** `personalize-button`
**Spec Version:** 1.0
**Plan Version:** 1.0
**Created:** 2025-12-06
**Estimated Effort:** 3-4 days (1 developer)

---

## 1. Executive Summary

### 1.1 Implementation Overview
Build a React-based personalization system that wraps every MDX chapter with a `<ChapterWrapper>` component. When authenticated users click "Personalize for Me", the system sends chapter content + user profile to the existing `@personalizer` subagent (from 8-agent system), receives rewritten content, and dynamically replaces the chapter using React state management.

### 1.2 Tech Stack (All Existing)
- ✅ **Frontend Framework:** React 18.x (already in use)
- ✅ **Auth Context:** Better-Auth `useUserProfile()` hook (already implemented)
- ✅ **AI Agent:** `@personalizer` subagent (already exists from 8-agent system)
- ✅ **Content Format:** MDX v2 (already used for all chapters)
- ✅ **State Management:** React.useState + useEffect (built-in)
- ✅ **Styling:** Tailwind CSS (already configured) + custom.css
- ✅ **Backend:** FastAPI (already running on port 8000)
- ✅ **Database:** Neon DB with Better-Auth profile schema (already set up)

### 1.3 Key Design Decisions

**Decision 1: Component Architecture**
- **Choice:** Single `<ChapterWrapper>` component that wraps all MDX chapters
- **Rationale:** DRY principle, centralized logic, automatic coverage of all chapters
- **Alternative Rejected:** Manual button insertion in each chapter (too much duplication)

**Decision 2: State Management**
- **Choice:** React.useState for content + localStorage for caching
- **Rationale:** Simple, built-in, no extra dependencies, predictable
- **Alternative Rejected:** Redux/Zustand (overkill for this feature)

**Decision 3: Content Replacement Strategy**
- **Choice:** Dynamic MDX rendering with dangerouslySetInnerHTML (sanitized)
- **Rationale:** Instant updates, no page reload, smooth UX
- **Alternative Rejected:** Full page reload (poor UX, loses scroll position)

**Decision 4: AI Integration**
- **Choice:** Use existing `@personalizer` subagent via FastAPI endpoint
- **Rationale:** Already built, tested, and integrated in 8-agent system
- **Alternative Rejected:** Build new AI integration (waste of time)

**Decision 5: Caching Strategy**
- **Choice:** localStorage with composite key: `personalized_${chapterId}_${userId}_${profileHash}`
- **Rationale:** Fast, persistent across sessions, automatic expiry
- **Alternative Rejected:** IndexedDB (too complex), sessionStorage (loses on tab close)

---

## 2. Architecture Design

### 2.1 Component Hierarchy

```
App (Docusaurus)
└── MDXComponents Provider
    └── ChapterWrapper (NEW)
        ├── PersonalizeButton (NEW)
        │   ├── Tooltip
        │   ├── LoadingSpinner
        │   └── ErrorMessage
        ├── PersonalizationBadge (NEW)
        │   └── ResetButton
        └── Chapter Content (MDX)
            └── [Dynamic content based on personalization state]
```

### 2.2 Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    User Clicks "Personalize"                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  ChapterWrapper Component                                    │
│  1. Check localStorage cache                                 │
│     └─→ Cache HIT? → Load instantly, skip to step 7         │
│  2. Check authentication (useUserProfile)                    │
│  3. Fetch user profile from context                          │
│  4. Show loading overlay + disable button                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Frontend → Backend API Call                                 │
│  POST /api/personalize/chapter                               │
│  {                                                            │
│    chapterId: "01-intro/ros2-basics",                        │
│    originalContent: "# ROS 2 Basics\n...",                   │
│    userProfile: {                                             │
│      experience: "beginner",                                  │
│      hasRTX: true,                                            │
│      hasJetson: false,                                        │
│      hasRobot: false                                          │
│    }                                                          │
│  }                                                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Backend: personalize.py Endpoint                            │
│  1. Verify JWT token                                         │
│  2. Validate request payload                                 │
│  3. Call @personalizer subagent                              │
│     └─→ Pass: original content + user profile                │
│  4. Receive personalized MDX                                 │
│  5. Validate MDX syntax                                      │
│  6. Return response with metadata                            │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  @personalizer Subagent (Existing from 8-agent system)       │
│  1. Parse user profile                                       │
│  2. Apply experience-level transformations:                  │
│     • Beginner: Add explanations, analogies, heavy comments  │
│     • Intermediate: Balanced content                         │
│     • Advanced: Concise, optimization tips, low-level        │
│  3. Apply hardware-specific adjustments:                     │
│     • RTX GPU: Local Isaac Sim, GPU optimization             │
│     • Jetson: Edge deployment, inference optimization        │
│     • No Jetson: Cloud training suggestions                  │
│  4. Maintain MDX structure and accuracy                      │
│  5. Return personalized MDX string                           │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  ChapterWrapper Component (Response Handling)                │
│  7. Receive personalized content                             │
│  8. Save to localStorage cache                               │
│  9. Update React state → triggers re-render                  │
│  10. Hide loading overlay                                    │
│  11. Show PersonalizationBadge                               │
│  12. Scroll to top of chapter smoothly                       │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 File Structure (New Files Only)

```
ai_robotics_book/
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   └── Personalize/
│   │   │       ├── ChapterWrapper.tsx          # Main wrapper component
│   │   │       ├── PersonalizeButton.tsx       # Button with loading states
│   │   │       ├── PersonalizationBadge.tsx    # Profile badge + reset
│   │   │       ├── LoadingOverlay.tsx          # Spinner + message
│   │   │       └── personalize.module.css      # Tailwind + custom styles
│   │   ├── hooks/
│   │   │   └── usePersonalization.ts           # Custom hook for logic
│   │   ├── services/
│   │   │   └── personalizationService.ts       # API calls to backend
│   │   └── utils/
│   │       └── mdxValidator.ts                 # Validate MDX syntax
│   └── docusaurus.config.ts                    # Add MDXComponents wrapper
├── backend/
│   └── src/
│       └── api/
│           └── personalize.py                  # New FastAPI endpoint
└── specs/
    └── personalize-button/
        ├── spec.md                             # Already created
        ├── plan.md                             # This file
        └── tasks.md                            # To be created next
```

---

## 3. Detailed Component Design

### 3.1 ChapterWrapper Component

**Purpose:** Wraps every MDX chapter, manages personalization state and rendering

**File:** `frontend/src/components/Personalize/ChapterWrapper.tsx`

```typescript
import React, { useState, useEffect } from 'react';
import { useUserProfile } from '@/contexts/UserProfileContext';
import PersonalizeButton from './PersonalizeButton';
import PersonalizationBadge from './PersonalizationBadge';
import LoadingOverlay from './LoadingOverlay';
import { usePersonalization } from '@/hooks/usePersonalization';

interface ChapterWrapperProps {
  chapterId: string;        // e.g., "01-intro/ros2-basics"
  children: React.ReactNode; // Original MDX content
}

export default function ChapterWrapper({
  chapterId,
  children
}: ChapterWrapperProps): JSX.Element {
  const { isAuthenticated, profile } = useUserProfile();

  const {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  } = usePersonalization(chapterId);

  // If not authenticated, render original content only
  if (!isAuthenticated) {
    return <>{children}</>;
  }

  // If personalized, render personalized content
  if (personalizedContent) {
    return (
      <div className="chapter-wrapper">
        <PersonalizationBadge
          profile={profile}
          isFromCache={isFromCache}
          onReset={resetToDefault}
        />
        <div
          className="personalized-content"
          dangerouslySetInnerHTML={{ __html: personalizedContent }}
        />
      </div>
    );
  }

  // Default: render original content with personalize button
  return (
    <div className="chapter-wrapper">
      <PersonalizeButton
        chapterId={chapterId}
        onPersonalize={personalizeContent}
        isLoading={isPersonalizing}
        error={error}
      />
      {isPersonalizing && <LoadingOverlay />}
      {children}
    </div>
  );
}
```

**Key Features:**
- ✅ Checks authentication via `useUserProfile()` hook
- ✅ Manages personalization state via custom `usePersonalization` hook
- ✅ Conditionally renders button only for authenticated users
- ✅ Handles loading, success, and error states
- ✅ Supports reset to default functionality

---

### 3.2 PersonalizeButton Component

**Purpose:** Renders the "Personalize for Me" button with states

**File:** `frontend/src/components/Personalize/PersonalizeButton.tsx`

```typescript
import React from 'react';
import styles from './personalize.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize: () => Promise<void>;
  isLoading: boolean;
  error: Error | null;
}

export default function PersonalizeButton({
  chapterId,
  onPersonalize,
  isLoading,
  error
}: PersonalizeButtonProps): JSX.Element {
  return (
    <div className={styles.buttonContainer}>
      <button
        onClick={onPersonalize}
        disabled={isLoading}
        className={styles.personalizeButton}
        aria-label="Personalize this chapter to your experience and hardware"
        title="Adapt this chapter to your experience and hardware"
      >
        <span className={styles.icon}>✨</span>
        <span className={styles.text}>
          {isLoading ? 'Personalizing...' : 'Personalize for Me'}
        </span>
      </button>

      {error && (
        <div className={styles.errorMessage} role="alert">
          ❌ {error.message}
          <button
            onClick={onPersonalize}
            className={styles.retryButton}
          >
            Retry
          </button>
        </div>
      )}
    </div>
  );
}
```

**Styling (Tailwind + Custom CSS):**
```css
/* personalize.module.css */
.buttonContainer {
  @apply mb-6 flex flex-col items-start gap-2;
}

.personalizeButton {
  @apply inline-flex items-center gap-2 px-6 py-3 rounded-xl font-semibold text-white;
  @apply bg-gradient-to-r from-blue-500 via-purple-500 to-pink-500;
  @apply hover:shadow-lg hover:-translate-y-0.5 transition-all duration-300;
  @apply disabled:opacity-50 disabled:cursor-not-allowed;
  background-size: 200% 100%;
  animation: gradientShift 8s ease infinite;
}

.personalizeButton:disabled {
  animation: none;
}

.icon {
  @apply text-lg;
}

.text {
  @apply text-sm md:text-base;
}

.errorMessage {
  @apply p-3 bg-red-50 border border-red-200 rounded-lg text-red-800 text-sm;
  @apply flex items-center gap-2;
}

.retryButton {
  @apply ml-auto px-3 py-1 bg-red-600 text-white rounded hover:bg-red-700 text-xs;
}

@keyframes gradientShift {
  0%, 100% { background-position: 0% 50%; }
  50% { background-position: 100% 50%; }
}
```

---

### 3.3 PersonalizationBadge Component

**Purpose:** Shows user profile summary + reset option

**File:** `frontend/src/components/Personalize/PersonalizationBadge.tsx`

```typescript
import React from 'react';
import styles from './personalize.module.css';

interface PersonalizationBadgeProps {
  profile: {
    experience: 'beginner' | 'intermediate' | 'advanced';
    hasRTX: boolean;
    hasJetson: boolean;
    hasRobot: boolean;
  };
  isFromCache: boolean;
  onReset: () => void;
}

export default function PersonalizationBadge({
  profile,
  isFromCache,
  onReset
}: PersonalizationBadgeProps): JSX.Element {
  const hardwareList = [];
  if (profile.hasRTX) hardwareList.push('RTX GPU');
  if (profile.hasJetson) hardwareList.push('Jetson');
  if (profile.hasRobot) hardwareList.push('Real Robot');

  const hardwareText = hardwareList.length > 0
    ? ` with ${hardwareList.join(' & ')}`
    : ' (simulation-focused)';

  const experienceText = profile.experience.charAt(0).toUpperCase() +
                         profile.experience.slice(1);

  return (
    <div className={styles.badge}>
      <div className={styles.badgeContent}>
        <span className={styles.badgeIcon}>📌</span>
        <span className={styles.badgeText}>
          Personalized for: <strong>{experienceText}</strong> user{hardwareText}
        </span>
        {isFromCache && (
          <span className={styles.cacheIndicator}>(cached)</span>
        )}
      </div>
      <button
        onClick={onReset}
        className={styles.resetButton}
        aria-label="Reset to default content"
      >
        Reset to Default
      </button>
    </div>
  );
}
```

**Styling:**
```css
.badge {
  @apply mb-6 p-4 bg-gradient-to-r from-blue-50 to-purple-50 border-2 border-blue-200;
  @apply rounded-xl flex flex-col md:flex-row items-start md:items-center justify-between gap-3;
}

[data-theme='dark'] .badge {
  @apply from-blue-900/20 to-purple-900/20 border-blue-700/50;
}

.badgeContent {
  @apply flex items-center gap-2 flex-wrap;
}

.badgeIcon {
  @apply text-xl;
}

.badgeText {
  @apply text-sm text-gray-700;
}

[data-theme='dark'] .badgeText {
  @apply text-gray-300;
}

.cacheIndicator {
  @apply text-xs text-gray-500 italic;
}

.resetButton {
  @apply px-3 py-1.5 bg-white border border-gray-300 rounded-lg text-sm;
  @apply hover:bg-gray-50 transition-colors duration-200;
}

[data-theme='dark'] .resetButton {
  @apply bg-gray-800 border-gray-600 hover:bg-gray-700 text-gray-300;
}
```

---

### 3.4 LoadingOverlay Component

**Purpose:** Full-screen overlay with spinner during personalization

**File:** `frontend/src/components/Personalize/LoadingOverlay.tsx`

```typescript
import React from 'react';
import styles from './personalize.module.css';

export default function LoadingOverlay(): JSX.Element {
  return (
    <div className={styles.overlay} role="status" aria-live="polite">
      <div className={styles.overlayContent}>
        <div className={styles.spinner} />
        <p className={styles.loadingText}>Personalizing content for you...</p>
        <p className={styles.loadingSubtext}>This may take 5-10 seconds</p>
      </div>
    </div>
  );
}
```

**Styling:**
```css
.overlay {
  @apply fixed inset-0 bg-black/50 backdrop-blur-sm z-50;
  @apply flex items-center justify-center;
  animation: fadeIn 0.2s ease-in;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

.overlayContent {
  @apply bg-white rounded-2xl p-8 shadow-2xl max-w-md mx-4;
  @apply flex flex-col items-center gap-4;
}

[data-theme='dark'] .overlayContent {
  @apply bg-gray-800;
}

.spinner {
  @apply w-12 h-12 border-4 border-purple-200 border-t-purple-600 rounded-full;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.loadingText {
  @apply text-lg font-semibold text-gray-800;
}

[data-theme='dark'] .loadingText {
  @apply text-gray-200;
}

.loadingSubtext {
  @apply text-sm text-gray-600;
}

[data-theme='dark'] .loadingSubtext {
  @apply text-gray-400;
}
```

---

### 3.5 usePersonalization Custom Hook

**Purpose:** Encapsulates all personalization logic (caching, API calls, state)

**File:** `frontend/src/hooks/usePersonalization.ts`

```typescript
import { useState, useEffect } from 'react';
import { useUserProfile } from '@/contexts/UserProfileContext';
import { personalizeChapter } from '@/services/personalizationService';
import { generateProfileHash } from '@/utils/profileHash';

interface UsePersonalizationResult {
  isPersonalizing: boolean;
  personalizedContent: string | null;
  error: Error | null;
  isFromCache: boolean;
  personalizeContent: () => Promise<void>;
  resetToDefault: () => void;
}

export function usePersonalization(chapterId: string): UsePersonalizationResult {
  const { profile, isAuthenticated } = useUserProfile();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [error, setError] = useState<Error | null>(null);
  const [isFromCache, setIsFromCache] = useState(false);

  // Generate cache key
  const userId = profile?.userId || 'anonymous';
  const profileHash = profile ? generateProfileHash(profile) : '';
  const cacheKey = `personalized_${chapterId}_${userId}_${profileHash}`;

  // Check cache on mount
  useEffect(() => {
    if (!isAuthenticated || !profile) return;

    const cached = localStorage.getItem(cacheKey);
    if (cached) {
      try {
        const { content, timestamp } = JSON.parse(cached);
        const age = Date.now() - timestamp;
        const maxAge = 7 * 24 * 60 * 60 * 1000; // 7 days

        if (age < maxAge) {
          setPersonalizedContent(content);
          setIsFromCache(true);
        } else {
          localStorage.removeItem(cacheKey);
        }
      } catch (err) {
        console.error('Cache parse error:', err);
        localStorage.removeItem(cacheKey);
      }
    }
  }, [chapterId, cacheKey, isAuthenticated, profile]);

  const personalizeContent = async () => {
    if (!isAuthenticated || !profile) {
      setError(new Error('Please log in to personalize content'));
      return;
    }

    setIsPersonalizing(true);
    setError(null);

    try {
      // Get original chapter content
      const originalContent = document.querySelector('.markdown')?.textContent || '';

      // Call backend API
      const result = await personalizeChapter({
        chapterId,
        originalContent,
        userProfile: {
          experience: profile.experience,
          hasRTX: profile.hasRTX,
          hasJetson: profile.hasJetson,
          hasRobot: profile.hasRobot
        }
      });

      // Cache the result
      localStorage.setItem(cacheKey, JSON.stringify({
        content: result.personalizedContent,
        timestamp: Date.now()
      }));

      setPersonalizedContent(result.personalizedContent);
      setIsFromCache(false);
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Personalization failed'));
    } finally {
      setIsPersonalizing(false);
    }
  };

  const resetToDefault = () => {
    setPersonalizedContent(null);
    setIsFromCache(false);
    localStorage.removeItem(cacheKey);
  };

  return {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  };
}
```

---

### 3.6 Personalization Service (API Client)

**Purpose:** Makes HTTP requests to backend

**File:** `frontend/src/services/personalizationService.ts`

```typescript
interface PersonalizeRequest {
  chapterId: string;
  originalContent: string;
  userProfile: {
    experience: 'beginner' | 'intermediate' | 'advanced';
    hasRTX: boolean;
    hasJetson: boolean;
    hasRobot: boolean;
  };
}

interface PersonalizeResponse {
  success: boolean;
  personalizedContent: string;
  metadata: {
    generationTime: number;
    cached: boolean;
  };
}

export async function personalizeChapter(
  request: PersonalizeRequest
): Promise<PersonalizeResponse> {
  const token = localStorage.getItem('auth_token');

  if (!token) {
    throw new Error('Authentication required');
  }

  const response = await fetch('http://localhost:8000/api/personalize/chapter', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify(request)
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Personalization failed');
  }

  return response.json();
}
```

---

### 3.7 Backend Endpoint

**Purpose:** Receives requests, calls @personalizer subagent, returns personalized content

**File:** `backend/src/api/personalize.py`

```python
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Literal
from ..services.auth_service import get_current_user
from ..agents.personalizer_agent import personalize_chapter_content

router = APIRouter()

class UserProfile(BaseModel):
    experience: Literal['beginner', 'intermediate', 'advanced']
    hasRTX: bool
    hasJetson: bool
    hasRobot: bool

class PersonalizeChapterRequest(BaseModel):
    chapterId: str
    originalContent: str
    userProfile: UserProfile

class PersonalizeChapterResponse(BaseModel):
    success: bool
    personalizedContent: str
    metadata: dict

@router.post("/api/personalize/chapter", response_model=PersonalizeChapterResponse)
async def personalize_chapter(
    request: PersonalizeChapterRequest,
    current_user = Depends(get_current_user)
):
    """
    Personalize chapter content based on user profile.

    Uses the @personalizer subagent from the 8-agent system.
    """
    try:
        # Call @personalizer subagent
        personalized_content = await personalize_chapter_content(
            original_content=request.originalContent,
            user_profile=request.userProfile.dict(),
            chapter_id=request.chapterId
        )

        return PersonalizeChapterResponse(
            success=True,
            personalizedContent=personalized_content,
            metadata={
                "generationTime": 0,  # Track actual time in production
                "cached": False
            }
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**Integration with @personalizer subagent:**

**File:** `backend/src/agents/personalizer_agent.py`

```python
from claude_agent_sdk import Agent, PromptTemplate

# @personalizer subagent configuration (already exists from 8-agent system)
personalizer_agent = Agent(
    name="personalizer",
    description="Personalizes educational content based on user profile",
    model="claude-opus-4-5"
)

async def personalize_chapter_content(
    original_content: str,
    user_profile: dict,
    chapter_id: str
) -> str:
    """
    Call @personalizer subagent to rewrite chapter content.

    This function interfaces with the existing @personalizer subagent
    that was created as part of the 8-agent system.
    """

    # Build personalization prompt
    prompt = f"""
You are a technical content personalizer for a Physical AI & Robotics textbook.

User Profile:
- Experience: {user_profile['experience']}
- Hardware: RTX GPU: {user_profile['hasRTX']}, Jetson: {user_profile['hasJetson']}, Real Robot: {user_profile['hasRobot']}

Original Chapter Content:
{original_content}

Task: Rewrite this chapter to match the user's profile:

Experience Level Adjustments:
- Beginner: Add detailed explanations, analogies, heavily commented code, step-by-step guidance
- Intermediate: Balanced explanations, moderate comments, some advanced tips
- Advanced: Concise explanations, advanced optimizations, low-level details, performance tips

Hardware-Specific Adjustments:
- Has RTX GPU: Mention local Isaac Sim, GPU optimization, local training
- Has Jetson: Emphasize edge deployment, Jetson-specific examples, power efficiency
- No Jetson: Suggest cloud training (AWS/GCP/Azure), remote control, simulation-first
- Has Real Robot: Include real-world deployment considerations, safety protocols

Requirements:
1. Maintain 100% technical accuracy
2. Keep all code examples functional and correct
3. Preserve chapter structure and MDX formatting
4. Add personalized callouts/notes where relevant
5. Adjust depth of explanations to match experience level
6. Include hardware-specific examples where applicable

Return ONLY the personalized MDX content, no explanations.
"""

    # Call @personalizer subagent
    response = await personalizer_agent.run(prompt)

    return response.content
```

---

## 4. Implementation Sequence

### Phase 1: Frontend Components (Day 1)
**Time Estimate:** 6-8 hours

**Step 1.1:** Create component files
- Create `ChapterWrapper.tsx`
- Create `PersonalizeButton.tsx`
- Create `PersonalizationBadge.tsx`
- Create `LoadingOverlay.tsx`
- Create `personalize.module.css`

**Step 1.2:** Implement `usePersonalization` hook
- Cache check logic
- State management
- API call integration
- Reset functionality

**Step 1.3:** Implement `personalizationService.ts`
- API client with fetch
- Error handling
- TypeScript types

**Step 1.4:** Integrate with Docusaurus
- Modify `docusaurus.config.ts` to add MDXComponents wrapper
- Register ChapterWrapper as global component
- Test with sample chapter

**Acceptance Criteria:**
- ✅ Button renders only for authenticated users
- ✅ Loading overlay appears on click
- ✅ API call is made with correct payload
- ✅ Error messages display correctly

---

### Phase 2: Backend Endpoint (Day 2)
**Time Estimate:** 4-6 hours

**Step 2.1:** Create personalize.py endpoint
- FastAPI route definition
- Request/response models with Pydantic
- JWT authentication dependency
- Error handling

**Step 2.2:** Integrate with @personalizer subagent
- Import existing personalizer agent
- Build prompt template
- Call agent and await response
- Validate MDX output

**Step 2.3:** Add endpoint to main router
- Register in `backend/src/api/__init__.py`
- Test with Postman/curl

**Acceptance Criteria:**
- ✅ Endpoint accepts POST requests with JWT
- ✅ Rejects unauthenticated requests (401)
- ✅ Calls @personalizer subagent successfully
- ✅ Returns valid MDX content
- ✅ Response time < 10 seconds

---

### Phase 3: Caching & Performance (Day 3)
**Time Estimate:** 4-6 hours

**Step 3.1:** Implement localStorage caching
- Generate cache keys with profile hash
- Save personalized content with timestamp
- Check cache on component mount
- Handle cache expiry (7 days)

**Step 3.2:** Add cache indicators
- "Using cached personalization" badge
- "Regenerate" button to force fresh
- Clear cache on profile change

**Step 3.3:** Optimize performance
- Lazy load ChapterWrapper
- Debounce button clicks
- Minimize re-renders

**Acceptance Criteria:**
- ✅ Cached content loads instantly (< 100ms)
- ✅ Cache invalidates after 7 days
- ✅ Cache clears on profile change
- ✅ No memory leaks

---

### Phase 4: Testing & Polish (Day 4)
**Time Estimate:** 6-8 hours

**Step 4.1:** Unit tests
- Test usePersonalization hook
- Test PersonalizeButton states
- Test cache logic
- Test API service

**Step 4.2:** Integration tests
- Full personalization flow
- Cache hit/miss scenarios
- Error handling paths
- Reset functionality

**Step 4.3:** Manual testing
- Test all 3 experience levels
- Test all hardware combinations
- Test on mobile devices
- Test with long chapters

**Step 4.4:** Accessibility audit
- Keyboard navigation
- Screen reader support
- ARIA labels
- Color contrast

**Step 4.5:** UI polish
- Animation timing
- Responsive design
- Dark mode support
- Loading states

**Acceptance Criteria:**
- ✅ All unit tests pass
- ✅ All integration tests pass
- ✅ WCAG 2.1 Level AA compliance
- ✅ Works on mobile, tablet, desktop
- ✅ No console errors or warnings

---

## 5. Testing Strategy

### 5.1 Unit Tests

**Frontend Tests (Jest + React Testing Library):**

```typescript
// ChapterWrapper.test.tsx
describe('ChapterWrapper', () => {
  it('hides button when user is not authenticated', () => {
    const { queryByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Original content</div>
      </ChapterWrapper>
    );

    expect(queryByText('Personalize for Me')).not.toBeInTheDocument();
  });

  it('shows button when user is authenticated', () => {
    mockUseUserProfile({ isAuthenticated: true, profile: mockProfile });

    const { getByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Original content</div>
      </ChapterWrapper>
    );

    expect(getByText('Personalize for Me')).toBeInTheDocument();
  });

  it('loads cached content on mount if available', () => {
    localStorage.setItem('personalized_test_user1_hash123', JSON.stringify({
      content: '<div>Personalized</div>',
      timestamp: Date.now()
    }));

    const { getByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Original content</div>
      </ChapterWrapper>
    );

    expect(getByText(/cached/i)).toBeInTheDocument();
  });
});
```

**Backend Tests (pytest):**

```python
# test_personalize.py
def test_personalize_endpoint_requires_auth():
    response = client.post("/api/personalize/chapter", json={
        "chapterId": "test",
        "originalContent": "# Test",
        "userProfile": {"experience": "beginner", "hasRTX": True, "hasJetson": False, "hasRobot": False}
    })
    assert response.status_code == 401

def test_personalize_endpoint_returns_valid_mdx():
    response = client.post(
        "/api/personalize/chapter",
        json={
            "chapterId": "test",
            "originalContent": "# Test Chapter",
            "userProfile": {"experience": "beginner", "hasRTX": True, "hasJetson": False, "hasRobot": False}
        },
        headers={"Authorization": f"Bearer {valid_token}"}
    )
    assert response.status_code == 200
    assert "personalizedContent" in response.json()
```

### 5.2 Integration Tests

**End-to-End Flow (Playwright/Cypress):**

```typescript
describe('Personalization E2E', () => {
  beforeEach(() => {
    // Login as beginner with RTX GPU
    cy.login('beginner@test.com', 'password123');
    cy.visit('/docs/01-intro/ros2-basics');
  });

  it('personalizes chapter on button click', () => {
    // Click personalize button
    cy.contains('Personalize for Me').click();

    // Loading overlay appears
    cy.contains('Personalizing content for you...').should('be.visible');

    // Wait for personalization to complete
    cy.contains('Personalized for: Beginner with RTX GPU', { timeout: 15000 })
      .should('be.visible');

    // Verify content changed
    cy.contains('Since you have an RTX GPU').should('be.visible');
  });

  it('loads cached content on second visit', () => {
    // First personalization
    cy.contains('Personalize for Me').click();
    cy.contains('Personalized for', { timeout: 15000 }).should('be.visible');

    // Reload page
    cy.reload();

    // Should load from cache instantly
    cy.contains('(cached)').should('be.visible');
  });
});
```

### 5.3 AI Quality Tests

**Manual Verification Checklist:**

For each experience level (Beginner, Intermediate, Advanced):
- ✅ Beginner content has more explanations
- ✅ Advanced content is more concise
- ✅ Code examples remain functional
- ✅ Technical accuracy maintained
- ✅ MDX syntax is valid
- ✅ Links and images preserved

For each hardware combination:
- ✅ RTX GPU mentions appear for RTX users
- ✅ Jetson deployment tips for Jetson users
- ✅ Cloud training suggested for users without hardware

---

## 6. Deployment Plan

### 6.1 Development Environment

**Prerequisites:**
- ✅ Node.js 18+ and npm installed
- ✅ Python 3.11+ and pip installed
- ✅ PostgreSQL (Neon DB) accessible
- ✅ Better-Auth already configured
- ✅ Claude API key configured for @personalizer agent

**Setup Steps:**

1. **Install Frontend Dependencies:**
```bash
cd frontend
npm install
```

2. **Install Backend Dependencies:**
```bash
cd backend
pip install fastapi uvicorn pydantic
```

3. **Configure Environment Variables:**
```bash
# backend/.env
DATABASE_URL=postgresql://...  # Neon DB URL (already configured)
JWT_SECRET=...                 # Better-Auth secret (already configured)
CLAUDE_API_KEY=...             # For @personalizer agent
```

4. **Start Development Servers:**
```bash
# Terminal 1: Frontend
cd frontend
npm start

# Terminal 2: Backend
cd backend
uvicorn src.main:app --reload --port 8000
```

### 6.2 Production Deployment

**Frontend (Vercel/Netlify):**
- Build Docusaurus site: `npm run build`
- Deploy `build/` directory
- Configure environment variables for backend API URL

**Backend (Railway/Render/Fly.io):**
- Deploy FastAPI app
- Set environment variables
- Enable CORS for frontend domain
- Configure rate limiting (10 req/min per user)

**Database:**
- No schema changes needed (uses existing Better-Auth tables)

---

## 7. Monitoring & Analytics

### 7.1 Metrics to Track

**Usage Metrics:**
- Personalization button click rate (per chapter)
- Personalization success rate
- Cache hit rate
- Time-on-page (personalized vs. default)

**Performance Metrics:**
- API response time (p50, p95, p99)
- Frontend rendering time
- Cache load time

**Error Metrics:**
- Personalization failure rate
- Error types and frequencies
- Timeout occurrences

### 7.2 Logging

**Frontend Logging (Console + Analytics):**
```typescript
// Log personalization events
analytics.track('personalization_clicked', {
  chapterId,
  userProfile: { experience, hasRTX, hasJetson, hasRobot }
});

analytics.track('personalization_completed', {
  chapterId,
  fromCache: isFromCache,
  duration: Date.now() - startTime
});
```

**Backend Logging (Python logging):**
```python
import logging

logger = logging.getLogger(__name__)

@router.post("/api/personalize/chapter")
async def personalize_chapter(request, current_user):
    logger.info(f"Personalization request: user={current_user.id}, chapter={request.chapterId}")

    try:
        result = await personalize_chapter_content(...)
        logger.info(f"Personalization success: chapter={request.chapterId}, time={elapsed}s")
        return result
    except Exception as e:
        logger.error(f"Personalization failed: {e}", exc_info=True)
        raise
```

---

## 8. Risks & Mitigations

### 8.1 Technical Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| AI generates invalid MDX | High | Medium | Validate MDX syntax, retry once, fallback to original |
| Personalization too slow (>15s) | High | Medium | Optimize prompts, implement timeout, show progress |
| Cache becomes stale | Medium | Low | TTL of 7 days, version tracking for chapters |
| High API costs | High | Low | Aggressive caching, rate limiting, monitor usage |

### 8.2 Product Risks

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Users don't understand feature | Medium | Medium | Clear tooltip, onboarding tutorial |
| Personalization creates confusion | Medium | Low | Always show "Reset to Default" option |
| Feature ignored by users | Low | Medium | A/B test button placement, track adoption |

---

## 9. Success Criteria

### 9.1 Technical Success (Launch Readiness)

- ✅ All unit tests pass (>90% coverage)
- ✅ All integration tests pass
- ✅ P95 response time < 10 seconds
- ✅ Personalization success rate > 95% in staging
- ✅ No console errors or warnings
- ✅ WCAG 2.1 Level AA compliance
- ✅ Works on Chrome, Firefox, Safari, Edge
- ✅ Mobile responsive (iOS, Android)

### 9.2 Product Success (30 days post-launch)

- 🎯 70%+ of logged-in users try personalization
- 🎯 50%+ increase in time-on-page for personalized content
- 🎯 Cache hit rate > 60%
- 🎯 User satisfaction rating ≥ 4.2/5
- 🎯 < 5% personalization error rate

---

## 10. Next Steps

### 10.1 Immediate Actions

1. ✅ **Review this plan** with stakeholders
2. ⬜ **Create tasks.md** with detailed task breakdown
3. ⬜ **Set up development environment** (if not already done)
4. ⬜ **Begin Phase 1** (Frontend Components)

### 10.2 Follow-Up Plans

After MVP launch:
- Plan Phase 2 features (personalization slider, manual overrides)
- Gather user feedback via surveys
- Analyze usage data and optimize
- Iterate on AI prompts based on quality feedback

---

## 11. Appendix

### 11.1 Key Dependencies

**Already Implemented (From Previous Work):**
- ✅ Better-Auth integration
- ✅ User profile schema in Neon DB
- ✅ `useUserProfile()` hook in frontend
- ✅ JWT authentication in backend
- ✅ @personalizer subagent (from 8-agent system)

**Need to Implement (This Feature):**
- ⬜ ChapterWrapper component
- ⬜ PersonalizeButton component
- ⬜ PersonalizationBadge component
- ⬜ LoadingOverlay component
- ⬜ usePersonalization hook
- ⬜ personalizationService.ts
- ⬜ Backend /api/personalize/chapter endpoint
- ⬜ Integration with @personalizer subagent

### 11.2 Estimated Timeline

| Phase | Duration | Completion Target |
|-------|----------|-------------------|
| Phase 1: Frontend Components | 1 day | Day 1 |
| Phase 2: Backend Endpoint | 1 day | Day 2 |
| Phase 3: Caching & Performance | 1 day | Day 3 |
| Phase 4: Testing & Polish | 1 day | Day 4 |
| **Total** | **3-4 days** | **End of Week 1** |

### 11.3 Resource Requirements

**Development:**
- 1 Full-stack developer (React + Python)
- Access to staging environment
- Claude API key for @personalizer agent

**Testing:**
- QA tester for manual testing
- Accessibility tester
- Beta users (5-10 volunteers)

**Infrastructure:**
- Existing: Vercel/Netlify (frontend), Railway/Render (backend), Neon DB
- No additional infrastructure needed

---

**End of Implementation Plan**

**Ready to proceed to:** `/sp.tasks` for detailed task breakdown
