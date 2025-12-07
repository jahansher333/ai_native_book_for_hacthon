# Task Breakdown: Per-Chapter "Personalize for Me" Button

**Feature ID:** `personalize-button`
**Spec Version:** 1.0
**Plan Version:** 1.0
**Tasks Version:** 1.0
**Created:** 2025-12-06

---

## Task Overview

**Total Tasks:** 20
**Estimated Total Time:** 24-32 hours (3-4 days)
**Dependencies:** Better-Auth integration, @personalizer subagent from 8-agent system

---

## Phase 1: Frontend Components (Day 1)

### Task 1: Create PersonalizeButton Component
**File:** `frontend/src/components/Personalize/PersonalizeButton.tsx`
**Estimated Time:** 2 hours
**Priority:** High
**Dependencies:** None

**Description:**
Create the main button component that triggers personalization. Include loading states, error handling, and tooltip.

**Acceptance Criteria:**
- ✅ Button renders with gradient styling (blue → purple → pink)
- ✅ Shows "✨ Personalize for Me" text with sparkle icon
- ✅ Displays "Personalizing..." during loading state
- ✅ Button disabled during loading
- ✅ Tooltip on hover: "Adapt this chapter to your experience and hardware"
- ✅ Error message displays below button if personalization fails
- ✅ Retry button appears on error
- ✅ Accessible via keyboard (Tab + Enter)
- ✅ ARIA labels present for screen readers

**Implementation Steps:**
1. Create `frontend/src/components/Personalize/PersonalizeButton.tsx`
2. Define TypeScript interface for props:
   ```typescript
   interface PersonalizeButtonProps {
     chapterId: string;
     onPersonalize: () => Promise<void>;
     isLoading: boolean;
     error: Error | null;
   }
   ```
3. Implement button with conditional rendering based on `isLoading`
4. Add gradient animation with CSS keyframes
5. Add error display with retry button
6. Add ARIA attributes and keyboard handlers
7. Create `personalize.module.css` for styles (Tailwind + custom)

**Test Cases:**
- [ ] Button renders correctly
- [ ] Clicking button triggers `onPersonalize` callback
- [ ] Button disables during loading
- [ ] Error message displays when error prop is set
- [ ] Retry button calls `onPersonalize` again
- [ ] Keyboard navigation works (Tab to focus, Enter to click)

---

### Task 2: Create PersonalizationBadge Component
**File:** `frontend/src/components/Personalize/PersonalizationBadge.tsx`
**Estimated Time:** 1.5 hours
**Priority:** High
**Dependencies:** None

**Description:**
Create the badge that displays user profile summary after personalization completes. Shows experience level, hardware, and reset option.

**Acceptance Criteria:**
- ✅ Badge displays: "📌 Personalized for: {experience} user with {hardware}"
- ✅ Hardware list formatted correctly (e.g., "RTX GPU & Jetson")
- ✅ Shows "(cached)" indicator when loaded from cache
- ✅ "Reset to Default" button on right side
- ✅ Gradient border (blue to purple)
- ✅ Responsive layout (stacks on mobile)
- ✅ Dark mode support

**Implementation Steps:**
1. Create `frontend/src/components/Personalize/PersonalizationBadge.tsx`
2. Define TypeScript interface:
   ```typescript
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
   ```
3. Build hardware list string from profile
4. Format experience text (capitalize first letter)
5. Add conditional cache indicator
6. Add reset button with hover state
7. Style with Tailwind utilities + custom gradient border

**Test Cases:**
- [ ] Badge renders with correct profile text
- [ ] Hardware list formats correctly (comma-separated)
- [ ] "(cached)" appears when `isFromCache` is true
- [ ] Reset button triggers `onReset` callback
- [ ] Badge responsive on mobile (stacks vertically)
- [ ] Dark mode styles apply correctly

---

### Task 3: Create LoadingOverlay Component
**File:** `frontend/src/components/Personalize/LoadingOverlay.tsx`
**Estimated Time:** 1 hour
**Priority:** Medium
**Dependencies:** None

**Description:**
Create a full-screen overlay with spinner and message that appears during personalization.

**Acceptance Criteria:**
- ✅ Semi-transparent backdrop (50% black with blur)
- ✅ Centered spinner animation (rotating border)
- ✅ Text: "Personalizing content for you..."
- ✅ Subtext: "This may take 5-10 seconds"
- ✅ ARIA live region for screen readers
- ✅ Fade-in animation (0.2s)
- ✅ Dark mode support

**Implementation Steps:**
1. Create `frontend/src/components/Personalize/LoadingOverlay.tsx`
2. Add fixed positioning with z-index 50
3. Create spinner with rotating border animation
4. Add loading text and subtext
5. Add ARIA attributes: `role="status"` `aria-live="polite"`
6. Create CSS animations (fadeIn, spin)
7. Style backdrop with backdrop-blur

**Test Cases:**
- [ ] Overlay covers entire screen
- [ ] Spinner rotates continuously
- [ ] Text is centered and readable
- [ ] Fade-in animation plays on mount
- [ ] Screen readers announce loading state
- [ ] Dark mode applies darker background

---

### Task 4: Create ChapterWrapper Component
**File:** `frontend/src/components/Personalize/ChapterWrapper.tsx`
**Estimated Time:** 3 hours
**Priority:** Critical
**Dependencies:** Tasks 1, 2, 3

**Description:**
Main wrapper component that manages personalization state and conditionally renders button, badge, loading overlay, and chapter content.

**Acceptance Criteria:**
- ✅ Wraps MDX chapter content (`children` prop)
- ✅ Checks authentication via `useUserProfile()` hook
- ✅ Hides button when user not authenticated
- ✅ Shows PersonalizeButton when authenticated
- ✅ Manages personalization state (loading, content, error)
- ✅ Conditionally renders personalized content or original
- ✅ Integrates with usePersonalization hook (Task 5)
- ✅ Handles all state transitions smoothly

**Implementation Steps:**
1. Create `frontend/src/components/Personalize/ChapterWrapper.tsx`
2. Define TypeScript interface:
   ```typescript
   interface ChapterWrapperProps {
     chapterId: string;
     children: React.ReactNode;
   }
   ```
3. Import `useUserProfile()` to check authentication
4. Import `usePersonalization()` hook (from Task 5)
5. Implement conditional rendering logic:
   - Not authenticated → render children only
   - Authenticated + not personalized → render button + children
   - Authenticated + personalized → render badge + personalized content
   - Loading → render overlay + children
6. Handle personalized content with `dangerouslySetInnerHTML` (sanitized)
7. Add error boundary for graceful failure

**Test Cases:**
- [ ] Button hidden when not authenticated
- [ ] Button shown when authenticated
- [ ] Loading overlay appears during personalization
- [ ] Personalized content replaces original on success
- [ ] Badge displays after personalization
- [ ] Reset button reverts to original content
- [ ] Error message appears on failure
- [ ] Original content always remains accessible

---

### Task 5: Create usePersonalization Custom Hook
**File:** `frontend/src/hooks/usePersonalization.ts`
**Estimated Time:** 3 hours
**Priority:** Critical
**Dependencies:** Task 7 (personalizationService)

**Description:**
Custom React hook that encapsulates all personalization logic: caching, API calls, state management, and cache invalidation.

**Acceptance Criteria:**
- ✅ Returns personalization state and actions
- ✅ Checks localStorage cache on mount
- ✅ Generates cache key from chapter ID + user ID + profile hash
- ✅ Validates cache age (7-day TTL)
- ✅ Calls personalization API when cache miss
- ✅ Saves result to cache on success
- ✅ Provides `personalizeContent()` function
- ✅ Provides `resetToDefault()` function
- ✅ Handles errors gracefully

**Implementation Steps:**
1. Create `frontend/src/hooks/usePersonalization.ts`
2. Define return type:
   ```typescript
   interface UsePersonalizationResult {
     isPersonalizing: boolean;
     personalizedContent: string | null;
     error: Error | null;
     isFromCache: boolean;
     personalizeContent: () => Promise<void>;
     resetToDefault: () => void;
   }
   ```
3. Use `useUserProfile()` to get profile and auth status
4. Generate cache key with profile hash
5. Implement cache check in `useEffect` (runs on mount)
6. Implement `personalizeContent()`:
   - Get original chapter content from DOM
   - Call `personalizeChapter()` API service
   - Save to cache with timestamp
   - Update state with personalized content
7. Implement `resetToDefault()`:
   - Clear state
   - Remove from localStorage
8. Add error handling with try-catch

**Test Cases:**
- [ ] Hook returns correct initial state
- [ ] Cache check runs on mount
- [ ] Expired cache (>7 days) is ignored
- [ ] Valid cache loads instantly
- [ ] `personalizeContent()` calls API and updates state
- [ ] `resetToDefault()` clears state and cache
- [ ] Errors are caught and stored in state
- [ ] Profile change invalidates cache

---

### Task 6: Create personalizationService API Client
**File:** `frontend/src/services/personalizationService.ts`
**Estimated Time:** 1.5 hours
**Priority:** High
**Dependencies:** None

**Description:**
API client that makes HTTP requests to the backend personalization endpoint. Handles authentication headers and error responses.

**Acceptance Criteria:**
- ✅ Exports `personalizeChapter()` function
- ✅ Includes JWT token in Authorization header
- ✅ Makes POST request to `/api/personalize/chapter`
- ✅ Sends chapter ID, original content, and user profile
- ✅ Returns personalized content and metadata
- ✅ Throws errors for failed requests
- ✅ Handles 401 Unauthorized (redirect to login)
- ✅ Handles 500 Server Error (user-friendly message)

**Implementation Steps:**
1. Create `frontend/src/services/personalizationService.ts`
2. Define request and response TypeScript interfaces:
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
   ```
3. Implement `personalizeChapter()` function:
   - Get JWT token from localStorage
   - Build fetch request with headers
   - Parse response JSON
   - Handle errors with descriptive messages
4. Add timeout handling (15 seconds)

**Test Cases:**
- [ ] Function throws error if no auth token
- [ ] Request includes Authorization header
- [ ] Request body matches expected format
- [ ] Success response returns personalized content
- [ ] 401 error redirects to /signin
- [ ] 500 error shows user-friendly message
- [ ] Network timeout throws appropriate error

---

### Task 7: Create Profile Hash Utility
**File:** `frontend/src/utils/profileHash.ts`
**Estimated Time:** 0.5 hours
**Priority:** Medium
**Dependencies:** None

**Description:**
Utility function to generate a hash from user profile for cache key generation. Ensures cache invalidates when profile changes.

**Acceptance Criteria:**
- ✅ Generates consistent hash for same profile
- ✅ Generates different hash for different profiles
- ✅ Uses MD5 or similar fast hash algorithm
- ✅ Returns 8-character hex string

**Implementation Steps:**
1. Create `frontend/src/utils/profileHash.ts`
2. Install `crypto-js` if not available (or use Web Crypto API)
3. Implement `generateProfileHash()` function:
   ```typescript
   export function generateProfileHash(profile: UserProfile): string {
     const profileString = JSON.stringify({
       experience: profile.experience,
       hasRTX: profile.hasRTX,
       hasJetson: profile.hasJetson,
       hasRobot: profile.hasRobot
     });
     // Generate MD5 hash and return first 8 characters
     return md5(profileString).substring(0, 8);
   }
   ```

**Test Cases:**
- [ ] Same profile generates same hash
- [ ] Different profiles generate different hashes
- [ ] Hash is 8 characters long
- [ ] Hash is alphanumeric

---

### Task 8: Integrate ChapterWrapper with Docusaurus
**File:** `frontend/docusaurus.config.ts` and `frontend/src/theme/MDXComponents/index.tsx`
**Estimated Time:** 1.5 hours
**Priority:** Critical
**Dependencies:** Task 4 (ChapterWrapper)

**Description:**
Configure Docusaurus to wrap all MDX chapters with ChapterWrapper component automatically. Use MDXComponents provider.

**Acceptance Criteria:**
- ✅ All chapter pages automatically wrapped with ChapterWrapper
- ✅ Chapter ID derived from file path
- ✅ No manual wrapping needed in individual chapters
- ✅ Original MDX content preserved
- ✅ All existing MDX components still work

**Implementation Steps:**
1. Create or modify `frontend/src/theme/MDXComponents/index.tsx`
2. Import ChapterWrapper component
3. Create wrapper function that extracts chapter ID:
   ```typescript
   import ChapterWrapper from '@/components/Personalize/ChapterWrapper';
   import { useLocation } from '@docusaurus/router';

   export default function MDXComponents(components) {
     return {
       ...components,
       wrapper: ({ children }) => {
         const location = useLocation();
         // Extract chapter ID from pathname (e.g., /docs/01-intro/ros2 → 01-intro/ros2)
         const chapterId = location.pathname.replace('/docs/', '').replace(/\/$/, '');

         return (
           <ChapterWrapper chapterId={chapterId}>
             {children}
           </ChapterWrapper>
         );
       }
     };
   }
   ```
4. Test with multiple chapter pages
5. Verify chapter ID extraction works correctly

**Test Cases:**
- [ ] ChapterWrapper renders on all /docs/* pages
- [ ] Chapter ID correctly extracted from URL
- [ ] Original MDX content renders inside wrapper
- [ ] All MDX features still work (code blocks, images, etc.)
- [ ] No infinite render loops

---

## Phase 2: Backend Endpoint (Day 2)

### Task 9: Create Backend Personalize Endpoint
**File:** `backend/src/api/personalize.py`
**Estimated Time:** 3 hours
**Priority:** Critical
**Dependencies:** Task 10 (personalizer integration)

**Description:**
Create FastAPI endpoint that receives personalization requests, validates JWT, and calls @personalizer subagent.

**Acceptance Criteria:**
- ✅ Endpoint: `POST /api/personalize/chapter`
- ✅ Requires JWT authentication
- ✅ Validates request payload with Pydantic models
- ✅ Returns 401 for unauthenticated requests
- ✅ Returns 400 for invalid payloads
- ✅ Returns 500 for server errors
- ✅ Calls @personalizer subagent successfully
- ✅ Returns personalized MDX content
- ✅ Response time < 10 seconds (p95)

**Implementation Steps:**
1. Create `backend/src/api/personalize.py`
2. Define Pydantic models:
   ```python
   from pydantic import BaseModel
   from typing import Literal

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
   ```
3. Create route handler:
   ```python
   @router.post("/api/personalize/chapter", response_model=PersonalizeChapterResponse)
   async def personalize_chapter(
       request: PersonalizeChapterRequest,
       current_user = Depends(get_current_user)
   ):
       # Call @personalizer subagent
       personalized_content = await personalize_chapter_content(
           original_content=request.originalContent,
           user_profile=request.userProfile.dict(),
           chapter_id=request.chapterId
       )

       return PersonalizeChapterResponse(
           success=True,
           personalizedContent=personalized_content,
           metadata={"generationTime": 0, "cached": False}
       )
   ```
4. Add error handling with try-except
5. Register router in `backend/src/api/__init__.py`
6. Test with Postman/curl

**Test Cases:**
- [ ] Endpoint rejects requests without JWT (401)
- [ ] Endpoint validates payload schema (400 if invalid)
- [ ] Endpoint calls @personalizer and returns content
- [ ] Response format matches PersonalizeChapterResponse
- [ ] Endpoint handles errors gracefully (500 with message)
- [ ] Response time < 10 seconds

---

### Task 10: Integrate with @personalizer Subagent
**File:** `backend/src/agents/personalizer_agent.py`
**Estimated Time:** 2 hours
**Priority:** Critical
**Dependencies:** Existing @personalizer subagent from 8-agent system

**Description:**
Create integration layer between FastAPI endpoint and existing @personalizer subagent. Build prompt template and call agent.

**Acceptance Criteria:**
- ✅ Function `personalize_chapter_content()` calls @personalizer
- ✅ Builds personalization prompt with user profile
- ✅ Includes all experience level instructions
- ✅ Includes all hardware-specific instructions
- ✅ Maintains MDX structure and accuracy
- ✅ Returns personalized MDX string
- ✅ Handles agent errors gracefully

**Implementation Steps:**
1. Create `backend/src/agents/personalizer_agent.py`
2. Import existing @personalizer agent from 8-agent system
3. Create `personalize_chapter_content()` function:
   ```python
   from claude_agent_sdk import Agent

   personalizer_agent = Agent(
       name="personalizer",
       description="Personalizes educational content",
       model="claude-opus-4-5"
   )

   async def personalize_chapter_content(
       original_content: str,
       user_profile: dict,
       chapter_id: str
   ) -> str:
       # Build prompt
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

       # Call agent
       response = await personalizer_agent.run(prompt)
       return response.content
   ```
4. Add error handling and logging
5. Add timeout (15 seconds)

**Test Cases:**
- [ ] Function accepts correct parameters
- [ ] Prompt includes all profile fields
- [ ] Agent returns valid MDX content
- [ ] Technical accuracy maintained
- [ ] MDX structure preserved
- [ ] Errors are caught and logged
- [ ] Timeout enforced

---

### Task 11: Add Rate Limiting to Backend
**File:** `backend/src/middleware/rate_limit.py`
**Estimated Time:** 1.5 hours
**Priority:** Medium
**Dependencies:** Task 9

**Description:**
Add rate limiting middleware to prevent abuse of personalization endpoint. Limit to 10 requests per minute per user.

**Acceptance Criteria:**
- ✅ Limit: 10 personalization requests per minute per user
- ✅ Returns 429 Too Many Requests when limit exceeded
- ✅ Includes Retry-After header in response
- ✅ Uses Redis or in-memory store for tracking
- ✅ Resets counter after 1 minute

**Implementation Steps:**
1. Create `backend/src/middleware/rate_limit.py`
2. Install `slowapi` or implement custom rate limiter
3. Create rate limit dependency:
   ```python
   from slowapi import Limiter
   from slowapi.util import get_remote_address

   limiter = Limiter(key_func=get_remote_address)

   @router.post("/api/personalize/chapter")
   @limiter.limit("10/minute")
   async def personalize_chapter(...):
       ...
   ```
4. Configure rate limit error handler
5. Add Retry-After header to 429 responses
6. Test with rapid requests

**Test Cases:**
- [ ] First 10 requests succeed
- [ ] 11th request within minute returns 429
- [ ] Counter resets after 1 minute
- [ ] Retry-After header present in 429 response
- [ ] Different users have independent counters

---

## Phase 3: Caching & Performance (Day 3)

### Task 12: Implement localStorage Caching
**File:** Already in `usePersonalization` hook (Task 5)
**Estimated Time:** 2 hours (included in Task 5)
**Priority:** High
**Dependencies:** Task 5

**Description:**
Implement caching logic in `usePersonalization` hook using localStorage. Cache key includes chapter ID, user ID, and profile hash.

**Acceptance Criteria:**
- ✅ Cache key format: `personalized_{chapterId}_{userId}_{profileHash}`
- ✅ Cached content includes: `{ content: string, timestamp: number }`
- ✅ Cache checked on component mount
- ✅ Cache TTL: 7 days (604,800,000 milliseconds)
- ✅ Expired cache automatically deleted
- ✅ Cache cleared on profile change
- ✅ Cache cleared on reset

**Implementation:** (Included in Task 5)

**Test Cases:**
- [ ] Fresh personalization saves to cache
- [ ] Cache loads instantly on return visit (< 100ms)
- [ ] Cache expires after 7 days
- [ ] Expired cache is deleted from localStorage
- [ ] Profile change invalidates cache
- [ ] Reset clears cache for chapter

---

### Task 13: Add Cache Indicators & Regenerate Option
**File:** `frontend/src/components/Personalize/PersonalizationBadge.tsx` (Task 2)
**Estimated Time:** 1 hour
**Priority:** Medium
**Dependencies:** Task 2, Task 5

**Description:**
Add visual indicators when content is loaded from cache and provide option to regenerate fresh personalization.

**Acceptance Criteria:**
- ✅ "(cached)" indicator in badge when `isFromCache` is true
- ✅ "Regenerate" button appears next to "(cached)"
- ✅ Regenerate button clears cache and fetches fresh content
- ✅ Regenerate button shows loading state
- ✅ Tooltip explains why regeneration might be useful

**Implementation Steps:**
1. Modify PersonalizationBadge component from Task 2
2. Add conditional "(cached)" text
3. Add "Regenerate" button next to cache indicator
4. Pass `onRegenerate` prop to badge component
5. Implement regenerate logic in usePersonalization hook:
   ```typescript
   const regenerate = async () => {
     localStorage.removeItem(cacheKey);
     await personalizeContent();
   };
   ```
6. Add tooltip: "Get fresh personalization with latest improvements"

**Test Cases:**
- [ ] "(cached)" appears when loading from cache
- [ ] "Regenerate" button only appears with "(cached)"
- [ ] Clicking Regenerate clears cache
- [ ] Fresh content fetched after Regenerate
- [ ] Loading state shown during regeneration

---

### Task 14: Optimize Performance & Reduce Re-renders
**File:** Various component files
**Estimated Time:** 2 hours
**Priority:** Medium
**Dependencies:** All frontend tasks

**Description:**
Optimize React components to minimize unnecessary re-renders. Use React.memo, useMemo, and useCallback where appropriate.

**Acceptance Criteria:**
- ✅ ChapterWrapper memoized with React.memo
- ✅ PersonalizeButton callbacks memoized with useCallback
- ✅ Badge props memoized with useMemo
- ✅ No re-renders when unrelated state changes
- ✅ Personalized content renders in < 100ms

**Implementation Steps:**
1. Wrap ChapterWrapper with React.memo
2. Memoize callbacks in usePersonalization:
   ```typescript
   const personalizeContent = useCallback(async () => {
     // ... implementation
   }, [chapterId, profile]);

   const resetToDefault = useCallback(() => {
     // ... implementation
   }, [cacheKey]);
   ```
3. Memoize derived values in PersonalizationBadge:
   ```typescript
   const hardwareText = useMemo(() => {
     const list = [];
     if (profile.hasRTX) list.push('RTX GPU');
     if (profile.hasJetson) list.push('Jetson');
     return list.join(' & ');
   }, [profile]);
   ```
4. Profile React DevTools to identify unnecessary re-renders
5. Add key props to lists

**Test Cases:**
- [ ] Component doesn't re-render when parent updates
- [ ] Callbacks maintain referential equality
- [ ] Derived values only recalculate when dependencies change
- [ ] React DevTools shows minimal re-renders

---

### Task 15: Add Loading Progress Indicator
**File:** `frontend/src/components/Personalize/LoadingOverlay.tsx` (Task 3)
**Estimated Time:** 1 hour
**Priority:** Low
**Dependencies:** Task 3

**Description:**
Enhance loading overlay with progress indicator or estimated time remaining to improve perceived performance.

**Acceptance Criteria:**
- ✅ Shows elapsed time (e.g., "3 seconds...")
- ✅ Updates every second during personalization
- ✅ Shows "Almost done..." after 8 seconds
- ✅ Smooth transitions between states

**Implementation Steps:**
1. Modify LoadingOverlay component from Task 3
2. Add `useEffect` with timer:
   ```typescript
   const [elapsed, setElapsed] = useState(0);

   useEffect(() => {
     const timer = setInterval(() => {
       setElapsed(prev => prev + 1);
     }, 1000);

     return () => clearInterval(timer);
   }, []);
   ```
3. Display elapsed time in overlay
4. Show different messages based on elapsed time:
   - 0-5s: "Personalizing content for you..."
   - 5-8s: "Still working on it..."
   - 8+s: "Almost done..."
5. Add subtle animation to keep user engaged

**Test Cases:**
- [ ] Timer starts at 0 seconds
- [ ] Timer increments every second
- [ ] Message changes at 5 and 8 seconds
- [ ] Timer stops when personalization completes
- [ ] Timer resets on new personalization

---

## Phase 4: Testing & Polish (Day 4)

### Task 16: Write Unit Tests for Frontend Components
**File:** `frontend/src/components/Personalize/__tests__/`
**Estimated Time:** 3 hours
**Priority:** High
**Dependencies:** All frontend tasks

**Description:**
Write comprehensive unit tests for all React components using Jest and React Testing Library.

**Acceptance Criteria:**
- ✅ Test coverage > 80% for all components
- ✅ All components have test files
- ✅ Tests cover happy path and edge cases
- ✅ Tests verify accessibility features
- ✅ All tests pass

**Test Files to Create:**
1. `ChapterWrapper.test.tsx`
2. `PersonalizeButton.test.tsx`
3. `PersonalizationBadge.test.tsx`
4. `LoadingOverlay.test.tsx`
5. `usePersonalization.test.ts`

**Sample Test (ChapterWrapper):**
```typescript
describe('ChapterWrapper', () => {
  it('hides button when user not authenticated', () => {
    mockUseUserProfile({ isAuthenticated: false });
    const { queryByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Content</div>
      </ChapterWrapper>
    );
    expect(queryByText('Personalize for Me')).not.toBeInTheDocument();
  });

  it('shows button when user authenticated', () => {
    mockUseUserProfile({ isAuthenticated: true, profile: mockProfile });
    const { getByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Content</div>
      </ChapterWrapper>
    );
    expect(getByText('Personalize for Me')).toBeInTheDocument();
  });

  it('loads cached content on mount', async () => {
    localStorage.setItem('personalized_test_user1_hash123', JSON.stringify({
      content: '<div>Cached</div>',
      timestamp: Date.now()
    }));

    const { getByText } = render(
      <ChapterWrapper chapterId="test">
        <div>Original</div>
      </ChapterWrapper>
    );

    await waitFor(() => {
      expect(getByText(/cached/i)).toBeInTheDocument();
    });
  });
});
```

**Test Cases:**
- [ ] All components render without crashing
- [ ] Button visibility based on auth state
- [ ] Loading states work correctly
- [ ] Error states display properly
- [ ] Cache logic works as expected
- [ ] Reset functionality works
- [ ] Accessibility attributes present

---

### Task 17: Write Integration Tests
**File:** `frontend/cypress/e2e/personalization.cy.ts` or `frontend/__tests__/integration/`
**Estimated Time:** 3 hours
**Priority:** High
**Dependencies:** All tasks

**Description:**
Write end-to-end integration tests using Cypress or Playwright to test full personalization flow.

**Acceptance Criteria:**
- ✅ Test full personalization flow (login → click → personalize → display)
- ✅ Test cache hit scenario
- ✅ Test error handling
- ✅ Test reset functionality
- ✅ Test across different user profiles
- ✅ All integration tests pass

**Test Scenarios:**

**Scenario 1: First-Time Personalization**
```typescript
describe('Personalization E2E', () => {
  it('personalizes chapter for beginner with RTX', () => {
    cy.login('beginner@test.com', 'password');
    cy.visit('/docs/01-intro/ros2-basics');

    cy.contains('Personalize for Me').click();
    cy.contains('Personalizing content').should('be.visible');

    cy.contains('Personalized for: Beginner with RTX GPU', { timeout: 15000 })
      .should('be.visible');

    cy.contains('Since you have an RTX GPU').should('be.visible');
  });
});
```

**Scenario 2: Cached Content**
```typescript
it('loads cached content on return visit', () => {
  // First visit - personalize
  cy.login('beginner@test.com', 'password');
  cy.visit('/docs/01-intro/ros2-basics');
  cy.contains('Personalize for Me').click();
  cy.contains('Personalized for', { timeout: 15000 });

  // Reload page
  cy.reload();

  // Should load from cache
  cy.contains('(cached)').should('be.visible');
  cy.contains('Personalized for').should('be.visible');
});
```

**Test Cases:**
- [ ] Full personalization flow works end-to-end
- [ ] Cache loads on second visit
- [ ] Reset button reverts to original
- [ ] Regenerate button fetches fresh content
- [ ] Error handling works for network failures
- [ ] Multiple chapters can be personalized independently

---

### Task 18: Manual Testing & QA
**Estimated Time:** 3 hours
**Priority:** High
**Dependencies:** All tasks

**Description:**
Perform comprehensive manual testing across browsers, devices, and user profiles.

**Test Matrix:**

| Profile | Hardware | Expected Mentions |
|---------|----------|-------------------|
| Beginner | RTX GPU | More explanations, local Isaac Sim, GPU setup |
| Beginner | No hardware | More explanations, cloud training suggestions |
| Intermediate | Jetson | Balanced content, Jetson deployment examples |
| Intermediate | RTX + Jetson | Balanced content, local training, edge deployment |
| Advanced | Real Robot | Concise, advanced optimization, safety protocols |
| Advanced | No hardware | Concise, cloud training, advanced optimization |

**Browsers to Test:**
- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)

**Devices to Test:**
- [ ] Desktop (1920x1080, 2560x1440)
- [ ] Laptop (1366x768, 1920x1080)
- [ ] Tablet (iPad, 768x1024)
- [ ] Mobile (iPhone 14, 390x844)
- [ ] Mobile (Android, various sizes)

**Checklist:**
- [ ] Personalization works on all browsers
- [ ] Responsive design works on all screen sizes
- [ ] Loading states appear correctly
- [ ] Error messages are clear and actionable
- [ ] Cache works across browser sessions
- [ ] Reset functionality works
- [ ] No console errors or warnings
- [ ] Performance is acceptable (< 10s personalization time)

---

### Task 19: Accessibility Audit
**Estimated Time:** 2 hours
**Priority:** High
**Dependencies:** All frontend tasks

**Description:**
Perform comprehensive accessibility audit to ensure WCAG 2.1 Level AA compliance.

**Acceptance Criteria:**
- ✅ WCAG 2.1 Level AA compliance
- ✅ Keyboard navigation works throughout
- ✅ Screen reader announces all states
- ✅ Color contrast ratios meet standards (4.5:1)
- ✅ Focus indicators visible
- ✅ ARIA labels present and correct

**Testing Tools:**
- [ ] axe DevTools Chrome extension
- [ ] WAVE browser extension
- [ ] Lighthouse accessibility audit
- [ ] Manual keyboard navigation test
- [ ] Screen reader test (NVDA, JAWS, VoiceOver)

**Checklist:**

**Keyboard Navigation:**
- [ ] Tab key moves to Personalize button
- [ ] Enter key activates Personalize button
- [ ] Tab key moves to Reset button
- [ ] Escape key closes loading overlay (if applicable)
- [ ] All interactive elements are keyboard accessible

**Screen Reader:**
- [ ] Button announced as "Personalize for Me, button"
- [ ] Loading state announced: "Personalizing content for you"
- [ ] Badge text read correctly
- [ ] Error messages announced immediately (ARIA live region)
- [ ] Focus management after state changes

**Color Contrast:**
- [ ] Button text on gradient background: 4.5:1 minimum
- [ ] Badge text on light background: 4.5:1 minimum
- [ ] Error message text on red background: 4.5:1 minimum
- [ ] Dark mode contrasts meet standards

**ARIA Attributes:**
- [ ] `role="button"` on interactive elements
- [ ] `aria-label` on PersonalizeButton
- [ ] `aria-live="polite"` on LoadingOverlay
- [ ] `role="alert"` on error messages
- [ ] `aria-busy="true"` during loading

**Test Cases:**
- [ ] All axe DevTools issues resolved
- [ ] Lighthouse accessibility score > 95
- [ ] Can complete full personalization flow with keyboard only
- [ ] Screen reader user can understand all states
- [ ] Color contrast ratios verified

---

### Task 20: UI Polish & Final Touches
**Estimated Time:** 2 hours
**Priority:** Medium
**Dependencies:** All frontend tasks

**Description:**
Polish UI animations, transitions, and visual feedback for premium feel.

**Acceptance Criteria:**
- ✅ Smooth transitions between states
- ✅ Button hover states feel responsive
- ✅ Loading animations are smooth (60fps)
- ✅ Badge appears with fade-in animation
- ✅ Personalized content loads with smooth transition
- ✅ Dark mode looks polished
- ✅ Mobile touch targets are adequate (44x44px minimum)

**Polish Items:**

**1. Button Animations:**
- [ ] Gradient flows smoothly (8s infinite loop)
- [ ] Hover state lifts button (-2px translateY)
- [ ] Active state scales down slightly (0.98)
- [ ] Disabled state has reduced opacity (50%)
- [ ] Transition timing: 0.3s cubic-bezier(0.4, 0, 0.2, 1)

**2. Loading Overlay:**
- [ ] Fade-in animation (0.2s ease-in)
- [ ] Spinner rotates at consistent speed (1s per rotation)
- [ ] Backdrop blur is smooth (not janky)
- [ ] Message text fades in after overlay

**3. Badge Appearance:**
- [ ] Badge slides in from top (0.4s ease-out)
- [ ] Text fades in after slide completes
- [ ] Reset button appears with fade-in

**4. Content Transition:**
- [ ] Old content fades out (0.2s)
- [ ] New content fades in (0.4s)
- [ ] Smooth scroll to top of chapter after personalization
- [ ] No content flash or jank

**5. Dark Mode:**
- [ ] Smooth transition between light/dark (0.3s)
- [ ] All colors have dark mode equivalents
- [ ] Contrast maintained in dark mode
- [ ] Shadows adapted for dark backgrounds

**6. Touch Targets (Mobile):**
- [ ] PersonalizeButton: minimum 44x44px
- [ ] Reset button: minimum 44x44px
- [ ] Retry button: minimum 44x44px
- [ ] All buttons have adequate spacing (8px minimum)

**Test Cases:**
- [ ] All animations run at 60fps (check with DevTools)
- [ ] No layout shifts during state transitions
- [ ] Transitions feel smooth and natural
- [ ] Dark mode toggle works without flash
- [ ] Touch targets are easy to tap on mobile

---

## Summary

### Task Dependencies Graph

```
Phase 1 (Day 1):
├── Task 1: PersonalizeButton ──────┐
├── Task 2: PersonalizationBadge ───┤
├── Task 3: LoadingOverlay ─────────┼──> Task 4: ChapterWrapper ──┐
├── Task 7: Profile Hash Utility ───┤                              │
└── Task 6: personalizationService ─┴──> Task 5: usePersonalization ┤
                                                                     │
                                    Task 8: Integrate with Docusaurus┘

Phase 2 (Day 2):
└── Task 10: @personalizer integration ──> Task 9: Backend Endpoint ──> Task 11: Rate Limiting

Phase 3 (Day 3):
├── Task 12: localStorage Caching (in Task 5)
├── Task 13: Cache Indicators (in Task 2)
├── Task 14: Performance Optimization
└── Task 15: Progress Indicator (in Task 3)

Phase 4 (Day 4):
├── Task 16: Unit Tests
├── Task 17: Integration Tests
├── Task 18: Manual Testing & QA
├── Task 19: Accessibility Audit
└── Task 20: UI Polish
```

### Effort Summary

| Phase | Tasks | Estimated Time |
|-------|-------|----------------|
| Phase 1: Frontend Components | 8 tasks | 14-16 hours |
| Phase 2: Backend Endpoint | 3 tasks | 6-8 hours |
| Phase 3: Caching & Performance | 4 tasks | 4-6 hours |
| Phase 4: Testing & Polish | 5 tasks | 13-15 hours |
| **Total** | **20 tasks** | **37-45 hours (5-6 days)** |

### Critical Path

The critical path for completing this feature:

1. Task 7 (Profile Hash) → Task 6 (API Service) → Task 5 (usePersonalization)
2. Tasks 1, 2, 3 (UI Components) → Task 4 (ChapterWrapper)
3. Task 4 → Task 8 (Docusaurus Integration)
4. Task 10 (@personalizer) → Task 9 (Backend Endpoint)
5. Task 16, 17 (Tests) → Task 18, 19 (QA)

**Estimated delivery with 1 developer:** 5-6 days
**Estimated delivery with 2 developers:** 3-4 days

---

## Acceptance Checklist

Before marking this feature as **DONE**, verify:

### Functional Requirements:
- [ ] Button visible only for authenticated users
- [ ] Personalization works for all 3 experience levels
- [ ] Hardware-specific content appears correctly
- [ ] Caching works (instant load on return)
- [ ] Cache expires after 7 days
- [ ] Reset to default works
- [ ] Error handling works (network, timeout, AI failure)
- [ ] Rate limiting enforced (10 req/min/user)

### Non-Functional Requirements:
- [ ] P95 response time < 10 seconds
- [ ] Cached content loads in < 100ms
- [ ] WCAG 2.1 Level AA compliance
- [ ] Works on Chrome, Firefox, Safari, Edge
- [ ] Responsive on mobile, tablet, desktop
- [ ] No console errors or warnings
- [ ] Unit test coverage > 80%
- [ ] All integration tests pass

### Documentation:
- [ ] Component API documentation added
- [ ] Backend API documented in spec
- [ ] User guide for personalization feature
- [ ] Developer setup instructions

### Deployment:
- [ ] Frontend deployed to production
- [ ] Backend deployed to production
- [ ] Environment variables configured
- [ ] Rate limiting configured
- [ ] Monitoring/analytics enabled

---

**End of Task Breakdown**

**Status:** Ready for implementation
**Next Action:** Begin Task 1 (Create PersonalizeButton Component)
