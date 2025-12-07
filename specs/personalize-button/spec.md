# Feature Specification: Per-Chapter "Personalize for Me" Button

**Feature ID:** `personalize-button`
**Priority:** High (50 Bonus Points)
**Status:** Draft
**Created:** 2025-12-06
**Last Updated:** 2025-12-06

---

## 1. Overview

### 1.1 Feature Summary
Add a "Personalize for Me" button at the top of every chapter that dynamically rewrites the chapter content based on the logged-in user's profile (experience level and hardware). The personalized content adapts explanations, code examples, and recommendations to match the user's specific learning needs and available hardware.

### 1.2 Business Value
- **Enhanced Learning Experience:** Content adapts to each user's level and hardware setup
- **Increased Engagement:** Users stay engaged with content tailored to their needs
- **Better Learning Outcomes:** Appropriate difficulty level improves comprehension
- **Hardware-Specific Guidance:** Users get relevant examples for their actual hardware
- **Competitive Differentiation:** Unique AI-powered personalization feature

### 1.3 Success Metrics
- 70%+ of logged-in users click "Personalize for Me" within first 3 chapters
- Average time-on-page increases by 40% for personalized content
- User satisfaction score increases from personalized content feedback
- 80%+ of personalized content requests complete successfully within 10 seconds

---

## 2. User Stories

### 2.1 Primary User Stories

**US-1: View Personalize Button (Authenticated Users)**
```
As a logged-in user
When I view any chapter page
Then I see a "Personalize for Me" button at the top of the chapter
And the button is visually prominent and clearly labeled
```

**US-2: Personalize Chapter Content**
```
As a logged-in user with profile data
When I click the "Personalize for Me" button
Then the system fetches my profile (experience level, hardware)
And generates personalized chapter content via AI
And replaces the chapter content with the personalized version
And displays a badge showing my personalization settings
```

**US-3: Beginner-Level Personalization**
```
As a beginner user
When I personalize a chapter
Then I receive:
  - More detailed explanations of concepts
  - Analogies and real-world examples
  - Simpler code with extensive comments
  - Step-by-step breakdowns of complex topics
  - Links to prerequisite materials
```

**US-4: Intermediate-Level Personalization**
```
As an intermediate user
When I personalize a chapter
Then I receive:
  - Balanced explanations (not too basic, not too advanced)
  - Standard code examples with moderate comments
  - Some advanced tips and best practices
  - References to related advanced topics
```

**US-5: Advanced-Level Personalization**
```
As an advanced user
When I personalize a chapter
Then I receive:
  - Concise explanations assuming prior knowledge
  - Advanced optimization techniques
  - Low-level implementation details
  - Performance considerations
  - Edge cases and advanced patterns
```

**US-6: RTX GPU Hardware Personalization**
```
As a user with an RTX GPU
When I personalize a chapter
Then I receive:
  - References to running Isaac Sim locally
  - GPU-specific optimization tips
  - Local training examples
  - Ray tracing and tensor core utilization guidance
```

**US-7: Jetson Hardware Personalization**
```
As a user with a Jetson device
When I personalize a chapter
Then I receive:
  - Deployment examples targeting Jetson
  - Edge inference optimization tips
  - Jetson-specific performance tuning
  - Power efficiency considerations
```

**US-8: No Jetson Hardware Personalization**
```
As a user without a Jetson device
When I personalize a chapter
Then I receive:
  - Cloud training recommendations (AWS, GCP, Azure)
  - Cloud deployment strategies
  - Remote robot control examples
  - Simulation-first workflows
```

**US-9: Loading State**
```
As a user who clicked "Personalize for Me"
While the content is being generated
Then I see:
  - A loading spinner overlay
  - A message "Personalizing content for you..."
  - The button disabled to prevent duplicate requests
```

**US-10: Personalization Badge**
```
As a user viewing personalized content
After personalization completes
Then I see a badge displaying:
  - "Personalized for: Advanced user with Jetson & RTX GPU"
  - Or appropriate variation based on my profile
  - The badge is visually distinct and informative
```

### 2.2 Secondary User Stories

**US-11: Non-Authenticated Users**
```
As a non-logged-in user
When I view any chapter page
Then I do NOT see the "Personalize for Me" button
And the content shows the default generic version
```

**US-12: Error Handling**
```
As a user
When personalization fails (network error, AI service down, etc.)
Then I see:
  - An error message explaining what happened
  - The original chapter content remains visible
  - An option to retry personalization
```

**US-13: Reset to Default**
```
As a user viewing personalized content
When I want to see the original content
Then I can click "Reset to Default"
And the chapter reverts to the original generic version
```

---

## 3. Functional Requirements

### 3.1 Authentication & Authorization

**FR-1.1:** The "Personalize for Me" button MUST only be visible to authenticated users
**FR-1.2:** Authentication status MUST be checked via `useUserProfile()` hook from Better-Auth
**FR-1.3:** Unauthenticated users MUST NOT see the button or any personalization UI
**FR-1.4:** The system MUST verify the user has a valid JWT token before processing personalization

### 3.2 User Profile Retrieval

**FR-2.1:** User profile MUST be fetched from Neon DB via Better-Auth session
**FR-2.2:** Profile MUST include:
- `experience`: "beginner" | "intermediate" | "advanced"
- `hasRTX`: boolean
- `hasJetson`: boolean
- `hasRobot`: boolean
- `user_email`: string

**FR-2.3:** Profile fetch MUST happen on button click (not on page load) to reduce DB load
**FR-2.4:** If profile is incomplete, system MUST use default values:
- Default experience: "intermediate"
- Default hardware: all false

### 3.3 Content Personalization Logic

**FR-3.1:** Personalization MUST call backend API endpoint: `POST /api/personalize/chapter`
**FR-3.2:** API request payload MUST include:
```json
{
  "chapterId": "string",        // e.g., "01-intro/ros2-basics"
  "originalContent": "string",  // Full MDX content
  "userProfile": {
    "experience": "beginner|intermediate|advanced",
    "hasRTX": boolean,
    "hasJetson": boolean,
    "hasRobot": boolean
  }
}
```

**FR-3.3:** Backend MUST use Claude Agent SDK `@personalizer` subagent with prompt:
```
You are a technical content personalizer for a Physical AI & Robotics textbook.

User Profile:
- Experience: {experience}
- Hardware: RTX GPU: {hasRTX}, Jetson: {hasJetson}, Real Robot: {hasRobot}

Original Chapter Content:
{originalContent}

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
```

**FR-3.4:** Personalized content MUST maintain:
- Valid MDX syntax
- All code blocks functional
- Original chapter structure (headings, sections)
- Links and references intact

**FR-3.5:** Content generation MUST complete within 15 seconds or timeout
**FR-3.6:** If generation fails, original content MUST remain displayed

### 3.4 UI Components

**FR-4.1: Personalize Button**
- Position: Top of chapter, below title, above content
- Text: "✨ Personalize for Me"
- Style: Gradient button matching navbar CTA style
- Hover: Show tooltip "Adapt this chapter to your experience and hardware"
- Disabled state while loading

**FR-4.2: Loading Overlay**
- Semi-transparent backdrop over chapter content
- Centered spinner animation
- Text: "Personalizing content for you..."
- Subtext: "This may take 5-10 seconds"

**FR-4.3: Personalization Badge**
- Position: Below button, above content
- Text format: "📌 Personalized for: {experience} user with {hardware}"
- Examples:
  - "📌 Personalized for: Beginner with RTX GPU"
  - "📌 Personalized for: Advanced user with Jetson & Real Robot"
  - "📌 Personalized for: Intermediate user (simulation-focused)"
- Style: Subtle badge with blue/purple gradient border
- Include "Reset to Default" link

**FR-4.4: Reset Button**
- Text: "Reset to Default"
- Position: Inside personalization badge
- Action: Restore original chapter content
- Confirmation: Optional "Are you sure?" if content significantly changed

**FR-4.5: Error Message**
- Display when personalization fails
- Text: "❌ Personalization failed. Please try again."
- Include retry button
- Show error details in development mode only

### 3.5 Content Caching & Performance

**FR-5.1:** Personalized content MUST be cached in browser localStorage
**FR-5.2:** Cache key format: `personalized_chapter_{chapterId}_{userId}_{profileHash}`
**FR-5.3:** Cache expiry: 7 days or until profile changes
**FR-5.4:** On subsequent visits, load from cache instantly
**FR-5.5:** Show "Using cached personalization" message when loading from cache
**FR-5.6:** Provide "Regenerate" option to force fresh personalization

### 3.6 Analytics & Tracking

**FR-6.1:** Track personalization button clicks (chapter ID, user ID, profile)
**FR-6.2:** Track personalization success/failure rates
**FR-6.3:** Track time spent on personalized vs. default content
**FR-6.4:** Track which experience levels personalize most frequently
**FR-6.5:** Log personalization errors for debugging

---

## 4. Non-Functional Requirements

### 4.1 Performance

**NFR-1.1:** Personalization API response time MUST be ≤ 10 seconds (p95)
**NFR-1.2:** Cached personalized content MUST load in ≤ 100ms
**NFR-1.3:** Button click to loading state MUST feel instant (< 50ms)
**NFR-1.4:** Page MUST remain responsive during personalization
**NFR-1.5:** Concurrent personalization requests MUST be queued (max 3 per user)

### 4.2 Scalability

**NFR-2.1:** System MUST handle 100 concurrent personalization requests
**NFR-2.2:** AI API MUST have rate limiting (10 requests/minute per user)
**NFR-2.3:** Database queries MUST use connection pooling
**NFR-2.4:** Static chapter content MUST be served via CDN

### 4.3 Reliability

**NFR-3.1:** Personalization failure MUST NOT break page rendering
**NFR-3.2:** Original content MUST always be available as fallback
**NFR-3.3:** System MUST gracefully handle AI API timeouts
**NFR-3.4:** Uptime target: 99.5% for personalization service

### 4.4 Security

**NFR-4.1:** User profile data MUST be transmitted over HTTPS only
**NFR-4.2:** JWT tokens MUST be validated on every personalization request
**NFR-4.3:** AI prompts MUST NOT expose sensitive user data
**NFR-4.4:** Personalized content MUST be scoped to user (no cross-user leaks)
**NFR-4.5:** Rate limiting MUST prevent abuse (max 20 personalizations/hour/user)

### 4.5 Accessibility

**NFR-5.1:** Button MUST be keyboard accessible (Tab navigation)
**NFR-5.2:** Loading state MUST announce to screen readers
**NFR-5.3:** Error messages MUST have ARIA live regions
**NFR-5.4:** Personalization badge MUST have proper semantic HTML
**NFR-5.5:** WCAG 2.1 Level AA compliance required

### 4.6 Maintainability

**NFR-6.1:** Personalization logic MUST be isolated in dedicated module
**NFR-6.2:** AI prompts MUST be configurable (not hardcoded)
**NFR-6.3:** Content transformation MUST be unit testable
**NFR-6.4:** Error handling MUST provide actionable debug information

---

## 5. Technical Design

### 5.1 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Frontend (React/Docusaurus)              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Chapter Page (MDX)                                     │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │ PersonalizeButton Component                       │  │ │
│  │  │  - useUserProfile() → Check auth                  │  │ │
│  │  │  - onClick → fetch profile & personalize          │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │ PersonalizedContent Component                     │  │ │
│  │  │  - Displays personalized MDX                      │  │ │
│  │  │  - Shows badge with profile info                  │  │ │
│  │  │  - Handles loading/error states                   │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────┘ │
└────────────────────────┬────────────────────────────────────┘
                         │ HTTP POST /api/personalize/chapter
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                   Backend (FastAPI)                          │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  POST /api/personalize/chapter                         │ │
│  │  1. Verify JWT token                                   │ │
│  │  2. Fetch user profile from Neon DB                    │ │
│  │  3. Call @personalizer subagent (Claude Agent SDK)     │ │
│  │  4. Validate personalized MDX                          │ │
│  │  5. Return personalized content                        │ │
│  └────────────────────────────────────────────────────────┘ │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              @personalizer Subagent (Claude Agent SDK)       │
│  - Receives: original content + user profile                │
│  - Applies: experience-level transformations                │
│  - Applies: hardware-specific customizations                │
│  - Validates: MDX syntax & technical accuracy               │
│  - Returns: Personalized MDX content                        │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 File Structure

```
ai_robotics_book/
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   └── Personalize/
│   │   │       ├── PersonalizeButton.tsx          # Main button component
│   │   │       ├── PersonalizedContent.tsx        # Personalized content display
│   │   │       ├── PersonalizationBadge.tsx       # Profile badge
│   │   │       ├── PersonalizeLoadingOverlay.tsx  # Loading state
│   │   │       └── personalize.module.css         # Styles
│   │   ├── hooks/
│   │   │   └── usePersonalization.ts              # Custom hook for logic
│   │   ├── services/
│   │   │   └── personalizationService.ts          # API calls
│   │   └── theme/
│   │       └── MDXComponents/
│   │           └── index.tsx                      # Wrap MDX with personalization
│   └── docs/
│       └── **/*.md                                # All chapters
├── backend/
│   ├── src/
│   │   ├── api/
│   │   │   └── personalize.py                     # Personalization endpoint
│   │   ├── services/
│   │   │   └── personalization_service.py         # Core logic
│   │   └── agents/
│   │       └── personalizer_agent.py              # Claude Agent SDK integration
│   └── prompts/
│       └── personalization_prompt.md              # AI prompt template
└── specs/
    └── personalize-button/
        ├── spec.md                                # This file
        ├── plan.md                                # Implementation plan
        └── tasks.md                               # Task breakdown
```

### 5.3 Component APIs

#### PersonalizeButton Component
```typescript
interface PersonalizeButtonProps {
  chapterId: string;
  originalContent: string;
  onPersonalizationComplete?: (content: string) => void;
  onPersonalizationError?: (error: Error) => void;
}

export function PersonalizeButton({
  chapterId,
  originalContent,
  onPersonalizationComplete,
  onPersonalizationError
}: PersonalizeButtonProps): JSX.Element;
```

#### usePersonalization Hook
```typescript
interface UsePersonalizationResult {
  isPersonalizing: boolean;
  personalizedContent: string | null;
  error: Error | null;
  personalizeContent: () => Promise<void>;
  resetToDefault: () => void;
  isFromCache: boolean;
}

export function usePersonalization(
  chapterId: string,
  originalContent: string
): UsePersonalizationResult;
```

#### Backend API Endpoint
```python
@router.post("/api/personalize/chapter")
async def personalize_chapter(
    request: PersonalizeChapterRequest,
    current_user: User = Depends(get_current_user)
) -> PersonalizeChapterResponse:
    """
    Personalize chapter content based on user profile.

    Args:
        request: Chapter ID and original content
        current_user: Authenticated user from JWT

    Returns:
        Personalized MDX content and profile metadata

    Raises:
        HTTPException: 401 if unauthenticated, 500 if AI service fails
    """
```

### 5.4 Database Schema

No new tables required. Uses existing `user_profiles` table from Better-Auth:

```sql
-- Existing schema (no changes needed)
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    experience VARCHAR(20) CHECK (experience IN ('beginner', 'intermediate', 'advanced')),
    has_rtx BOOLEAN DEFAULT FALSE,
    has_jetson BOOLEAN DEFAULT FALSE,
    has_robot BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Add index for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

### 5.5 API Contracts

#### Request Schema
```typescript
interface PersonalizeChapterRequest {
  chapterId: string;           // e.g., "01-intro/ros2-basics"
  originalContent: string;     // Full MDX content
}
```

#### Response Schema
```typescript
interface PersonalizeChapterResponse {
  success: boolean;
  personalizedContent: string;  // Personalized MDX
  profile: {
    experience: "beginner" | "intermediate" | "advanced";
    hasRTX: boolean;
    hasJetson: boolean;
    hasRobot: boolean;
  };
  metadata: {
    generationTime: number;     // milliseconds
    cached: boolean;
    profileHash: string;        // For cache validation
  };
}
```

#### Error Response Schema
```typescript
interface PersonalizeChapterError {
  success: false;
  error: string;                // User-friendly error message
  code: string;                 // Error code (e.g., "AI_SERVICE_TIMEOUT")
  retryable: boolean;           // Whether user can retry
}
```

---

## 6. User Experience Flow

### 6.1 Happy Path: First-Time Personalization

```
1. User logs in → navigates to Chapter 1: ROS 2 Basics
2. User sees "✨ Personalize for Me" button at top of chapter
3. User hovers → tooltip: "Adapt this chapter to your experience and hardware"
4. User clicks button
   ↓
5. Loading overlay appears
   - Semi-transparent backdrop
   - Spinner animation
   - Text: "Personalizing content for you..."
   ↓
6. Backend processes request (5-8 seconds)
   - Fetches profile: Beginner, has RTX GPU, no Jetson
   - Calls @personalizer subagent
   - Generates personalized content
   ↓
7. Content updates smoothly
   - Badge appears: "📌 Personalized for: Beginner with RTX GPU"
   - Chapter content now has:
     * More detailed explanations
     * Analogies comparing ROS nodes to familiar concepts
     * Heavily commented code examples
     * Callout: "💡 Tip: You can run this example locally on your RTX GPU"
     * Step-by-step setup instructions
   ↓
8. User reads personalized content
9. Content cached in localStorage for next visit
```

### 6.2 Returning User Path

```
1. User returns to same chapter
2. Button shows "✨ Personalize for Me"
3. User clicks button
   ↓
4. System checks cache
   - Cache hit! Personalized content found
   - Profile hasn't changed
   ↓
5. Content loads instantly (<100ms)
   - Badge: "📌 Personalized for: Beginner with RTX GPU (cached)"
   - Small note: "Using cached personalization • Regenerate"
   ↓
6. User can click "Regenerate" for fresh personalization if desired
```

### 6.3 Error Path

```
1. User clicks "Personalize for Me"
2. Loading overlay appears
3. Backend request times out (network issue)
   ↓
4. Error message displays:
   "❌ Personalization failed. Please try again."
   [Retry Button]
   ↓
5. Original content remains visible
6. User can retry or continue reading default content
```

### 6.4 Reset to Default Path

```
1. User viewing personalized content
2. User clicks "Reset to Default" link in badge
3. Optional confirmation: "Reset to original content?"
   ↓
4. Content reverts to default immediately
5. Badge disappears
6. Button returns to "✨ Personalize for Me" state
7. Cache cleared for this chapter
```

---

## 7. Edge Cases & Error Handling

### 7.1 Edge Cases

**EC-1: User has incomplete profile**
- Scenario: User signed up but didn't complete all profile questions
- Handling: Use default values (intermediate, no hardware) and show notice:
  "⚠️ Complete your profile for better personalization"

**EC-2: User changes profile mid-session**
- Scenario: User updates profile in another tab
- Handling: Invalidate cache, detect profile change on next personalization

**EC-3: Chapter already personalized**
- Scenario: User clicks button on already-personalized content
- Handling: Show confirmation: "Regenerate personalized content?"

**EC-4: Multiple rapid clicks**
- Scenario: User spam-clicks "Personalize for Me"
- Handling: Disable button, show single loading overlay, queue only one request

**EC-5: Very long chapter (>10,000 words)**
- Scenario: Chapter content exceeds AI token limit
- Handling: Split into sections, personalize separately, merge results

**EC-6: Invalid MDX in personalized output**
- Scenario: AI generates broken MDX syntax
- Handling: Validate MDX, retry once, fallback to original if still invalid

**EC-7: User logs out while personalizing**
- Scenario: Auth session expires during personalization
- Handling: Cancel request, clear UI, show login prompt

**EC-8: Offline user**
- Scenario: User loses internet connection
- Handling: Show offline message, load cached content if available

### 7.2 Error Handling Matrix

| Error Type | User Message | Technical Action | Retryable? |
|------------|--------------|------------------|------------|
| Auth Expired | "Please log in again" | Redirect to /signin | No |
| Network Timeout | "Connection timeout. Try again." | None | Yes |
| AI Service Down | "Service temporarily unavailable" | Log error, alert devs | Yes |
| Invalid Profile | "Profile data incomplete" | Use defaults | No |
| Rate Limit Hit | "Too many requests. Wait 1 minute." | Return 429 | Yes (after wait) |
| Invalid MDX Output | "Generation failed. Using default." | Fallback to original | Yes |
| DB Connection Lost | "Database error. Try again." | Retry with backoff | Yes |
| Cache Corrupted | "Loading fresh content..." | Clear cache, regenerate | No |

### 7.3 Error Recovery Strategies

**Strategy 1: Graceful Degradation**
- Always keep original content accessible
- Never block page render on personalization failure
- Show clear error messages with actionable steps

**Strategy 2: Retry Logic**
- Automatic retry: Network timeouts (1 retry with exponential backoff)
- Manual retry: User-triggered via "Retry" button
- Max retries: 3 attempts before showing persistent error

**Strategy 3: Fallback Chain**
1. Try personalized generation
2. If fails, try cached version
3. If no cache, show original content
4. Never show broken/empty page

**Strategy 4: Monitoring & Alerts**
- Log all personalization errors with context
- Alert on >5% failure rate
- Track error types in dashboard
- Weekly error analysis report

---

## 8. Testing Strategy

### 8.1 Unit Tests

**Frontend Components:**
- ✅ PersonalizeButton renders only when authenticated
- ✅ PersonalizeButton hidden when not authenticated
- ✅ PersonalizeButton disables during loading
- ✅ PersonalizationBadge displays correct profile text
- ✅ usePersonalization hook caches results correctly
- ✅ usePersonalization hook invalidates cache on profile change
- ✅ Error messages display correctly

**Backend Services:**
- ✅ Profile fetch retrieves correct user data
- ✅ AI prompt generation includes all profile fields
- ✅ MDX validation catches syntax errors
- ✅ Rate limiting enforces 10 req/min limit
- ✅ Token validation rejects invalid JWTs

### 8.2 Integration Tests

**End-to-End Flows:**
- ✅ Full personalization flow (button click → content update)
- ✅ Cache hit scenario (instant load)
- ✅ Cache miss scenario (full generation)
- ✅ Profile change invalidates cache
- ✅ Reset to default works correctly
- ✅ Multiple chapters can be personalized independently

**API Integration:**
- ✅ POST /api/personalize/chapter returns valid MDX
- ✅ Endpoint validates JWT correctly
- ✅ Endpoint handles missing profile gracefully
- ✅ Endpoint respects rate limits
- ✅ Endpoint times out after 15 seconds

### 8.3 AI Quality Tests

**Content Accuracy:**
- ✅ Beginner content has more explanations than intermediate
- ✅ Advanced content includes optimization tips
- ✅ RTX GPU mentions appear for users with RTX
- ✅ Jetson deployment tips appear for Jetson users
- ✅ Cloud training suggested for users without hardware
- ✅ Code examples remain functional after personalization
- ✅ Technical facts remain accurate

**Content Structure:**
- ✅ All original headings preserved
- ✅ Code blocks maintain syntax highlighting
- ✅ Links remain intact
- ✅ Images/diagrams preserved
- ✅ MDX components render correctly

### 8.4 Performance Tests

**Load Testing:**
- ✅ 100 concurrent personalization requests handled
- ✅ P95 response time < 10 seconds
- ✅ Cached content loads in < 100ms
- ✅ Database connection pool doesn't saturate
- ✅ AI API rate limits respected

**Stress Testing:**
- ✅ System handles 500 concurrent users
- ✅ Graceful degradation under high load
- ✅ No memory leaks during extended operation

### 8.5 Accessibility Tests

**WCAG Compliance:**
- ✅ Button accessible via keyboard (Tab, Enter)
- ✅ Loading state announced to screen readers
- ✅ Error messages have ARIA live regions
- ✅ Badge has semantic HTML
- ✅ Color contrast meets AA standards (4.5:1)
- ✅ Focus indicators visible

### 8.6 Security Tests

**Authentication & Authorization:**
- ✅ Unauthenticated requests rejected (401)
- ✅ Invalid JWT tokens rejected
- ✅ User A cannot access User B's profile
- ✅ Rate limiting prevents abuse
- ✅ No SQL injection in profile queries
- ✅ No XSS in personalized content output

### 8.7 Manual Test Scenarios

**Test Case 1: Beginner with RTX GPU**
```
Given: User profile = Beginner, has RTX
When: User personalizes Chapter 3 (Isaac Sim)
Then: Content should mention:
  - "Since you have an RTX GPU, you can run Isaac Sim locally"
  - Detailed setup instructions for local installation
  - GPU driver requirements explained
  - Analogies for physics simulation concepts
```

**Test Case 2: Advanced with Jetson**
```
Given: User profile = Advanced, has Jetson
When: User personalizes Chapter 8 (Deployment)
Then: Content should mention:
  - Edge deployment optimization techniques
  - Jetson-specific inference acceleration
  - Power consumption considerations
  - Minimal explanations (assumes prior knowledge)
```

**Test Case 3: Intermediate with no hardware**
```
Given: User profile = Intermediate, no hardware
When: User personalizes Chapter 5 (Training)
Then: Content should mention:
  - Cloud training options (AWS, GCP, Azure)
  - Cost optimization tips
  - Remote robot control via cloud
  - Balanced explanations (not too basic/advanced)
```

---

## 9. Rollout Plan

### 9.1 Phase 1: Alpha (Week 1-2)

**Scope:** Internal testing only
- Deploy to dev environment
- Enable for 5 test users
- Test all 3 experience levels
- Validate AI quality manually

**Success Criteria:**
- ✅ All unit tests pass
- ✅ 90%+ personalization success rate
- ✅ No crashes or data corruption
- ✅ AI content deemed accurate by 3 reviewers

### 9.2 Phase 2: Beta (Week 3-4)

**Scope:** Limited user testing
- Deploy to staging environment
- Enable for 50 beta users (opt-in)
- Monitor error rates and performance
- Collect user feedback via survey

**Success Criteria:**
- ✅ 85%+ personalization success rate in production
- ✅ P95 response time < 10 seconds
- ✅ No critical bugs reported
- ✅ 70%+ beta users rate feature 4/5 or higher

### 9.3 Phase 3: General Availability (Week 5)

**Scope:** All users
- Deploy to production
- Enable for all authenticated users
- Monitor at scale
- Marketing announcement

**Success Criteria:**
- ✅ 95%+ uptime
- ✅ 80%+ personalization success rate
- ✅ 50%+ of users try personalization within first week
- ✅ No major incidents

### 9.4 Phase 4: Optimization (Week 6+)

**Scope:** Performance improvements
- Optimize AI prompts based on usage data
- Improve caching strategy
- A/B test UI variations
- Add advanced features (e.g., manual customization)

---

## 10. Dependencies

### 10.1 External Dependencies

| Dependency | Purpose | Version | Critical? |
|------------|---------|---------|-----------|
| Claude API | AI content generation | Latest | Yes |
| Claude Agent SDK | @personalizer subagent | Latest | Yes |
| Better-Auth | User authentication | 1.x | Yes |
| Neon DB (PostgreSQL) | User profile storage | Latest | Yes |
| React | Frontend framework | 18.x | Yes |
| Docusaurus | Static site generator | 3.x | Yes |
| FastAPI | Backend API | 0.100+ | Yes |

### 10.2 Internal Dependencies

- ✅ Better-Auth integration completed (already done)
- ✅ User profile schema in Neon DB (already done)
- ✅ Frontend auth hooks (`useUserProfile`) available
- ⚠️ Backend personalization endpoint (needs to be built)
- ⚠️ @personalizer subagent (needs to be created)
- ⚠️ MDX component wrapper (needs to be added)

### 10.3 Blocked By

- None (all prerequisites met)

### 10.4 Blocking

- Advanced features (e.g., manual customization, difficulty slider)
- Multi-language personalization
- Voice-based content delivery

---

## 11. Open Questions

### 11.1 Technical Questions

**Q1:** Should we use streaming for personalized content (display as it generates)?
- **Pro:** Better perceived performance, users see progress
- **Con:** Complexity in handling partial MDX, harder to validate
- **Decision:** TBD - need to test streaming viability with MDX

**Q2:** How to handle very long chapters (>10K words)?
- **Option A:** Split into sections, personalize separately
- **Option B:** Summarize less critical sections
- **Option C:** Limit personalization to key sections only
- **Decision:** TBD - need to analyze chapter length distribution

**Q3:** Should personalized content be stored in DB for future access?
- **Pro:** No regeneration needed, faster across devices
- **Con:** Storage cost, stale content if original chapter updates
- **Decision:** TBD - need to assess storage vs. regeneration cost

### 11.2 Product Questions

**Q4:** Should we allow manual override of personalization settings?
- Example: "I'm Advanced but show me Beginner-level explanations for this chapter"
- **Pro:** More flexibility, users control experience
- **Con:** Added complexity, confusion
- **Decision:** Phase 2 feature after GA

**Q5:** Should we show a diff/comparison view of original vs. personalized?
- **Pro:** Transparency, educational value
- **Con:** UI complexity, might confuse users
- **Decision:** TBD - user research needed

**Q6:** How to handle chapters that update frequently?
- Cache might become stale if original content changes
- **Option A:** Invalidate cache when chapter updates (needs version tracking)
- **Option B:** Show "Content updated, regenerate?" prompt
- **Option C:** Auto-regenerate on chapter version change
- **Decision:** TBD - need chapter versioning strategy

### 11.3 Business Questions

**Q7:** What's the AI API cost per personalization?
- Estimated ~5,000 tokens per chapter = $0.15 per personalization
- At 1,000 users × 20 chapters = $3,000 total
- **Decision:** Acceptable cost, but monitor usage

**Q8:** Should personalization be a premium feature?
- **Pro:** Revenue opportunity
- **Con:** Reduces engagement for free users
- **Decision:** Free for all users initially, reassess after 3 months

---

## 12. Success Criteria

### 12.1 Launch Criteria (Must-Have for GA)

- ✅ All functional requirements implemented
- ✅ 95%+ personalization success rate in staging
- ✅ P95 response time < 10 seconds
- ✅ All security tests pass
- ✅ WCAG 2.1 Level AA compliance
- ✅ No critical bugs in beta
- ✅ Positive feedback from 70%+ beta users

### 12.2 Post-Launch Success Metrics (30 days)

**Adoption Metrics:**
- 60%+ of logged-in users try personalization
- Average 8+ chapters personalized per user
- 40%+ users return to use personalization again

**Quality Metrics:**
- 95%+ personalization success rate in production
- P95 response time < 10 seconds maintained
- < 5% error rate
- < 1% invalid MDX outputs

**Engagement Metrics:**
- 50%+ increase in time-on-page for personalized content
- 30%+ increase in chapter completion rate
- 20%+ increase in return visits

**User Satisfaction:**
- 4.2/5 average rating for personalization feature
- 75%+ users find personalized content "helpful" or "very helpful"
- < 5% requests to disable/remove feature

### 12.3 Long-Term Success Metrics (90 days)

- 80%+ of active users use personalization regularly
- 2x increase in course completion rate vs. pre-personalization
- Personalization cited as top 3 feature in user surveys
- AI cost per user < $0.50/month (sustainable)

---

## 13. Risks & Mitigations

### 13.1 Technical Risks

**Risk 1: AI generates inaccurate content**
- **Impact:** High - damages trust, educational integrity
- **Probability:** Medium
- **Mitigation:**
  - Validate technical facts with reference check
  - Human review of sample personalizations before GA
  - User "Report Error" button to flag issues
  - Continuous monitoring of reported issues

**Risk 2: Personalization takes too long**
- **Impact:** Medium - poor UX, user abandons feature
- **Probability:** Medium
- **Mitigation:**
  - Optimize AI prompts for faster generation
  - Pre-generate common profile combinations
  - Implement aggressive caching
  - Show progress indicator to set expectations

**Risk 3: Cache inconsistency**
- **Impact:** Medium - stale content shown
- **Probability:** Low
- **Mitigation:**
  - Version tracking for chapter content
  - Cache invalidation on chapter updates
  - TTL of 7 days maximum
  - "Regenerate" option always available

**Risk 4: AI API rate limits hit**
- **Impact:** High - feature unavailable
- **Probability:** Low (with monitoring)
- **Mitigation:**
  - Implement request queuing
  - Multiple AI API keys for failover
  - Clear error messages with retry timing
  - Monitor usage and upgrade plan proactively

### 13.2 Product Risks

**Risk 5: Users overwhelmed by feature**
- **Impact:** Low - feature ignored
- **Probability:** Medium
- **Mitigation:**
  - Clear onboarding tooltip on first visit
  - Optional tutorial video
  - Prominent examples of personalization benefits
  - A/B test button placement/text

**Risk 6: Personalization creates accessibility issues**
- **Impact:** High - excludes users
- **Probability:** Low (with testing)
- **Mitigation:**
  - WCAG compliance testing before GA
  - Screen reader testing with real users
  - Keyboard navigation verification
  - High contrast mode support

**Risk 7: Users expect perfect personalization**
- **Impact:** Medium - disappointment, negative reviews
- **Probability:** Medium
- **Mitigation:**
  - Set realistic expectations in UI ("AI-powered suggestions")
  - Feedback mechanism to improve over time
  - Always show "Reset to Default" option
  - Gradual rollout to manage expectations

### 13.3 Business Risks

**Risk 8: High AI API costs**
- **Impact:** High - unsustainable economics
- **Probability:** Low (with monitoring)
- **Mitigation:**
  - Aggressive caching (reduces regenerations)
  - Cost monitoring dashboard
  - Usage caps per user (20 personalizations/hour)
  - Pre-generate popular profiles

**Risk 9: Competitive feature copying**
- **Impact:** Low - loss of differentiation
- **Probability:** High
- **Mitigation:**
  - Continuous innovation (new personalization dimensions)
  - Quality of personalization as moat
  - Ecosystem lock-in (Better-Auth integration)
  - Patent/IP protection (if applicable)

---

## 14. Future Enhancements (Post-MVP)

### 14.1 Phase 2 Features (3-6 months)

**F1: Personalization Slider**
- Allow users to adjust personalization intensity
- Slider: Simplified ←→ Standard ←→ Advanced
- Override default profile setting for specific chapter

**F2: Multi-Language Personalization**
- Translate personalized content to user's language
- Preserve technical accuracy across languages
- Support: English, Spanish, Chinese, Hindi

**F3: Learning Path Recommendations**
- Suggest next chapters based on profile and progress
- Skip beginner chapters for advanced users
- Highlight hardware-relevant chapters

**F4: Collaborative Personalization**
- Share personalized content with team/classmates
- Annotate and discuss personalized sections
- Team profile (e.g., entire class has Jetson kits)

### 14.2 Advanced Features (6-12 months)

**F5: Voice-Based Content Delivery**
- Text-to-speech for personalized content
- Adjust narration speed/voice to preference
- Audio-only mode for hands-free learning

**F6: Interactive Exercises**
- Personalized coding challenges
- Difficulty adapts to user performance
- Hints tailored to experience level

**F7: Visual Learning Preferences**
- More diagrams for visual learners
- More code for hands-on learners
- Video snippets for demonstration

**F8: Peer Comparison**
- Anonymous comparison with similar profiles
- "Users like you found this section challenging"
- Gamification elements (optional)

### 14.3 Research Ideas (12+ months)

**F9: Neuro-Adaptive Learning**
- Track reading patterns (time per section, scrolling)
- Dynamically adjust difficulty mid-chapter
- Predict confusion before user realizes it

**F10: AR/VR Personalized Content**
- 3D visualizations of robot kinematics
- Spatial computing for Isaac Sim tutorials
- Hardware-specific AR overlays

---

## 15. Appendices

### 15.1 Glossary

- **MDX:** Markdown with JSX components, used by Docusaurus
- **@personalizer subagent:** Claude Agent SDK agent specialized in content personalization
- **Better-Auth:** Authentication library used for user management
- **Neon DB:** Serverless PostgreSQL database
- **Claude Agent SDK:** Framework for building AI agents with tool use
- **JWT:** JSON Web Token for authentication
- **Profile Hash:** Hash of user profile for cache key generation
- **TTL:** Time To Live, cache expiration duration

### 15.2 References

- [Docusaurus MDX Documentation](https://docusaurus.io/docs/markdown-features)
- [Better-Auth Documentation](https://better-auth.com)
- [Claude Agent SDK Documentation](https://github.com/anthropics/claude-agent-sdk)
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [FastAPI Best Practices](https://fastapi.tiangolo.com/tutorial/)

### 15.3 Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-06 | Claude | Initial specification |

---

**End of Specification**

**Next Steps:**
1. Review and approve specification
2. Create implementation plan (`plan.md`)
3. Break down into tasks (`tasks.md`)
4. Begin Phase 1 (Alpha) development
