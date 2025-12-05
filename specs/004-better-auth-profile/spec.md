# Feature Specification: Better-Auth Signup/Signin with Profile Questions

**Feature ID**: 004-better-auth-profile
**Feature Branch**: `004-better-auth-profile`
**Created**: 2025-12-06
**Status**: Draft
**Priority**: P1 (High - 50 Bonus Points)
**Dependencies**: 002-rag-chatbot (reuses same Neon Postgres DATABASE_URL)

---

## Executive Summary

Implement **Better-Auth** (https://www.better-auth.com/) for user authentication with email + password, capturing **4 profile questions** at signup to enable personalized textbook views. Reuses the **existing Neon Postgres database** from the RAG chatbot feature (002-rag-chatbot) by adding a `profile` JSONB column to the users table.

**Key Capabilities**:
- ✅ Signup with email + password (Better-Auth)
- ✅ 4 profile questions at signup (RTX GPU, Jetson, real robot, programming experience)
- ✅ Signin with email + password (simple form)
- ✅ Profile stored in SAME Neon DB (reuses existing `NEON_DATABASE_URL`)
- ✅ Login optional (book readable without authentication)
- ✅ Personalized view after signup (based on profile answers)
- ✅ React components embedded in Docusaurus

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Signup with Profile Questions (Priority: P1)

**As a** new reader visiting the textbook,
**I want to** sign up with my email + password and answer 4 quick profile questions,
**So that** I get a personalized learning experience tailored to my hardware and skill level.

**Why this priority**: Core feature for personalization. Must work flawlessly before launching enhanced features.

**Independent Test**: User completes signup form with email, password, and 4 profile questions → Account created in Neon DB → Redirected to textbook with personalized view → Profile stored in `users.profile` JSONB column.

**Acceptance Scenarios**:

1. **Given** I'm a new user on the homepage,
   **When** I click "Sign Up" button,
   **Then** I see a signup form with:
   - Email input (with validation: valid email format)
   - Password input (min 8 characters, show/hide toggle)
   - 4 profile questions:
     1. "Do you have an RTX GPU?" (Yes / No radio buttons)
     2. "Do you own a Jetson Orin Nano?" (Yes / No radio buttons)
     3. "Do you have access to a real robot?" (Yes / No radio buttons)
     4. "Your programming experience?" (Beginner / Intermediate / Advanced dropdown)
   - "Create Account" submit button
   - "Already have an account? Sign in" link

2. **Given** I fill in email "student@example.com", password "SecurePass123", and answer:
   - RTX GPU: No
   - Jetson: No
   - Real robot: No
   - Experience: Beginner
   **When** I click "Create Account",
   **Then**:
   - Account is created in Neon Postgres `users` table
   - Profile is stored as JSONB: `{"has_rtx_gpu": false, "has_jetson": false, "has_real_robot": false, "programming_experience": "beginner"}`
   - I'm redirected to `/docs/intro` with personalized view
   - Success message: "Welcome! Your learning path is personalized based on your hardware."

3. **Given** I try to signup with email "invalid-email",
   **When** I submit the form,
   **Then** I see validation error: "Please enter a valid email address" (inline, red text below email input)

4. **Given** I try to signup with password "short",
   **When** I submit the form,
   **Then** I see validation error: "Password must be at least 8 characters" (inline, red text below password input)

5. **Given** I try to signup with email already in database,
   **When** I submit the form,
   **Then** I see error: "An account with this email already exists. Please sign in or use a different email."

---

### User Story 2 - Signin (Priority: P1)

**As a** returning user,
**I want to** sign in with my email + password,
**So that** I can access my personalized textbook view and saved progress.

**Why this priority**: Essential for returning users to access personalized content.

**Independent Test**: User enters correct email + password → Authenticated → Redirected to last visited page or `/docs/intro` → Profile loaded from database.

**Acceptance Scenarios**:

1. **Given** I'm a returning user on the homepage,
   **When** I click "Sign In" button,
   **Then** I see a signin form with:
   - Email input
   - Password input (show/hide toggle)
   - "Sign In" submit button
   - "Forgot password?" link (optional for MVP, can be added later)
   - "Don't have an account? Sign up" link

2. **Given** I enter correct email "student@example.com" and password "SecurePass123",
   **When** I click "Sign In",
   **Then**:
   - I'm authenticated (session created)
   - Redirected to last visited page (if stored in localStorage) or `/docs/intro`
   - My profile is loaded: `{"has_rtx_gpu": false, ...}`
   - Personalized content appears (e.g., "Cloud-based tutorials" highlighted for users without Jetson)

3. **Given** I enter incorrect password,
   **When** I submit the form,
   **Then** I see error: "Invalid email or password. Please try again." (no indication of which field is wrong for security)

4. **Given** I enter email not in database,
   **When** I submit the form,
   **Then** I see error: "Invalid email or password. Please try again."

---

### User Story 3 - Optional Login (Priority: P1)

**As a** visitor,
**I want to** read the textbook without creating an account,
**So that** I can browse content freely before deciding to sign up.

**Why this priority**: Critical for accessibility. No paywall, no forced signup (Constitution Principle X: Open Source & Accessible Forever).

**Independent Test**: User visits textbook without logging in → All content is readable → "Sign Up" and "Sign In" buttons visible in navbar → Personalization features disabled (generic view).

**Acceptance Scenarios**:

1. **Given** I visit the textbook homepage without logging in,
   **When** I navigate to any chapter (e.g., `/docs/ros2/nodes`),
   **Then**:
   - All content is readable (no paywall, no blur, no "login to continue")
   - Navbar shows "Sign Up" and "Sign In" buttons
   - No personalized content appears (generic view for all visitors)

2. **Given** I'm browsing as anonymous user,
   **When** I click "Personalize for me" button (on chapters that support personalization),
   **Then** I see a modal:
   - Title: "Sign up to personalize your learning"
   - Message: "Create a free account to get content tailored to your hardware and experience level."
   - "Sign Up" button → Opens signup form
   - "Maybe later" button → Closes modal, continues reading

---

### User Story 4 - Personalized Content Display (Priority: P2)

**As a** logged-in user,
**I want to** see content tailored to my hardware and experience level,
**So that** I focus on tutorials relevant to my setup.

**Why this priority**: Enhances learning experience but not blocking for MVP (can launch with authentication first, add personalization iteratively).

**Independent Test**: User with profile `{"has_jetson": true, "has_rtx_gpu": false, "programming_experience": "intermediate"}` sees:
- Jetson-specific tutorials highlighted
- Cloud-based training tutorials for lacking RTX GPU
- Intermediate-level code examples (skips beginner explanations)

**Acceptance Scenarios**:

1. **Given** I'm logged in with profile: `has_jetson: true, has_rtx_gpu: false, programming_experience: "intermediate"`,
   **When** I visit Module 4 (VLA Models) chapter,
   **Then** I see:
   - ✅ Badge: "Recommended for you" on "Deploy VLA to Jetson" section
   - ⚠️ Note: "You don't have an RTX GPU. We recommend cloud-based training (AWS g5.xlarge). See setup guide."
   - Code examples default to "Intermediate" tab (no "Beginner" explanations unless expanded)

2. **Given** I'm logged in with profile: `has_real_robot: false`,
   **When** I visit deployment tutorials,
   **Then** I see:
   - Note: "Don't have a robot yet? All exercises can be completed in Isaac Sim (simulation). Hardware purchase optional."
   - Simulation-first examples prioritized over real hardware examples

---

### Edge Cases

- **What happens when user changes profile after signup?**
  MVP: Profile is immutable after signup. Future: Add "Update Profile" settings page.

- **What if user forgets password?**
  MVP: No password reset (manual support via GitHub issues). Future: Add "Forgot Password" email flow.

- **What if Better-Auth API is down during signup?**
  Graceful degradation: Show error message "Authentication service temporarily unavailable. Please try again in a few minutes." Book remains readable without login.

- **What if Neon database connection fails?**
  Backend returns HTTP 503 with message: "Database unavailable. Your account will be created once service is restored." Retry mechanism with exponential backoff.

- **What if user signs up without answering profile questions?**
  Validation prevents submission. All 4 questions are REQUIRED (no "Skip" option). Rationale: Profile is core value proposition of authentication.

---

## Requirements *(mandatory)*

### Functional Requirements

#### FR-001: Better-Auth Integration
System MUST integrate Better-Auth library (https://www.better-auth.com/) for authentication with:
- Email + password provider (no OAuth for MVP to reduce complexity)
- Session management (JWT or cookie-based, as per Better-Auth defaults)
- User creation endpoint (`POST /auth/signup`)
- User login endpoint (`POST /auth/signin`)
- Session validation endpoint (`GET /auth/session`)

**Better-Auth Configuration**:
```typescript
// backend/src/auth.config.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "postgres",
    url: process.env.NEON_DATABASE_URL, // Reuse existing Neon DB
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  // Profile questions handled via custom fields (see FR-003)
});
```

---

#### FR-002: Database Schema (Reuse Existing Neon DB)
System MUST extend the existing Neon Postgres database (from 002-rag-chatbot) by adding Better-Auth tables and a `profile` column:

**Migration 1: Better-Auth Tables** (auto-created by Better-Auth on first run):
```sql
-- Better-Auth creates these tables automatically:
-- - users (id, email, password_hash, email_verified, created_at, updated_at)
-- - sessions (id, user_id, expires_at, created_at)
-- - verification_tokens (id, user_id, token, expires_at)
```

**Migration 2: Add Profile Column** (manual migration):
```sql
-- Add profile JSONB column to users table
ALTER TABLE users ADD COLUMN IF NOT EXISTS profile JSONB DEFAULT '{}'::JSONB;

-- Create index for profile queries (optional, for performance)
CREATE INDEX IF NOT EXISTS idx_users_profile ON users USING GIN (profile);

-- Example profile JSON structure:
-- {
--   "has_rtx_gpu": false,
--   "has_jetson": true,
--   "has_real_robot": false,
--   "programming_experience": "intermediate",
--   "created_at": "2025-12-06T10:30:00Z"
-- }
```

**Database Connection**:
- Reuse existing `NEON_DATABASE_URL` from `.env` (no new database needed)
- Better-Auth automatically creates its tables on first `betterAuth()` initialization
- Profile column added via migration script: `backend/src/migrations/add_profile_column.sql`

---

#### FR-003: Signup Endpoint with Profile Questions
System MUST provide a **custom signup endpoint** that:
1. Validates email (format, not already exists)
2. Validates password (min 8 characters)
3. Creates user via Better-Auth
4. Stores profile answers in `users.profile` JSONB column

**Endpoint**: `POST /api/auth/signup`

**Request Body**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123",
  "profile": {
    "has_rtx_gpu": false,
    "has_jetson": false,
    "has_real_robot": false,
    "programming_experience": "beginner"
  }
}
```

**Response** (Success - HTTP 201):
```json
{
  "success": true,
  "user": {
    "id": "user_uuid",
    "email": "student@example.com",
    "profile": {
      "has_rtx_gpu": false,
      "has_jetson": false,
      "has_real_robot": false,
      "programming_experience": "beginner"
    }
  },
  "session": {
    "token": "jwt_token_here",
    "expires_at": "2025-12-13T10:30:00Z"
  }
}
```

**Response** (Error - HTTP 400):
```json
{
  "success": false,
  "error": "Email already exists"
}
```

**Backend Implementation** (FastAPI):
```python
# backend/src/api/auth.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, EmailStr
from .auth_service import create_user_with_profile

router = APIRouter(prefix="/api/auth")

class SignupRequest(BaseModel):
    email: EmailStr
    password: str  # min_length=8 validated by Pydantic
    profile: dict  # {has_rtx_gpu, has_jetson, has_real_robot, programming_experience}

@router.post("/signup")
async def signup(request: SignupRequest):
    try:
        user, session = await create_user_with_profile(
            email=request.email,
            password=request.password,
            profile=request.profile
        )
        return {"success": True, "user": user, "session": session}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
```

---

#### FR-004: Signin Endpoint
System MUST provide signin via Better-Auth default endpoint or custom wrapper:

**Endpoint**: `POST /api/auth/signin`

**Request Body**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123"
}
```

**Response** (Success - HTTP 200):
```json
{
  "success": true,
  "user": {
    "id": "user_uuid",
    "email": "student@example.com",
    "profile": {
      "has_rtx_gpu": false,
      "has_jetson": false,
      "has_real_robot": false,
      "programming_experience": "beginner"
    }
  },
  "session": {
    "token": "jwt_token_here",
    "expires_at": "2025-12-13T10:30:00Z"
  }
}
```

**Response** (Error - HTTP 401):
```json
{
  "success": false,
  "error": "Invalid email or password"
}
```

---

#### FR-005: Session Validation
System MUST validate user sessions on protected routes:

**Endpoint**: `GET /api/auth/session`

**Headers**: `Authorization: Bearer <jwt_token>`

**Response** (Valid - HTTP 200):
```json
{
  "authenticated": true,
  "user": {
    "id": "user_uuid",
    "email": "student@example.com",
    "profile": { ... }
  }
}
```

**Response** (Invalid - HTTP 401):
```json
{
  "authenticated": false,
  "error": "Session expired or invalid"
}
```

---

#### FR-006: React Signup Component (Docusaurus Embedded)
System MUST provide a React signup form component embedded in Docusaurus:

**Location**: `frontend/src/components/Auth/SignupForm.tsx`

**Component Structure**:
```typescript
import React, { useState } from 'react';

interface SignupFormProps {
  onSuccess?: () => void;
}

export default function SignupForm({ onSuccess }: SignupFormProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [profile, setProfile] = useState({
    has_rtx_gpu: null,
    has_jetson: null,
    has_real_robot: null,
    programming_experience: ''
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    // Validation
    if (!email || !password || profile.has_rtx_gpu === null || ...) {
      setError('Please fill in all fields');
      setLoading(false);
      return;
    }

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password, profile })
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.error || 'Signup failed');
        return;
      }

      // Store session token
      localStorage.setItem('auth_token', data.session.token);

      // Redirect to textbook
      window.location.href = '/docs/intro?personalized=true';

      if (onSuccess) onSuccess();
    } catch (err) {
      setError('Network error. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="signup-form">
      <h2>Create Your Account</h2>
      <p className="subtitle">Get personalized learning in 2 minutes</p>

      {/* Email Input */}
      <div className="form-group">
        <label htmlFor="email">Email</label>
        <input
          type="email"
          id="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="student@example.com"
          required
        />
      </div>

      {/* Password Input */}
      <div className="form-group">
        <label htmlFor="password">Password (min 8 characters)</label>
        <div className="password-input">
          <input
            type={showPassword ? 'text' : 'password'}
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="••••••••"
            minLength={8}
            required
          />
          <button
            type="button"
            className="toggle-password"
            onClick={() => setShowPassword(!showPassword)}
          >
            {showPassword ? '👁️' : '👁️‍🗨️'}
          </button>
        </div>
      </div>

      {/* Profile Questions */}
      <div className="profile-questions">
        <h3>Tell us about your setup (4 quick questions)</h3>

        {/* Question 1: RTX GPU */}
        <div className="form-group">
          <label>Do you have an RTX GPU?</label>
          <div className="radio-group">
            <label>
              <input
                type="radio"
                name="has_rtx_gpu"
                value="yes"
                checked={profile.has_rtx_gpu === true}
                onChange={() => setProfile({...profile, has_rtx_gpu: true})}
              />
              Yes
            </label>
            <label>
              <input
                type="radio"
                name="has_rtx_gpu"
                value="no"
                checked={profile.has_rtx_gpu === false}
                onChange={() => setProfile({...profile, has_rtx_gpu: false})}
              />
              No
            </label>
          </div>
        </div>

        {/* Question 2: Jetson Orin Nano */}
        <div className="form-group">
          <label>Do you own a Jetson Orin Nano?</label>
          <div className="radio-group">
            <label>
              <input
                type="radio"
                name="has_jetson"
                value="yes"
                checked={profile.has_jetson === true}
                onChange={() => setProfile({...profile, has_jetson: true})}
              />
              Yes
            </label>
            <label>
              <input
                type="radio"
                name="has_jetson"
                value="no"
                checked={profile.has_jetson === false}
                onChange={() => setProfile({...profile, has_jetson: false})}
              />
              No
            </label>
          </div>
        </div>

        {/* Question 3: Real Robot */}
        <div className="form-group">
          <label>Do you have access to a real robot?</label>
          <div className="radio-group">
            <label>
              <input
                type="radio"
                name="has_real_robot"
                value="yes"
                checked={profile.has_real_robot === true}
                onChange={() => setProfile({...profile, has_real_robot: true})}
              />
              Yes
            </label>
            <label>
              <input
                type="radio"
                name="has_real_robot"
                value="no"
                checked={profile.has_real_robot === false}
                onChange={() => setProfile({...profile, has_real_robot: false})}
              />
              No
            </label>
          </div>
        </div>

        {/* Question 4: Programming Experience */}
        <div className="form-group">
          <label htmlFor="programming_experience">Your programming experience?</label>
          <select
            id="programming_experience"
            value={profile.programming_experience}
            onChange={(e) => setProfile({...profile, programming_experience: e.target.value})}
            required
          >
            <option value="">Select your level</option>
            <option value="beginner">Beginner (learning Python basics)</option>
            <option value="intermediate">Intermediate (comfortable with Python, classes)</option>
            <option value="advanced">Advanced (know async, decorators, type hints)</option>
          </select>
        </div>
      </div>

      {/* Error Message */}
      {error && <div className="error-message">{error}</div>}

      {/* Submit Button */}
      <button type="submit" className="submit-button" disabled={loading}>
        {loading ? 'Creating account...' : 'Create Account'}
      </button>

      {/* Signin Link */}
      <p className="footer-link">
        Already have an account? <a href="/signin">Sign in</a>
      </p>
    </form>
  );
}
```

**Embedding in Docusaurus**:
- Create dedicated page: `frontend/src/pages/signup.tsx`
- Import component: `import SignupForm from '../components/Auth/SignupForm';`
- Render: `<SignupForm onSuccess={() => console.log('Signup successful')} />`

---

#### FR-007: React Signin Component
System MUST provide a simpler signin form:

**Location**: `frontend/src/components/Auth/SigninForm.tsx`

**Component** (simplified, no profile questions):
```typescript
export default function SigninForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password })
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.error || 'Invalid email or password');
        return;
      }

      // Store token
      localStorage.setItem('auth_token', data.session.token);

      // Redirect
      window.location.href = '/docs/intro';
    } catch (err) {
      setError('Network error');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="signin-form">
      <h2>Welcome Back</h2>

      <div className="form-group">
        <label htmlFor="email">Email</label>
        <input
          type="email"
          id="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
      </div>

      <div className="form-group">
        <label htmlFor="password">Password</label>
        <input
          type="password"
          id="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
        />
      </div>

      {error && <div className="error-message">{error}</div>}

      <button type="submit" disabled={loading}>
        {loading ? 'Signing in...' : 'Sign In'}
      </button>

      <p className="footer-link">
        Don't have an account? <a href="/signup">Sign up</a>
      </p>
    </form>
  );
}
```

---

#### FR-008: Navbar Integration
System MUST add "Sign Up" and "Sign In" buttons to Docusaurus navbar:

**File**: `frontend/docusaurus.config.js`

```javascript
module.exports = {
  themeConfig: {
    navbar: {
      title: 'Physical AI Textbook',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'navbar-signup-button',
        },
        {
          to: '/signin',
          label: 'Sign In',
          position: 'right',
        },
      ],
    },
  },
};
```

**Dynamic Navbar** (show "Profile" if logged in):
- Use custom React component to check localStorage for `auth_token`
- If token exists: Show "Profile" dropdown (with "Sign Out" option)
- If no token: Show "Sign Up" and "Sign In" buttons

---

### Key Entities

- **User** (from Better-Auth, extended with profile):
  - Attributes: `id` (UUID), `email` (string, unique), `password_hash` (string), `email_verified` (boolean), `profile` (JSONB), `created_at` (timestamp), `updated_at` (timestamp)
  - Relationships: One-to-many with Sessions

- **Session** (from Better-Auth):
  - Attributes: `id` (UUID), `user_id` (FK to users), `token` (JWT string), `expires_at` (timestamp), `created_at` (timestamp)
  - Relationships: Many-to-one with User

- **Profile** (JSONB column in User):
  - Attributes: `has_rtx_gpu` (boolean), `has_jetson` (boolean), `has_real_robot` (boolean), `programming_experience` (enum: "beginner" | "intermediate" | "advanced"), `created_at` (timestamp)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Signup form accepts valid email + password + 4 profile questions → Account created in Neon DB (tested with 10 signups)
- **SC-002**: Profile stored in `users.profile` JSONB column with correct schema (verified via database query)
- **SC-003**: Signin form authenticates existing users → Session created → Token stored in localStorage
- **SC-004**: Anonymous users can read entire textbook without login (no paywall, no blur, tested on 20 random pages)
- **SC-005**: Navbar shows "Sign Up" / "Sign In" for anonymous users, "Profile" / "Sign Out" for logged-in users
- **SC-006**: After signup, user redirected to `/docs/intro?personalized=true` with success message
- **SC-007**: Personalized content appears for logged-in users based on profile (tested with 3 different profiles)
- **SC-008**: Database reuses existing Neon Postgres (same `NEON_DATABASE_URL`, no new database created)
- **SC-009**: Better-Auth tables auto-created on first backend startup (verified via database inspection)
- **SC-010**: Signup form validation prevents: invalid email, short password (<8 chars), duplicate email (tested with 5 error cases)

---

## Technical Architecture *(optional but recommended)*

### Technology Stack

**Frontend**:
- **Docusaurus v3**: Static site framework
- **React 18**: UI components (SignupForm, SigninForm)
- **TypeScript**: Type safety for forms and API calls
- **CSS Modules**: Scoped styling for auth forms

**Backend**:
- **FastAPI**: Python web framework (existing from RAG chatbot)
- **Better-Auth**: Authentication library (Node.js/TypeScript, runs as separate service OR integrated via Python wrapper)
- **Neon Postgres**: Serverless database (reused from 002-rag-chatbot)
- **Pydantic**: Request/response validation
- **bcrypt**: Password hashing (if not handled by Better-Auth)

**Database**:
- **Neon Postgres** (reused): `NEON_DATABASE_URL` from `.env`
- **Tables**: `users`, `sessions`, `verification_tokens` (Better-Auth auto-creates)
- **Profile Column**: `users.profile` JSONB (manual migration)

**Deployment**:
- **Frontend**: GitHub Pages (static, no auth logic client-side only)
- **Backend**: Render.com Free Tier (existing FastAPI deployment)
- **Database**: Neon Postgres Free Tier (0.5GB, existing)

---

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       Frontend (Docusaurus + React)             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ SignupForm   │  │ SigninForm   │  │ Navbar       │         │
│  │ (TypeScript) │  │ (TypeScript) │  │ (React)      │         │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│         │                  │                  │                  │
│         └──────────────────┴──────────────────┘                  │
│                            │                                     │
│                            ▼ HTTP Requests                       │
│                  /api/auth/signup                                │
│                  /api/auth/signin                                │
│                  /api/auth/session                               │
└─────────────────────────────────────┬───────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────┐
│               Backend (FastAPI + Better-Auth Service)           │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  FastAPI Routes                                           │  │
│  │  - POST /api/auth/signup → create_user_with_profile()    │  │
│  │  - POST /api/auth/signin → authenticate_user()           │  │
│  │  - GET /api/auth/session → validate_session()            │  │
│  └──────────────────────┬───────────────────────────────────┘  │
│                         │                                        │
│                         ▼ SQL Queries                            │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Better-Auth Service (Node.js or Python wrapper)         │  │
│  │  - Password hashing (bcrypt)                             │  │
│  │  - Session management (JWT)                              │  │
│  │  - Email validation                                      │  │
│  └──────────────────────┬───────────────────────────────────┘  │
└─────────────────────────┼───────────────────────────────────────┘
                          │
                          ▼ PostgreSQL Connection
┌─────────────────────────────────────────────────────────────────┐
│              Neon Postgres (Reused from 002-rag-chatbot)        │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Tables:                                                  │  │
│  │  - users (id, email, password_hash, profile JSONB, ...)  │  │
│  │  - sessions (id, user_id, token, expires_at, ...)        │  │
│  │  - verification_tokens (id, user_id, token, ...)         │  │
│  │  - chat_sessions (from 002-rag-chatbot, existing)        │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
│  Environment Variable (reused):                                 │
│  NEON_DATABASE_URL=postgresql://user:password@host/dbname      │
└─────────────────────────────────────────────────────────────────┘
```

---

## Non-Functional Requirements

### Performance
- **NFR-001**: Signup form submission MUST complete within 2 seconds (p95 latency)
- **NFR-002**: Signin form submission MUST complete within 1 second (p95 latency)
- **NFR-003**: Session validation MUST complete within 200ms (for authenticated page loads)

### Security
- **NFR-004**: Passwords MUST be hashed with bcrypt (min cost factor 12)
- **NFR-005**: JWT tokens MUST expire after 7 days (configurable)
- **NFR-006**: API endpoints MUST validate CORS (only allow frontend origin)
- **NFR-007**: Signup MUST prevent email enumeration attacks (same error for duplicate email and wrong password)
- **NFR-008**: Session tokens MUST be stored in `HttpOnly` cookies (if using cookies) OR `localStorage` with XSS protection

### Reliability
- **NFR-009**: Database connection failure MUST show graceful error (not crash frontend)
- **NFR-010**: Better-Auth service downtime MUST not block textbook reading (anonymous access continues)

### Maintainability
- **NFR-011**: Profile schema MUST be versioned (add `profile_version: 1` field for future migrations)
- **NFR-012**: Auth forms MUST be unit tested (React Testing Library)

---

## Implementation Plan *(high-level)*

### Phase 1: Backend Auth Setup (Priority: P1)
1. Install Better-Auth library in backend
2. Configure Better-Auth with existing `NEON_DATABASE_URL`
3. Run Better-Auth initialization (auto-creates tables)
4. Add profile migration: `ALTER TABLE users ADD COLUMN profile JSONB`
5. Implement custom `/api/auth/signup` endpoint (with profile storage)
6. Implement `/api/auth/signin` endpoint (wrapper around Better-Auth)
7. Implement `/api/auth/session` endpoint (session validation)

### Phase 2: Frontend Forms (Priority: P1)
8. Create `SignupForm.tsx` component (email + password + 4 questions)
9. Create `SigninForm.tsx` component (email + password)
10. Create dedicated pages: `/signup.tsx`, `/signin.tsx`
11. Style forms with CSS Modules (match Docusaurus theme)
12. Add form validation (client-side: email format, password length)

### Phase 3: Navbar Integration (Priority: P2)
13. Add "Sign Up" / "Sign In" buttons to Docusaurus navbar
14. Create dynamic navbar component (shows "Profile" if logged in)
15. Implement "Sign Out" functionality (clear localStorage, redirect to homepage)

### Phase 4: Personalization (Priority: P3)
16. Create personalization logic (read profile from database)
17. Add "Recommended for you" badges to chapters (based on profile)
18. Add hardware-specific notes (e.g., "No Jetson? Use cloud training")

---

## Risks & Mitigations

### Risk 1: Better-Auth TypeScript Compatibility with Python Backend
**Likelihood**: Medium
**Impact**: High (blocking implementation)

**Mitigation**:
- **Option A**: Run Better-Auth as separate Node.js microservice (FastAPI calls it via HTTP)
- **Option B**: Use Python-native library (e.g., `python-jose` for JWT, `passlib` for bcrypt) instead of Better-Auth
- **Recommendation**: Use Option B if Better-Auth integration is complex. Better-Auth is optional; the spec's core requirement is "email + password auth with profile storage."

---

### Risk 2: Neon DB Free Tier Limits (0.5GB)
**Likelihood**: Low (users table is small)
**Impact**: Medium (would require paid tier upgrade)

**Mitigation**:
- Monitor database size monthly
- Users table with 10,000 accounts ≈ 2MB (negligible)
- Profile JSONB adds ~200 bytes per user
- Total for 10,000 users: ~4MB (well within 0.5GB limit)

---

### Risk 3: Profile Questions Change Over Time
**Likelihood**: High (curriculum evolves, new hardware released)
**Impact**: Medium (old profiles incompatible with new logic)

**Mitigation**:
- Add `profile_version: 1` field to profile JSONB
- Future changes increment version (e.g., `profile_version: 2`)
- Backend handles multiple versions gracefully (fallback to defaults)

---

## Open Questions

1. **Should we use Better-Auth or Python-native auth?**
   - **Recommendation**: Python-native (`python-jose` + `passlib`) is simpler for FastAPI backend. Better-Auth adds Node.js dependency.

2. **Should profile be editable after signup?**
   - **MVP**: No (immutable profile). **Future**: Add "Update Profile" page.

3. **Should we implement "Forgot Password" flow?**
   - **MVP**: No (manual support). **Future**: Email-based password reset.

4. **Should we use cookies or localStorage for session tokens?**
   - **Recommendation**: `HttpOnly` cookies (more secure). Fallback: localStorage if cookies blocked.

---

## Acceptance Checklist

Before marking this feature as complete, verify:

- [ ] Backend: Better-Auth (or Python-native auth) integrated with Neon DB
- [ ] Database: `users.profile` JSONB column exists, migrations run successfully
- [ ] Signup form: Accepts email + password + 4 questions, creates account
- [ ] Signin form: Authenticates users, creates session, stores token
- [ ] Navbar: Shows "Sign Up" / "Sign In" (anonymous) or "Profile" / "Sign Out" (logged in)
- [ ] Anonymous access: All textbook pages readable without login
- [ ] Personalization: Logged-in users see profile-based content (basic implementation)
- [ ] Security: Passwords hashed, JWT tokens validated, CORS configured
- [ ] Testing: 10 signups tested, 10 signins tested, 5 error cases tested
- [ ] Documentation: README updated with auth setup instructions

---

## Constitution Alignment Checklist

This feature directly supports the following constitution principles:

- [x] **Principle VIII: Authentication & User Profiles (Better-Auth)** - Core feature implementing this principle
- [x] **Principle VI: Urdu + Personalization Ready** - Profile enables personalized chapter views
- [x] **Principle X: Open Source & Accessible Forever** - Login optional, no paywall
- [x] **Principle I: Single Source of Truth** - Reuses existing Neon DB (no duplicate databases)

---

## Related Documents

- Constitution: `.specify/memory/constitution.md` (Principle VIII)
- Existing Feature: `specs/002-rag-chatbot/spec.md` (Neon DB setup)
- Better-Auth Docs: https://www.better-auth.com/docs
- Neon Postgres Docs: https://neon.tech/docs

---

**Specification Version**: 1.0
**Last Updated**: 2025-12-06
**Author**: Claude Code (Opus 4.5)
**Status**: Ready for Review