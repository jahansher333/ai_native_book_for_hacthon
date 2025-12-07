# Data Model: Chapter Personalization

**Feature**: 005-chapter-personalization
**Date**: 2025-12-06

## Entity Definitions

### User Profile

**Description**: User account information with hardware and experience profile

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| user_email | string | Yes | User's email (from JWT token) |
| experience | enum | Yes | Learning level: 'beginner', 'intermediate', 'advanced' |
| hasRTX | boolean | Yes | User has access to RTX GPU |
| hasJetson | boolean | Yes | User has access to Jetson device |
| hasRobot | boolean | Yes | User has access to real robot hardware |

**Source**: Existing Better-Auth user model (`backend/src/models/user.py`)

**Validation Rules**:
- `experience` must be one of: 'beginner', 'intermediate', 'advanced'
- Boolean fields default to `false` if not provided

**Example**:
```json
{
  "user_email": "student@example.com",
  "experience": "intermediate",
  "hasRTX": true,
  "hasJetson": false,
  "hasRobot": false
}
```

---

### Chapter

**Description**: A section of the Docusaurus textbook

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chapterId | string | Yes | Unique chapter identifier (e.g., "chapter-1-intro") |
| title | string | No | Chapter title (extracted from heading) |
| content | string | Yes | Rendered chapter content (HTML/markdown) |
| wordCount | number | No | Approximate word count (for timeout estimation) |

**Source**: Extracted from Docusaurus DOM

**Validation Rules**:
- `chapterId` must match pattern: `^[a-z0-9-]+$`
- `content` must not be empty
- `content` length: 100 - 50000 characters

**Example**:
```json
{
  "chapterId": "chapter-1-intro",
  "title": "Introduction to ROS 2",
  "content": "# Introduction to ROS 2\n\nROS 2 is...",
  "wordCount": 2500
}
```

---

### Personalization Request

**Description**: Request payload for chapter personalization

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chapterId | string | Yes | Chapter identifier |
| originalContent | string | Yes | Original chapter content to personalize |
| userProfile | UserProfile | Yes | User's profile information |

**Validation Rules**:
- All fields are required
- `originalContent` must contain at least one heading marker (`#`)
- `userProfile` must be valid UserProfile object

**Pydantic Model**:
```python
class PersonalizeChapterRequest(BaseModel):
    chapterId: str
    originalContent: str
    userProfile: UserProfile

    @validator('chapterId')
    def validate_chapter_id(cls, v):
        if not re.match(r'^[a-z0-9-]+$', v):
            raise ValueError('Invalid chapter ID format')
        return v

    @validator('originalContent')
    def validate_content(cls, v):
        if len(v) < 100:
            raise ValueError('Content too short')
        if len(v) > 50000:
            raise ValueError('Content too long (max 50k chars)')
        return v
```

**Example**:
```json
{
  "chapterId": "chapter-1-intro",
  "originalContent": "# Introduction to ROS 2\n\nROS 2 is a robot operating system...",
  "userProfile": {
    "experience": "beginner",
    "hasRTX": false,
    "hasJetson": true,
    "hasRobot": false
  }
}
```

---

### Personalization Response

**Description**: Response containing personalized chapter content

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| success | boolean | Yes | Whether personalization succeeded |
| personalizedContent | string | Yes (if success) | Rewritten chapter content |
| metadata | object | Yes | Generation metadata |
| metadata.generationTime | number | Yes | Time taken in seconds |
| metadata.cached | boolean | Yes | Whether result was cached |
| metadata.modelUsed | string | No | Model name (e.g., "gemini-2.0-flash-exp") |

**Pydantic Model**:
```python
class PersonalizationMetadata(BaseModel):
    generationTime: float
    cached: bool
    modelUsed: Optional[str] = "gemini-2.0-flash-exp"

class PersonalizeChapterResponse(BaseModel):
    success: bool
    personalizedContent: str
    metadata: PersonalizationMetadata
```

**Example** (Success):
```json
{
  "success": true,
  "personalizedContent": "# Introduction to ROS 2 (Personalized for You)\n\nSince you're new to robotics...",
  "metadata": {
    "generationTime": 12.5,
    "cached": false,
    "modelUsed": "gemini-2.0-flash-exp"
  }
}
```

**Example** (Error):
```json
{
  "success": false,
  "personalizedContent": "",
  "metadata": {
    "generationTime": 0,
    "cached": false,
    "error": "Timeout exceeded"
  }
}
```

---

### Personalization Cache Entry

**Description**: Client-side cache entry for personalized content

**Storage**: LocalStorage (client-side)

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| content | string | Yes | Personalized chapter content |
| timestamp | number | Yes | Unix timestamp of creation |
| profileHash | string | Yes | Hash of user profile (for cache key) |
| chapterId | string | Yes | Chapter identifier |

**Cache Key Format**:
```
personalized_{chapterId}_{userEmail}_{profileHash}
```

**TTL**: 7 days (604800000 milliseconds)

**Example**:
```json
{
  "content": "# Introduction to ROS 2...",
  "timestamp": 1733518800000,
  "profileHash": "a1b2c3d4",
  "chapterId": "chapter-1-intro"
}
```

**Cache Invalidation**:
- Age > 7 days: Remove from cache
- Profile changed: Different profileHash, new cache entry
- Manual clear: User clicks "Restore Original"

---

## Data Flow

```
┌─────────────────┐
│  Frontend       │
│  User Profile   │──┐
│  (Context)      │  │
└─────────────────┘  │
                     │
┌─────────────────┐  │
│  Chapter DOM    │  │
│  (Extract)      │  │
└─────────────────┘  │
                     │
                     ↓
          ┌──────────────────────┐
          │ Personalization      │
          │ Request              │
          │ (JSON Payload)       │
          └──────────┬───────────┘
                     │
                     │ HTTP POST
                     │ /api/personalize/chapter
                     ↓
          ┌──────────────────────┐
          │  Backend API         │
          │  - Verify JWT        │
          │  - Validate request  │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │  Personalizer Agent  │
          │  - Build prompt      │
          │  - Call Gemini       │
          │  - Validate output   │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │ Personalization      │
          │ Response             │
          │ (JSON Payload)       │
          └──────────┬───────────┘
                     │
                     ↓
          ┌──────────────────────┐
          │  Frontend            │
          │  - Cache result      │
          │  - Replace DOM       │
          │  - Show badge        │
          └──────────────────────┘
```

---

## State Transitions

### Personalization State Machine

```
[Initial: Not Personalized]
         │
         │ Click "Personalize"
         ↓
[Check Auth]
         │
         ├─→ Not Authenticated ──→ [Show Login Prompt]
         │
         ├─→ Authenticated
         │    │
         │    ├─→ Cache Hit ──→ [Display Cached Content]
         │    │
         │    └─→ Cache Miss
         │         │
         │         ↓
         │   [Loading State]
         │         │
         │         │ API Call
         │         ↓
         │   [Received Response]
         │         │
         │         ├─→ Success ──→ [Display Personalized]
         │         │
         │         └─→ Error ──→ [Show Error + Retry]
         │
         └─→ Click "Restore Original"
              │
              ↓
        [Not Personalized]
```

---

## Database Schema

**Note**: This feature does NOT require database tables for MVP (session-based only).

**Future Enhancement** (Post-MVP):

If persistent personalization is added:

```sql
CREATE TABLE personalized_chapters (
    id SERIAL PRIMARY KEY,
    user_email VARCHAR(255) NOT NULL,
    chapter_id VARCHAR(255) NOT NULL,
    profile_hash CHAR(8) NOT NULL,
    personalized_content TEXT NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),
    UNIQUE(user_email, chapter_id, profile_hash)
);

CREATE INDEX idx_personalized_user_chapter
ON personalized_chapters(user_email, chapter_id);
```

---

## Validation Summary

### Request Validation
- ✅ JWT token present and valid
- ✅ `chapterId` matches pattern
- ✅ `originalContent` length: 100-50000 chars
- ✅ `userProfile` has all required fields
- ✅ `experience` is valid enum value

### Response Validation
- ✅ `personalizedContent` not empty
- ✅ Content length within ±50% of original
- ✅ Markdown structure preserved (heading count similar)
- ✅ No obvious content loss (key terms present)

### Security Validation
- ✅ HTML sanitized before DOM insertion (DOMPurify)
- ✅ Rate limiting enforced (10 requests/minute/user)
- ✅ JWT signature verified
- ✅ HTTPS enforced in production
