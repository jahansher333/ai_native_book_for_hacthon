# Quickstart Guide: Urdu Translation Feature

**Branch**: `006-urdu-translation` | **Date**: 2025-12-07
**For**: Developers implementing or testing the Urdu translation feature

## Overview

This guide walks you through setting up, developing, and testing the per-chapter Urdu translation button feature.

**What you'll build**:
- "اردو میں پڑھیں" button on every chapter
- Backend translation API using @urdu-translator subagent
- Frontend React components with RTL text support
- localStorage caching with 7-day TTL
- Bilingual error handling

**Time estimate**: 6-8 hours for full implementation

---

## Prerequisites

### Required

- **Node.js**: v18+ (for frontend)
- **Python**: 3.11+ (for backend)
- **Gemini API Key**: Valid Google Gemini API key with quota
- **Git**: Installed and configured
- **Text Editor**: VS Code recommended (RTL text preview)

### Recommended

- **Browser Extensions**: React DevTools, Redux DevTools
- **API Client**: Postman or curl for testing
- **Urdu Font**: Noto Nastaliq Urdu installed locally for development

---

## Project Structure

```
ai_robotics_book/
├── backend/
│   ├── src/
│   │   ├── agents/
│   │   │   ├── urdu_translator_agent.py         # NEW - Agent orchestrator
│   │   │   └── skills/
│   │   │       └── translate_to_urdu_skill.py   # NEW - Translation skill
│   │   └── api/
│   │       └── translate.py                     # NEW - FastAPI endpoint
│   ├── .env                                     # MODIFIED - Add Gemini API key
│   └── requirements.txt                         # MODIFIED - Add dependencies
│
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   └── Urdu/                            # NEW - Urdu components
│   │   │       ├── UrduButton.tsx
│   │   │       ├── UrduChapterWrapper.tsx
│   │   │       └── UrduChapterWrapper.module.css
│   │   ├── hooks/
│   │   │   └── useUrduTranslation.ts            # NEW - Translation hook
│   │   └── services/
│   │       └── urduTranslationService.ts        # NEW - API client
│   └── package.json                             # MODIFIED - Add @types/dompurify
│
└── specs/006-urdu-translation/
    ├── spec.md                                  # Feature specification
    ├── plan.md                                  # Implementation plan
    ├── research.md                              # Technical decisions
    ├── data-model.md                            # Entity definitions
    ├── quickstart.md                            # This file
    └── contracts/
        └── translate-api.yaml                   # OpenAPI spec
```

---

## Setup Steps

### Step 1: Checkout Feature Branch

```bash
cd ai_robotics_book
git checkout 006-urdu-translation
git pull origin 006-urdu-translation
```

**Verify**:
```bash
git branch
# Should show: * 006-urdu-translation
```

---

### Step 2: Configure Backend

#### 2.1 Install Python Dependencies

```bash
cd backend
pip install openai-agents  # If not already installed
pip install -r requirements.txt
```

**Note**: `openai-agents` SDK is already in requirements.txt from feature 005 (personalization).

#### 2.2 Configure Gemini API Key

Edit `backend/.env`:

```bash
# Gemini API (via OpenAI-compatible endpoint)
GEMINI_API_KEY=your_actual_gemini_api_key_here
BASE_URL="https://generativelanguage.googleapis.com/v1beta/openai/"
MODEL_NAME="gemini-2.0-flash"
```

**Get API Key**:
1. Visit https://aistudio.google.com/app/apikey
2. Create new project or use existing
3. Generate API key
4. Copy and paste into `.env`

**Verify Configuration**:
```bash
cd backend
python -c "from src.config import settings; print(f'API Key: {settings.gemini_api_key[:10]}...'); print(f'Model: {settings.model_name}')"
```

Expected output:
```
API Key: AIzaSyB...
Model: gemini-2.0-flash
```

---

### Step 3: Start Backend Server

```bash
cd backend
python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify Backend**:
```bash
curl http://localhost:8000/api/translate/health
```

Expected response:
```json
{
  "status": "healthy",
  "gemini_api_available": true,
  "timestamp": 1733587200000
}
```

---

### Step 4: Configure Frontend

#### 4.1 Install Dependencies

```bash
cd frontend
npm install
```

**Note**: DOMPurify is already installed from feature 005 (personalization).

#### 4.2 Verify TypeScript Configuration

Ensure `frontend/tsconfig.json` includes:
```json
{
  "compilerOptions": {
    "lib": ["dom", "dom.iterable", "esnext"],
    "jsx": "react-jsx",
    "esModuleInterop": true
  }
}
```

---

### Step 5: Start Frontend Development Server

```bash
cd frontend
npm start
```

**Verify Frontend**:
- Open browser to `http://localhost:3000`
- Navigate to any chapter (e.g., `/docs/intro`)
- You should see existing "Personalize Chapter" button
- After implementation, "اردو میں پڑھیں" button will appear

---

## Development Workflow

### Phase 1: Backend Implementation

#### Task 1: Create Translation Skill

**File**: `backend/src/agents/skills/translate_to_urdu_skill.py`

```python
from openai_agents import Skill
from openai import AsyncOpenAI
import re
from typing import Tuple

class TranslateToUrduSkill(Skill):
    def __init__(self, client: AsyncOpenAI):
        super().__init__(
            name="translateToUrduSkill",
            description="Translates English technical content to natural Urdu"
        )
        self.client = client

    async def execute(self, chapter_content: str, chapter_id: str) -> str:
        """
        Translate chapter content to Urdu.

        Args:
            chapter_content: English markdown content
            chapter_id: Chapter identifier for logging

        Returns:
            Urdu translated markdown content

        Raises:
            ValueError: If translation fails validation
        """
        # Build prompt with technical glossary
        prompt = self._build_translation_prompt(chapter_content)

        # Call Gemini API
        response = await self.client.chat.completions.create(
            model="gemini-2.0-flash",
            messages=[
                {"role": "system", "content": self._get_system_prompt()},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=8192
        )

        translated = response.choices[0].message.content

        # Validate markdown structure
        is_valid, error_msg = self._validate_markdown_structure(
            chapter_content,
            translated
        )

        if not is_valid:
            raise ValueError(f"Translation validation failed: {error_msg}")

        return translated

    def _get_system_prompt(self) -> str:
        return """You are an expert technical translator specializing in robotics and AI content.
Your task is to translate English technical documentation into natural, readable Urdu.

**Guidelines**:
1. Preserve ALL markdown structure (headings, lists, code blocks, links)
2. Follow technical term translation rules
3. Keep code blocks unchanged
4. Maintain natural Urdu sentence flow
5. Use proper Urdu technical terminology
"""

    def _build_translation_prompt(self, content: str) -> str:
        glossary = self._get_technical_glossary()
        return f"""Translate the following English technical content to Urdu.

**Technical Term Translation Rules**:
{glossary}

**Important**:
- Preserve exact markdown structure
- Keep code blocks in English
- Maintain all links and formatting
- Use natural Urdu prose

**Content to translate**:

{content}
"""

    def _get_technical_glossary(self) -> str:
        return """
**Tier 1 - Full Translation**:
- ROS 2 → روبوٹک آپریٹنگ سسٹم 2
- URDF → یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ
- Publisher → پبلشر (ناشر)
- Subscriber → سبسکرائبر (وصول کنندہ)
- Node → نوڈ (اکائی)

**Tier 2 - Transliteration**:
- Jetson → جیٹسن
- NVIDIA → اینویڈیا
- Isaac Sim → آئزک سم
- Unitree → یونٹری

**Tier 3 - Hybrid**:
- Latency trap → لیٹنسی ٹریپ (خطرناک تاخیر)
- Sim-to-real → سم ٹو ریئل (نقلی سے حقیقی منتقلی)
"""

    def _validate_markdown_structure(
        self,
        original: str,
        translated: str
    ) -> Tuple[bool, str]:
        """Validate markdown structure preservation."""
        # Check heading count
        original_headings = len(re.findall(r'^#{1,6}\s', original, re.MULTILINE))
        translated_headings = len(re.findall(r'^#{1,6}\s', translated, re.MULTILINE))
        if original_headings != translated_headings:
            return False, f"Heading mismatch: {original_headings} vs {translated_headings}"

        # Check code block count
        original_code = original.count('```')
        translated_code = translated.count('```')
        if original_code != translated_code:
            return False, f"Code block mismatch: {original_code} vs {translated_code}"

        return True, ""
```

#### Task 2: Create Agent Orchestrator

**File**: `backend/src/agents/urdu_translator_agent.py`

```python
from openai_agents import Agent, Runner
from openai import AsyncOpenAI
import asyncio
from .skills.translate_to_urdu_skill import TranslateToUrduSkill
from ..config import settings

# Initialize OpenAI client with Gemini endpoint
client = AsyncOpenAI(
    api_key=settings.gemini_api_key,
    base_url=settings.base_url
)

# Create skill
translate_skill = TranslateToUrduSkill(client)

# Create agent
urdu_translator_agent = Agent(
    name="Urdu Translator",
    instructions="You translate technical robotics content from English to Urdu.",
    skills=[translate_skill],
    model="gemini-2.0-flash"
)

async def translate_chapter_content(
    original_content: str,
    chapter_id: str
) -> str:
    """
    Orchestrate chapter translation via agent.

    Args:
        original_content: English markdown content
        chapter_id: Chapter identifier

    Returns:
        Urdu translated content

    Raises:
        TimeoutError: If translation exceeds 60 seconds
        ValueError: If translation fails validation
    """
    runner = Runner(agent=urdu_translator_agent, client=client)

    try:
        result = await asyncio.wait_for(
            runner.run(
                chapter_content=original_content,
                chapter_id=chapter_id
            ),
            timeout=60.0  # 60-second timeout per FR-014
        )

        return result.translated_content

    except asyncio.TimeoutError:
        raise TimeoutError("Translation exceeded 60-second timeout")
```

#### Task 3: Create FastAPI Endpoint

**File**: `backend/src/api/translate.py`

```python
from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
import re
from ..agents.urdu_translator_agent import translate_chapter_content

router = APIRouter(prefix="/api/translate", tags=["translation"])

class TranslationRequest(BaseModel):
    chapterId: str = Field(..., pattern=r'^[a-z0-9-]+$', min_length=3, max_length=100)
    originalContent: str = Field(..., min_length=100, max_length=50000)

class TranslationResponse(BaseModel):
    chapterId: str
    translatedContent: str
    timestamp: int
    cached: bool = False

@router.post("/chapter", response_model=TranslationResponse)
async def translate_chapter(request: TranslationRequest):
    """
    Translate chapter to Urdu.

    Returns:
        TranslationResponse with Urdu content
    """
    try:
        translated = await translate_chapter_content(
            original_content=request.originalContent,
            chapter_id=request.chapterId
        )

        import time
        return TranslationResponse(
            chapterId=request.chapterId,
            translatedContent=translated,
            timestamp=int(time.time() * 1000),
            cached=False
        )

    except TimeoutError:
        raise HTTPException(
            status_code=status.HTTP_504_GATEWAY_TIMEOUT,
            detail="Translation took too long. Try a shorter chapter."
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Translation validation failed: {str(e)}"
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Translation failed"
        )

@router.get("/health")
async def health_check():
    """Health check endpoint."""
    from ..config import settings
    import time

    return {
        "status": "healthy",
        "gemini_api_available": bool(settings.gemini_api_key),
        "timestamp": int(time.time() * 1000)
    }
```

#### Task 4: Register Endpoint in Main App

**File**: `backend/src/main.py`

Add import and router registration:

```python
from .api import translate  # ADD THIS

app.include_router(translate.router)  # ADD THIS
```

---

### Phase 2: Frontend Implementation

#### Task 5: Create Translation Service

**File**: `frontend/src/services/urduTranslationService.ts`

```typescript
interface TranslationRequest {
  chapterId: string;
  originalContent: string;
}

interface TranslationResponse {
  chapterId: string;
  translatedContent: string;
  timestamp: number;
  cached: boolean;
}

export async function translateChapter(
  request: TranslationRequest
): Promise<TranslationResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 60000); // 60s timeout

  try {
    const response = await fetch('http://localhost:8000/api/translate/chapter', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      if (response.status === 429) {
        throw new Error('QUOTA_EXCEEDED');
      }
      if (response.status === 504) {
        throw new Error('TIMEOUT');
      }
      throw new Error('API_ERROR');
    }

    return await response.json();

  } catch (error: any) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new Error('TIMEOUT');
    }
    if (error.message.includes('NetworkError')) {
      throw new Error('NETWORK_ERROR');
    }

    throw error;
  }
}
```

#### Task 6: Create Translation Hook

**File**: `frontend/src/hooks/useUrduTranslation.ts`

```typescript
import { useState, useEffect } from 'react';
import { translateChapter } from '../services/urduTranslationService';

interface TranslationCacheEntry {
  translatedContent: string;
  timestamp: number;
  version: number;
}

export function useUrduTranslation(chapterId: string, originalContent: string) {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [isFromCache, setIsFromCache] = useState(false);

  const CACHE_VERSION = 1;
  const TTL = 7 * 24 * 60 * 60 * 1000; // 7 days

  // Check cache on mount
  useEffect(() => {
    const cached = getCachedTranslation(chapterId);
    if (cached) {
      setTranslatedContent(cached);
      setIsFromCache(true);
    }
  }, [chapterId]);

  const translateContent = async () => {
    setIsTranslating(true);
    setError(null);

    try {
      const response = await translateChapter({
        chapterId,
        originalContent,
      });

      setTranslatedContent(response.translatedContent);
      setIsFromCache(false);

      // Cache the translation
      cacheTranslation(chapterId, response.translatedContent);

      // Scroll to top
      window.scrollTo({ top: 0, behavior: 'smooth' });

    } catch (err: any) {
      setError(err.message || 'API_ERROR');
    } finally {
      setIsTranslating(false);
    }
  };

  const resetToDefault = () => {
    setTranslatedContent(null);
    setError(null);
    setIsFromCache(false);
  };

  const getCachedTranslation = (chapterId: string): string | null => {
    const cacheKey = `urdu_translation_${chapterId}_v${CACHE_VERSION}`;
    const cached = localStorage.getItem(cacheKey);
    if (!cached) return null;

    try {
      const entry: TranslationCacheEntry = JSON.parse(cached);
      const age = Date.now() - entry.timestamp;

      if (age > TTL || entry.version !== CACHE_VERSION) {
        localStorage.removeItem(cacheKey);
        return null;
      }

      return entry.translatedContent;
    } catch {
      return null;
    }
  };

  const cacheTranslation = (chapterId: string, content: string) => {
    const cacheKey = `urdu_translation_${chapterId}_v${CACHE_VERSION}`;
    const entry: TranslationCacheEntry = {
      translatedContent: content,
      timestamp: Date.now(),
      version: CACHE_VERSION,
    };
    localStorage.setItem(cacheKey, JSON.stringify(entry));
  };

  return {
    isTranslating,
    translatedContent,
    error,
    isFromCache,
    translateContent,
    resetToDefault,
  };
}
```

#### Task 7: Create Urdu Button Component

**File**: `frontend/src/components/Urdu/UrduButton.tsx`

```typescript
import React from 'react';
import styles from './UrduChapterWrapper.module.css';

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

interface UrduButtonProps {
  onTranslate: () => void;
  isLoading: boolean;
  error: string | null;
  isTranslated: boolean;
  isFromCache: boolean;
}

export default function UrduButton({
  onTranslate,
  isLoading,
  error,
  isTranslated,
  isFromCache
}: UrduButtonProps) {
  if (error) {
    const errorMsg = ERROR_MESSAGES[error as keyof typeof ERROR_MESSAGES] || ERROR_MESSAGES.API_ERROR;
    return (
      <div className={styles.errorContainer}>
        <p className={styles.errorEn}>{errorMsg.en}</p>
        <p className={styles.errorUr}>{errorMsg.ur}</p>
        <button onClick={onTranslate} className={styles.retryButton}>
          Retry / دوبارہ کوشش کریں
        </button>
      </div>
    );
  }

  return (
    <div className={styles.buttonContainer}>
      <button
        onClick={onTranslate}
        disabled={isLoading}
        className={styles.urduButton}
      >
        {isLoading ? 'Translating... / ترجمہ ہو رہا ہے' : 'اردو میں پڑھیں'}
      </button>

      {isTranslated && (
        <span className={styles.badge}>
          {isFromCache ? '✓ Cached / محفوظ' : '✓ اردو میں'}
        </span>
      )}
    </div>
  );
}
```

#### Task 8: Create Wrapper Component

**File**: `frontend/src/components/Urdu/UrduChapterWrapper.tsx`

```typescript
import React, { useEffect, useRef, useState } from 'react';
import DOMPurify from 'dompurify';
import { useUrduTranslation } from '../../hooks/useUrduTranslation';
import UrduButton from './UrduButton';
import styles from './UrduChapterWrapper.module.css';

interface UrduChapterWrapperProps {
  chapterId: string;
  children: React.ReactNode;
}

export default function UrduChapterWrapper({
  chapterId,
  children
}: UrduChapterWrapperProps) {
  const contentRef = useRef<HTMLDivElement>(null);
  const [originalContent, setOriginalContent] = useState<string>('');

  // Extract text content from DOM
  useEffect(() => {
    if (contentRef.current) {
      const content = contentRef.current.innerText || '';
      setOriginalContent(content);
    }
  }, [children]);

  const {
    isTranslating,
    translatedContent,
    error,
    isFromCache,
    translateContent,
    resetToDefault
  } = useUrduTranslation(chapterId, originalContent);

  // Render Urdu content if available
  if (translatedContent) {
    const sanitized = DOMPurify.sanitize(translatedContent, {
      ALLOWED_TAGS: ['p', 'br', 'strong', 'em', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
                     'ul', 'ol', 'li', 'a', 'code', 'pre', 'blockquote'],
      ALLOWED_ATTR: ['href', 'class', 'id']
    });

    return (
      <div className={styles.wrapper}>
        <div className={styles.controls}>
          <UrduButton
            onTranslate={translateContent}
            isLoading={isTranslating}
            error={error}
            isTranslated={true}
            isFromCache={isFromCache}
          />
          <button onClick={resetToDefault} className={styles.restoreButton}>
            English میں واپس
          </button>
        </div>

        <div
          className={`${styles.urduContent} markdown`}
          dangerouslySetInnerHTML={{ __html: sanitized }}
        />
      </div>
    );
  }

  // Render original content with translate button
  return (
    <div className={styles.wrapper}>
      <UrduButton
        onTranslate={translateContent}
        isLoading={isTranslating}
        error={error}
        isTranslated={false}
        isFromCache={false}
      />

      <div className={styles.content} ref={contentRef}>
        {children}
      </div>
    </div>
  );
}
```

#### Task 9: Create Styles

**File**: `frontend/src/components/Urdu/UrduChapterWrapper.module.css`

```css
.wrapper {
  position: relative;
}

.buttonContainer {
  display: flex;
  align-items: center;
  gap: 1rem;
  margin-bottom: 1.5rem;
}

.urduButton {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border: none;
  padding: 0.75rem 1.5rem;
  border-radius: 8px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
}

.urduButton:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
}

.urduButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.badge {
  background: #10b981;
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 20px;
  font-size: 0.875rem;
  font-weight: 500;
}

.restoreButton {
  background: #6b7280;
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 6px;
  cursor: pointer;
}

.restoreButton:hover {
  background: #4b5563;
}

.urduContent {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Arial', sans-serif;
  line-height: 2;
}

/* Keep code blocks LTR */
.urduContent pre,
.urduContent code {
  direction: ltr;
  text-align: left;
  font-family: 'Courier New', monospace;
}

.errorContainer {
  background: #fee2e2;
  border: 2px solid #ef4444;
  border-radius: 8px;
  padding: 1rem;
  margin-bottom: 1.5rem;
}

.errorEn, .errorUr {
  margin: 0.5rem 0;
  color: #991b1b;
}

.errorUr {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Arial', sans-serif;
}

.retryButton {
  margin-top: 0.5rem;
  background: #ef4444;
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 6px;
  cursor: pointer;
}

.retryButton:hover {
  background: #dc2626;
}
```

---

## Testing

### Backend Tests

#### Test Translation Endpoint

```bash
# Test health check
curl http://localhost:8000/api/translate/health

# Test translation (short chapter)
curl -X POST http://localhost:8000/api/translate/chapter \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "test-chapter",
    "originalContent": "# Test Chapter\n\nThis is a test chapter about ROS 2. The Publisher sends messages to the Subscriber. Jetson Orin Nano is a powerful edge device."
  }'
```

Expected response:
```json
{
  "chapterId": "test-chapter",
  "translatedContent": "# ٹیسٹ باب\n\nیہ روبوٹک آپریٹنگ سسٹم 2 کے بارے میں ٹیسٹ باب ہے...",
  "timestamp": 1733587200000,
  "cached": false
}
```

#### Test Error Cases

```bash
# Test invalid chapterId (should return 422)
curl -X POST http://localhost:8000/api/translate/chapter \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "INVALID_ID",
    "originalContent": "test content that is long enough to pass validation rules for minimum length"
  }'

# Test content too short (should return 422)
curl -X POST http://localhost:8000/api/translate/chapter \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "test",
    "originalContent": "too short"
  }'
```

---

### Frontend Tests

#### Manual Testing Checklist

1. **Button Visibility**
   - [ ] Navigate to any chapter
   - [ ] "اردو میں پڑھیں" button appears at top

2. **Translation Flow**
   - [ ] Click button
   - [ ] Loading spinner appears
   - [ ] Within 10 seconds, content changes to Urdu
   - [ ] Badge shows "✓ اردو میں"

3. **Cache Test**
   - [ ] Translate a chapter
   - [ ] Refresh page
   - [ ] Click button again
   - [ ] Content loads instantly
   - [ ] Badge shows "✓ Cached / محفوظ"

4. **Toggle Back**
   - [ ] Click "English میں واپس" button
   - [ ] Original English content restored
   - [ ] Button changes back to "اردو میں پڑھیں"

5. **Error Handling**
   - [ ] Stop backend server
   - [ ] Click translate button
   - [ ] Bilingual error message appears
   - [ ] Retry button visible

6. **RTL Layout**
   - [ ] Urdu text aligns right
   - [ ] Code blocks remain left-aligned
   - [ ] Technical terms render correctly

---

## Troubleshooting

### Issue: "QUOTA_EXCEEDED" Error

**Symptoms**: Error message shows "Translation service temporarily unavailable"

**Causes**:
- Gemini API free tier quota exhausted
- API key invalid

**Solutions**:
1. Wait 24 hours for quota reset
2. Verify API key: `echo $GEMINI_API_KEY`
3. Check quota: https://console.cloud.google.com/apis/dashboard
4. Upgrade to paid tier if needed

---

### Issue: Translation Takes > 60 Seconds

**Symptoms**: Timeout error after 60 seconds

**Causes**:
- Chapter too long (>10,000 words)
- Gemini API slow response

**Solutions**:
1. Test with shorter chapter
2. Check network latency
3. Consider chunking large chapters (future enhancement)

---

### Issue: Markdown Structure Broken

**Symptoms**: Missing headings, broken links in Urdu translation

**Causes**:
- Validation failed
- Gemini output malformed

**Solutions**:
1. Check backend logs for validation errors
2. Refine translation prompt
3. Add more examples to glossary

---

### Issue: Urdu Text Not Displaying

**Symptoms**: Square boxes or mojibake characters

**Causes**:
- Missing Urdu font
- Encoding issues

**Solutions**:
1. Add Noto Nastaliq Urdu to HTML head:
```html
<link href="https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap" rel="stylesheet">
```
2. Verify UTF-8 encoding in HTTP headers

---

## Next Steps

After completing implementation:

1. **Run `/sp.tasks`** to generate detailed task breakdown
2. **Implement tasks** following the order in `tasks.md`
3. **Test each user story** independently
4. **Commit work** with proper git messages
5. **Create PR** against main branch

---

**Questions?** Refer to:
- `spec.md` - Feature requirements
- `research.md` - Technical decisions
- `data-model.md` - Entity definitions
- `contracts/translate-api.yaml` - API documentation

**Last Updated**: 2025-12-07
