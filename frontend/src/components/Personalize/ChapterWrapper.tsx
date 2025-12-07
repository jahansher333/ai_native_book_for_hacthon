/**
 * ChapterWrapper - Wraps chapter content with personalization functionality
 *
 * This component:
 * - Renders the PersonalizeButton at the top of chapters
 * - Manages personalization state via usePersonalization hook
 * - Replaces chapter DOM with personalized content when available
 * - Sanitizes HTML with DOMPurify to prevent XSS attacks
 */
import React, { useEffect, useRef, useState } from 'react';
import DOMPurify from 'dompurify';
import { usePersonalization } from '../../hooks/usePersonalization';
import PersonalizeButton from './PersonalizeButton';
import { useUserProfile } from '../../contexts/UserProfileContext';
import styles from './ChapterWrapper.module.css';

interface ChapterWrapperProps {
  chapterId: string;
  children: React.ReactNode;
}

export default function ChapterWrapper({
  chapterId,
  children
}: ChapterWrapperProps): React.JSX.Element {
  // Debug log
  console.log('[ChapterWrapper] Rendering for chapter:', chapterId);

  // Get user profile and auth status from context
  const { profile, isAuthenticated, user } = useUserProfile();
  const contentRef = useRef<HTMLDivElement>(null);
  const [originalContent, setOriginalContent] = useState<string>('');

  // Urdu translation state
  const [isUrdu, setIsUrdu] = useState(false);
  const [urduContent, setUrduContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);
  const [isFromUrduCache, setIsFromUrduCache] = useState(false);

  console.log('[ChapterWrapper] Auth status:', { isAuthenticated, hasProfile: !!profile, hasUser: !!user });

  // Urdu translation handler
  const handleUrduClick = async () => {
    // Check cache first
    const cacheKey = `urdu_translation_${chapterId}_v1`;
    const cached = localStorage.getItem(cacheKey);

    if (cached) {
      try {
        const entry = JSON.parse(cached);
        if (Date.now() - entry.timestamp < 7 * 24 * 60 * 60 * 1000) {
          setUrduContent(entry.translatedContent);
          setIsUrdu(true);
          setIsFromUrduCache(true);
          console.log('[ChapterWrapper] Loaded Urdu translation from cache');
          return;
        }
      } catch (e) {
        console.warn('[ChapterWrapper] Failed to parse cached translation', e);
      }
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      const response = await fetch('http://localhost:8000/api/translate/chapter', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          chapterId,
          originalContent: originalContent
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();
      setUrduContent(data.translatedContent);
      setIsUrdu(true);
      setIsFromUrduCache(false);

      // Cache translation (7-day TTL)
      localStorage.setItem(cacheKey, JSON.stringify({
        translatedContent: data.translatedContent,
        timestamp: Date.now(),
        version: 1
      }));

      console.log('[ChapterWrapper] Urdu translation successful');

    } catch (error) {
      console.error('[ChapterWrapper] Urdu translation failed:', error);
      setTranslationError(error instanceof Error ? error.message : 'Translation failed');
    } finally {
      setIsTranslating(false);
    }
  };

  // Toggle back to English
  const handleBackToEnglish = () => {
    setIsUrdu(false);
    setUrduContent(null);
    setTranslationError(null);
    setIsFromUrduCache(false);
  };

  // Extract original content from DOM after render
  useEffect(() => {
    if (contentRef.current) {
      // Get the text content from the rendered children
      const content = contentRef.current.innerText || contentRef.current.textContent || '';
      setOriginalContent(content);
    }
  }, [children]);

  // Create profile with user_email
  const profileWithEmail = profile && user ? {
    ...profile,
    user_email: user.email
  } : null;

  // Use personalization hook
  const {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  } = usePersonalization(
    chapterId,
    originalContent,
    profileWithEmail,
    isAuthenticated
  );

  // If we have Urdu content, render it
  if (isUrdu && urduContent) {
    const sanitizedUrduContent = DOMPurify.sanitize(urduContent, {
      ALLOWED_TAGS: [
        'p', 'br', 'strong', 'em', 'u', 's', 'sup', 'sub',
        'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
        'ul', 'ol', 'li',
        'a', 'code', 'pre',
        'blockquote', 'hr',
        'table', 'thead', 'tbody', 'tfoot', 'tr', 'th', 'td',
        'img', 'figure', 'figcaption',
        'div', 'span', 'section', 'article',
        'details', 'summary'
      ],
      ALLOWED_ATTR: [
        'href', 'src', 'alt', 'title', 'class', 'id',
        'target', 'rel', 'width', 'height',
        'data-*', 'aria-*', 'role'
      ],
      ALLOW_DATA_ATTR: true,
      ALLOW_ARIA_ATTR: true
    });

    return (
      <div className={styles.wrapper}>
        <div className={styles.controls}>
          <button
            onClick={handleBackToEnglish}
            className={styles.backToEnglishButton}
            aria-label="Switch back to English"
          >
            English میں واپس
          </button>
          {isFromUrduCache && (
            <span className={styles.cachedBadge}>
              Cached
            </span>
          )}
        </div>

        <div className={styles.urdoBadge}>
          اردو میں دکھایا جا رہا ہے 🇵🇰
        </div>

        <div
          className={`${styles.content} ${styles.urduContent} markdown`}
          dangerouslySetInnerHTML={{ __html: sanitizedUrduContent }}
        />
      </div>
    );
  }

  // If we have personalized content, render it instead of children
  if (personalizedContent) {
    const sanitizedContent = DOMPurify.sanitize(personalizedContent, {
      ALLOWED_TAGS: [
        'p', 'br', 'strong', 'em', 'u', 's', 'sup', 'sub',
        'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
        'ul', 'ol', 'li',
        'a', 'code', 'pre',
        'blockquote', 'hr',
        'table', 'thead', 'tbody', 'tfoot', 'tr', 'th', 'td',
        'img', 'figure', 'figcaption',
        'div', 'span', 'section', 'article',
        'details', 'summary'
      ],
      ALLOWED_ATTR: [
        'href', 'src', 'alt', 'title', 'class', 'id',
        'target', 'rel', 'width', 'height',
        'data-*', 'aria-*', 'role'
      ],
      ALLOW_DATA_ATTR: true,
      ALLOW_ARIA_ATTR: true
    });

    return (
      <div className={styles.wrapper}>
        <div className={styles.controls}>
          <PersonalizeButton
            onPersonalize={personalizeContent}
            isLoading={isPersonalizing}
            error={error}
            isPersonalized={true}
            isFromCache={isFromCache}
          />
          <button
            onClick={resetToDefault}
            className={styles.resetButton}
            aria-label="Restore original chapter content"
          >
            Restore Original
          </button>
        </div>

        <div
          className={`${styles.content} ${styles.personalized} markdown`}
          dangerouslySetInnerHTML={{ __html: sanitizedContent }}
        />
      </div>
    );
  }

  // Default: render original content with Urdu and personalize buttons
  return (
    <div className={styles.wrapper}>
      <div className={styles.controls}>
        <button
          onClick={handleUrduClick}
          disabled={isTranslating}
          className={styles.urduButton}
          aria-label="Translate to Urdu"
        >
          {isTranslating ? 'Translating... / ترجمہ ہو رہا ہے...' : 'اردو میں پڑھیں'}
        </button>

        <PersonalizeButton
          onPersonalize={personalizeContent}
          isLoading={isPersonalizing}
          error={error}
          isPersonalized={false}
          isFromCache={false}
        />
      </div>

      {translationError && (
        <div className={styles.errorMessage}>
          Translation failed / ترجمہ ناکام: {translationError}
        </div>
      )}

      <div className={styles.content} ref={contentRef}>
        {children}
      </div>
    </div>
  );
}
