/**
 * ChapterWrapper - Unified wrapper for chapter personalization and Urdu translation
 *
 * This component:
 * - Renders THREE buttons: Personalize, Urdu/English, and Loading indicator
 * - Manages both personalization and translation state
 * - Uses LiteLLM Groq agents for both features
 * - Displays comprehensive badge showing active state
 * - Sanitizes HTML with DOMPurify to prevent XSS attacks
 */
import React, { useEffect, useRef, useState } from 'react';
import DOMPurify from 'dompurify';
import { usePersonalization } from '../../hooks/usePersonalization';
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
    // Validate originalContent is available and meets minimum length
    if (!originalContent || originalContent.trim().length < 100) {
      console.error('[ChapterWrapper] Original content too short:', originalContent?.length || 0);
      setTranslationError('Chapter content is too short for translation (minimum 100 characters)');
      return;
    }

    console.log('[ChapterWrapper] Starting Urdu translation, content length:', originalContent.length);

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
      const response = await fetch('http://localhost:8001/api/translate/chapter', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          chapterId: chapterId.toLowerCase().replace(/[^a-z0-9-_/]/g, '-'),
          originalContent: originalContent
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        // Handle validation errors (422)
        if (errorData.detail && Array.isArray(errorData.detail)) {
          const errors = errorData.detail.map((e: any) => e.msg).join(', ');
          throw new Error(`Validation error: ${errors}`);
        }
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

  // Generate badge text based on active state
  const generateBadgeText = () => {
    if (isUrdu) {
      return isFromUrduCache ? 'اردو میں دکھایا جا رہا ہے (Cached) 🇵🇰' : 'اردو میں دکھایا جا رہا ہے 🇵🇰';
    }
    if (personalizedContent && profile) {
      const experienceLabel = profile.experience.charAt(0).toUpperCase() + profile.experience.slice(1);
      const hardware = [];
      if (profile.hasJetson) hardware.push('Jetson Owner');
      if (profile.hasRTX) hardware.push('RTX GPU');
      if (profile.hasRobot) hardware.push('Robot Owner');
      const hardwareText = hardware.length > 0 ? ` + ${hardware.join(' + ')}` : '';
      const cacheIndicator = isFromCache ? ' (Cached)' : '';
      return `✨ Personalized for ${experienceLabel}${hardwareText}${cacheIndicator}`;
    }
    return null;
  };

  const badgeText = generateBadgeText();

  // Determine what content to display
  const displayContent = isUrdu && urduContent ? urduContent : personalizedContent ? personalizedContent : null;

  // Sanitize content if we have any
  const sanitizedContent = displayContent ? DOMPurify.sanitize(displayContent, {
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
  }) : null;

  // Unified render with three buttons at top
  return (
    <div className={styles.wrapper}>
      {/* Three-button control bar */}
      <div className={styles.controls}>
        {/* Button 1: Personalize (only if logged in) */}
        {isAuthenticated && profile && (
          <button
            onClick={personalizeContent}
            disabled={isPersonalizing || isTranslating}
            className={`${styles.personalizeButton} ${personalizedContent ? styles.active : ''} ${isPersonalizing ? styles.loading : ''}`}
            aria-label="Personalize chapter for your profile"
          >
            {isPersonalizing ? (
              <>
                <span className={styles.spinner}></span>
                Personalizing...
              </>
            ) : personalizedContent ? (
              <>✓ Personalized</>
            ) : (
              <>✨ Personalize for Me</>
            )}
          </button>
        )}

        {/* Button 2: Urdu Toggle */}
        <button
          onClick={isUrdu ? handleBackToEnglish : handleUrduClick}
          disabled={isTranslating || isPersonalizing}
          className={`${styles.urduButton} ${isUrdu ? styles.active : ''} ${isTranslating ? styles.loading : ''}`}
          aria-label={isUrdu ? "Switch back to English" : "Translate to Urdu"}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              Translating...
            </>
          ) : isUrdu ? (
            'English میں واپس'
          ) : (
            'اردو میں پڑھیں'
          )}
        </button>

        {/* Button 3: Restore Original (only if personalized OR urdu) */}
        {(personalizedContent || isUrdu) && (
          <button
            onClick={() => {
              if (isUrdu) handleBackToEnglish();
              if (personalizedContent) resetToDefault();
            }}
            className={styles.resetButton}
            aria-label="Restore original chapter content"
          >
            🔄 Restore Original
          </button>
        )}
      </div>

      {/* Badge showing active state */}
      {badgeText && (
        <div className={`${styles.badge} ${isUrdu ? styles.urdoBadge : styles.personalizedBadge}`}>
          {badgeText}
        </div>
      )}

      {/* Error messages */}
      {error && (
        <div className={styles.errorMessage} role="alert">
          <span className={styles.errorIcon}>⚠️</span>
          Personalization failed: {error.message}
        </div>
      )}

      {translationError && (
        <div className={styles.errorMessage} role="alert">
          <span className={styles.errorIcon}>⚠️</span>
          Translation failed / ترجمہ ناکام: {translationError}
        </div>
      )}

      {/* Content display */}
      {sanitizedContent ? (
        <div
          className={`${styles.content} ${isUrdu ? styles.urduContent : styles.personalized} markdown`}
          dangerouslySetInnerHTML={{ __html: sanitizedContent }}
        />
      ) : (
        <div className={styles.content} ref={contentRef}>
          {children}
        </div>
      )}
    </div>
  );
}
