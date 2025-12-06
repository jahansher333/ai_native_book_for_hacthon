import { useState, useEffect, useCallback } from 'react';
import { personalizeChapter } from '../services/personalizationService';

interface UserProfile {
  experience: 'beginner' | 'intermediate' | 'advanced';
  hasRTX: boolean;
  hasJetson: boolean;
  hasRobot: boolean;
  user_email?: string;
}

interface UsePersonalizationResult {
  isPersonalizing: boolean;
  personalizedContent: string | null;
  error: Error | null;
  isFromCache: boolean;
  personalizeContent: () => Promise<void>;
  resetToDefault: () => void;
}

// Simple hash function for profile
function hashProfile(profile: UserProfile): string {
  const str = JSON.stringify({
    experience: profile.experience,
    hasRTX: profile.hasRTX,
    hasJetson: profile.hasJetson,
    hasRobot: profile.hasRobot
  });

  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash;
  }
  return Math.abs(hash).toString(16).substring(0, 8);
}

export function usePersonalization(
  chapterId: string,
  originalContent: string,
  profile: UserProfile | null,
  isAuthenticated: boolean
): UsePersonalizationResult {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [error, setError] = useState<Error | null>(null);
  const [isFromCache, setIsFromCache] = useState(false);

  // Generate cache key
  const userEmail = profile?.user_email || 'anonymous';
  const profileHash = profile ? hashProfile(profile) : '';
  const cacheKey = `personalized_${chapterId}_${userEmail}_${profileHash}`;

  // Check cache on mount
  useEffect(() => {
    if (!isAuthenticated || !profile) return;

    try {
      const cached = localStorage.getItem(cacheKey);
      if (cached) {
        const { content, timestamp } = JSON.parse(cached);
        const age = Date.now() - timestamp;
        const maxAge = 7 * 24 * 60 * 60 * 1000; // 7 days

        if (age < maxAge) {
          setPersonalizedContent(content);
          setIsFromCache(true);
        } else {
          localStorage.removeItem(cacheKey);
        }
      }
    } catch (err) {
      console.error('Cache parse error:', err);
      localStorage.removeItem(cacheKey);
    }
  }, [chapterId, cacheKey, isAuthenticated, profile]);

  const personalizeContent = useCallback(async () => {
    if (!isAuthenticated || !profile) {
      setError(new Error('Please log in to personalize content'));
      return;
    }

    setIsPersonalizing(true);
    setError(null);

    try {
      // Call backend API
      const result = await personalizeChapter({
        chapterId,
        originalContent,
        userProfile: {
          experience: profile.experience || 'intermediate',
          hasRTX: profile.hasRTX || false,
          hasJetson: profile.hasJetson || false,
          hasRobot: profile.hasRobot || false
        }
      });

      // Cache the result
      localStorage.setItem(cacheKey, JSON.stringify({
        content: result.personalizedContent,
        timestamp: Date.now()
      }));

      setPersonalizedContent(result.personalizedContent);
      setIsFromCache(false);

      // Scroll to top smoothly
      window.scrollTo({ top: 0, behavior: 'smooth' });
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Personalization failed'));
      console.error('Personalization error:', err);
    } finally {
      setIsPersonalizing(false);
    }
  }, [chapterId, originalContent, profile, isAuthenticated, cacheKey]);

  const resetToDefault = useCallback(() => {
    setPersonalizedContent(null);
    setIsFromCache(false);
    setError(null);
    localStorage.removeItem(cacheKey);
  }, [cacheKey]);

  return {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  };
}
