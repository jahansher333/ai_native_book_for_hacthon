/**
 * Personalization Service - API client for chapter personalization
 *
 * This service handles all communication with the backend personalization API,
 * including authentication, error handling, and timeout management.
 */
import { API_ENDPOINTS } from '../config';

export interface UserProfile {
  experience: 'beginner' | 'intermediate' | 'advanced';
  hasRTX: boolean;
  hasJetson: boolean;
  hasRobot: boolean;
}

export interface PersonalizeChapterRequest {
  chapterId: string;
  originalContent: string;
  userProfile: UserProfile;
}

export interface PersonalizationMetadata {
  generationTime: number;
  cached: boolean;
  modelUsed?: string;
}

export interface PersonalizeChapterResponse {
  success: boolean;
  personalizedContent: string;
  metadata: PersonalizationMetadata;
}

/**
 * Personalize a chapter using the backend API
 *
 * @param request - Personalization request containing chapter content and user profile
 * @returns Personalized chapter response
 * @throws Error with specific message based on HTTP status code
 */
export async function personalizeChapter(
  request: PersonalizeChapterRequest
): Promise<PersonalizeChapterResponse> {
  // Get JWT token from localStorage
  const token = localStorage.getItem('auth_token');

  if (!token) {
    throw new Error('Not authenticated. Please log in to personalize chapters.');
  }

  // Create AbortController for 60-second timeout
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 60000);

  try {
    const response = await fetch(API_ENDPOINTS.PERSONALIZE_CHAPTER, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    // Handle different HTTP status codes
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
      const errorMessage = errorData.detail || response.statusText;

      switch (response.status) {
        case 401:
          localStorage.removeItem('auth_token'); // Clear invalid token
          throw new Error('Session expired, please log in again');

        case 403:
          throw new Error('Permission denied. Please complete your profile.');

        case 422:
          throw new Error(`Invalid request: ${errorMessage}`);

        case 429:
          throw new Error('Too many requests. Please wait before trying again.');

        case 504:
          throw new Error('Personalization took too long. Try a shorter chapter.');

        case 500:
        default:
          throw new Error('Something went wrong. Please retry.');
      }
    }

    const data: PersonalizeChapterResponse = await response.json();

    if (!data.success || !data.personalizedContent) {
      throw new Error('Invalid response from server');
    }

    return data;

  } catch (error) {
    clearTimeout(timeoutId);

    // Handle timeout
    if (error instanceof Error && error.name === 'AbortError') {
      throw new Error('Personalization took too long (timeout after 60s). Try a shorter chapter.');
    }

    // Handle network errors
    if (error instanceof TypeError) {
      throw new Error('Network error. Please check your connection.');
    }

    // Re-throw known errors
    throw error;
  }
}

/**
 * Check if user is authenticated
 *
 * @returns True if auth token exists in localStorage
 */
export function isAuthenticated(): boolean {
  return !!localStorage.getItem('auth_token');
}

/**
 * Get JWT token from localStorage
 *
 * @returns JWT token or null if not found
 */
export function getAuthToken(): string | null {
  return localStorage.getItem('auth_token');
}
