/**
 * Frontend Configuration
 * Centralized API endpoint configuration
 */

// Backend API URL - use environment variable or default to Railway backend
export const API_BASE_URL = process.env.REACT_APP_API_URL || 'https://web-production-d0418.up.railway.app';

// API Endpoints
export const API_ENDPOINTS = {
  // Authentication
  AUTH_SIGNUP: `${API_BASE_URL}/api/auth/signup`,
  AUTH_SIGNIN: `${API_BASE_URL}/api/auth/signin`,
  AUTH_SESSION: `${API_BASE_URL}/api/auth/session`,
  AUTH_LOGOUT: `${API_BASE_URL}/api/auth/logout`,
  AUTH_PROFILE: `${API_BASE_URL}/api/auth/profile`,

  // RAG Chatbot
  QUERY: `${API_BASE_URL}/api/query`,

  // Personalization
  PERSONALIZE_CHAPTER: `${API_BASE_URL}/api/personalize/chapter`,

  // Translation
  TRANSLATE_CHAPTER: `${API_BASE_URL}/api/translate/chapter`,

  // Health
  HEALTH: `${API_BASE_URL}/api/health`,
};

export default { API_BASE_URL, API_ENDPOINTS };
