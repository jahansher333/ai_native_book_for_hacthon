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

export async function personalizeChapter(
  request: PersonalizeRequest
): Promise<PersonalizeResponse> {
  const token = localStorage.getItem('auth_token');

  if (!token) {
    throw new Error('Authentication required. Please log in.');
  }

  const response = await fetch('http://localhost:8000/api/personalize/chapter', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`
    },
    body: JSON.stringify(request)
  });

  if (response.status === 401) {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_profile');
    localStorage.removeItem('user_email');
    window.location.href = '/signin';
    throw new Error('Session expired. Please log in again.');
  }

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Personalization failed' }));
    throw new Error(error.detail || 'Personalization failed');
  }

  return response.json();
}
