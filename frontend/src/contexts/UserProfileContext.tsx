import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface UserProfile {
  hasRTX: boolean;
  hasJetson: boolean;
  hasRobot: boolean;
  experience: 'beginner' | 'intermediate' | 'advanced';
  profile_version?: number;
  created_at?: string;
}

interface User {
  id: string;
  email: string;
  profile: UserProfile;
  email_verified: boolean;
  created_at: string;
}

interface UserProfileContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signOut: () => void;
  refreshUser: () => Promise<void>;
}

const UserProfileContext = createContext<UserProfileContextType | undefined>(undefined);

export function UserProfileProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  const refreshUser = async () => {
    const token = localStorage.getItem('auth_token');

    if (!token) {
      setUser(null);
      setProfile(null);
      setIsAuthenticated(false);
      setIsLoading(false);
      return;
    }

    try {
      const response = await fetch('/api/auth/session', {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      const data = await response.json();

      if (data.authenticated && data.user) {
        setUser(data.user);
        setProfile(data.user.profile);
        setIsAuthenticated(true);

        // Update localStorage
        localStorage.setItem('user_profile', JSON.stringify(data.user.profile));
        localStorage.setItem('user_email', data.user.email);
      } else {
        // Invalid token
        localStorage.removeItem('auth_token');
        localStorage.removeItem('user_profile');
        localStorage.removeItem('user_email');
        setUser(null);
        setProfile(null);
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error('Failed to refresh user:', error);
      setUser(null);
      setProfile(null);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  const signOut = () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_profile');
    localStorage.removeItem('user_email');
    setUser(null);
    setProfile(null);
    setIsAuthenticated(false);
    window.location.href = '/';
  };

  useEffect(() => {
    refreshUser();
  }, []);

  return (
    <UserProfileContext.Provider
      value={{
        user,
        profile,
        isAuthenticated,
        isLoading,
        signOut,
        refreshUser
      }}
    >
      {children}
    </UserProfileContext.Provider>
  );
}

export function useUserProfile() {
  const context = useContext(UserProfileContext);
  if (context === undefined) {
    throw new Error('useUserProfile must be used within a UserProfileProvider');
  }
  return context;
}

// Helper hook for getting profile summary
export function useProfileSummary(): string {
  const { profile } = useUserProfile();

  if (!profile) return 'Guest User';

  const hardware = [];
  if (profile.hasRTX) hardware.push('RTX GPU');
  if (profile.hasJetson) hardware.push('Jetson');
  if (profile.hasRobot) hardware.push('Real Robot');

  const hardwareStr = hardware.length > 0 ? hardware.join(', ') : 'No hardware yet';
  const experience = profile.experience.charAt(0).toUpperCase() + profile.experience.slice(1);

  return `${experience} developer with ${hardwareStr}`;
}
