import React, { useEffect, useState } from 'react';

interface AuthGuardProps {
  children: React.ReactNode;
}

/**
 * Authentication guard component
 * Redirects to /signin if user is not authenticated
 */
export default function AuthGuard({ children }: AuthGuardProps) {
  const [isChecking, setIsChecking] = useState(true);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    const checkAuth = async () => {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        // No token - redirect to signin
        window.location.href = '/signin';
        return;
      }

      try {
        // Verify token with backend
        const response = await fetch('http://localhost:8000/api/auth/session', {
          headers: {
            'Authorization': `Bearer ${token}`
          }
        });

        const data = await response.json();

        if (data.authenticated) {
          // Valid token
          setIsAuthenticated(true);
          setIsChecking(false);
        } else {
          // Invalid token - clear and redirect
          localStorage.removeItem('auth_token');
          localStorage.removeItem('user_profile');
          localStorage.removeItem('user_email');
          window.location.href = '/signin';
        }
      } catch (error) {
        console.error('Auth check failed:', error);
        // On error, redirect to signin
        window.location.href = '/signin';
      }
    };

    checkAuth();
  }, []);

  if (isChecking) {
    // Show loading while checking authentication
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '60vh',
        flexDirection: 'column',
        gap: '1rem'
      }}>
        <div style={{
          width: '50px',
          height: '50px',
          border: '4px solid #f3f4f6',
          borderTop: '4px solid #3b82f6',
          borderRadius: '50%',
          animation: 'spin 1s linear infinite'
        }}></div>
        <p style={{ color: '#6b7280' }}>Checking authentication...</p>
        <style>{`
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
        `}</style>
      </div>
    );
  }

  if (!isAuthenticated) {
    return null; // Will redirect in useEffect
  }

  return <>{children}</>;
}
