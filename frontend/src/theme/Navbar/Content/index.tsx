import React, { useState, useEffect } from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type { WrapperProps } from '@docusaurus/types';
import styles from './styles.module.css';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [profileSummary, setProfileSummary] = useState('');

  useEffect(() => {
    const checkAuth = () => {
      const token = localStorage.getItem('auth_token');
      const profileData = localStorage.getItem('user_profile');

      if (token && profileData) {
        setIsAuthenticated(true);

        try {
          const profile = JSON.parse(profileData);
          const hardware = [];
          if (profile.hasRTX) hardware.push('RTX GPU');
          if (profile.hasJetson) hardware.push('Jetson');
          if (profile.hasRobot) hardware.push('Real Robot');

          const hardwareStr = hardware.length > 0 ? hardware.join(', ') : 'No hardware';
          const experience = profile.experience ? profile.experience.charAt(0).toUpperCase() + profile.experience.slice(1) : 'Beginner';
          setProfileSummary(`${experience} • ${hardwareStr}`);
        } catch (e) {
          console.error('Failed to parse profile:', e);
        }
      }
    };

    checkAuth();

    // Listen for storage changes (cross-tab sync)
    window.addEventListener('storage', checkAuth);
    return () => window.removeEventListener('storage', checkAuth);
  }, []);

  const handleSignOut = () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_profile');
    localStorage.removeItem('user_email');
    window.location.href = '/';
  };

  return (
    <>
      <Content {...props} />
      {isAuthenticated && (
        <div className={styles.authInfo}>
          <a href="/profile" className={styles.profileLink}>
            {profileSummary}
          </a>
          <button onClick={handleSignOut} className={styles.signOutButton}>
            Sign Out
          </button>
        </div>
      )}
    </>
  );
}
