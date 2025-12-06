import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './profile.module.css';

interface UserProfile {
  hasRTX: boolean;
  hasJetson: boolean;
  hasRobot: boolean;
  experience: string;
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

export default function ProfilePage() {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const fetchProfile = async () => {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        window.location.href = '/signin';
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
        } else {
          window.location.href = '/signin';
        }
      } catch (error) {
        console.error('Failed to fetch profile:', error);
        window.location.href = '/signin';
      } finally {
        setLoading(false);
      }
    };

    fetchProfile();
  }, []);

  if (loading) {
    return (
      <Layout title="Profile" description="Your profile and hardware setup">
        <div className="container" style={{ marginTop: '2rem', textAlign: 'center' }}>
          <p>Loading profile...</p>
        </div>
      </Layout>
    );
  }

  if (!user) {
    return null; // Redirecting to signin
  }

  const profile = user.profile;
  const hardware = [];
  if (profile.hasRTX) hardware.push('NVIDIA RTX GPU');
  if (profile.hasJetson) hardware.push('Jetson Orin Nano ($249)');
  if (profile.hasRobot) hardware.push('Real Robot');

  const getRecommendations = () => {
    const recs = [];

    if (profile.hasRTX && !profile.hasJetson) {
      recs.push({
        title: 'Get a Jetson Orin Nano ($249)',
        description: 'You have an RTX GPU for training! Add a Jetson for edge deployment to complete your Physical AI workflow.',
        link: '/docs/hardware/economy-kit'
      });
    }

    if (!profile.hasRTX && profile.hasJetson) {
      recs.push({
        title: 'Train models in the cloud',
        description: 'You have a Jetson for deployment! Use cloud GPUs (Google Colab, AWS) for training VLA models.',
        link: '/docs/intro'
      });
    }

    if (profile.experience === 'beginner') {
      recs.push({
        title: 'Start with ROS 2 Basics',
        description: 'Learn the fundamentals of ROS 2 nodes, topics, and services before diving into Physical AI.',
        link: '/docs/intro'
      });
    }

    if (profile.experience === 'advanced' && profile.hasRobot) {
      recs.push({
        title: 'Sim-to-Real Transfer',
        description: 'You have a real robot! Focus on bridging the sim-to-real gap with domain randomization.',
        link: '/docs/hardware/latency-trap'
      });
    }

    if (!profile.hasRobot) {
      recs.push({
        title: 'Build a $700 Economy Kit',
        description: 'Get hands-on with a Jetson + RealSense D435i camera for complete Physical AI workflows.',
        link: '/docs/hardware/economy-kit'
      });
    }

    return recs;
  };

  const recommendations = getRecommendations();

  return (
    <Layout
      title="Your Profile"
      description="Your hardware setup and personalized learning recommendations"
    >
      <div className="container" style={{ marginTop: '2rem', maxWidth: '900px' }}>
        <div className={styles.profileHeader}>
          <h1>Your Profile</h1>
          <p className={styles.email}>{user.email}</p>
        </div>

        {/* Hardware Setup */}
        <div className={styles.section}>
          <h2>🛠️ Your Hardware Setup</h2>
          {hardware.length > 0 ? (
            <div className={styles.hardwareList}>
              {profile.hasRTX && (
                <div className={styles.hardwareCard}>
                  <div className={styles.hardwareIcon}>🎮</div>
                  <div>
                    <h3>NVIDIA RTX GPU</h3>
                    <p>Perfect for training VLA models locally</p>
                  </div>
                </div>
              )}
              {profile.hasJetson && (
                <div className={styles.hardwareCard}>
                  <div className={styles.hardwareIcon}>🤖</div>
                  <div>
                    <h3>Jetson Orin Nano</h3>
                    <p>Edge device for real-time robot control (&lt;10ms)</p>
                  </div>
                </div>
              )}
              {profile.hasRobot && (
                <div className={styles.hardwareCard}>
                  <div className={styles.hardwareIcon}>🦾</div>
                  <div>
                    <h3>Real Robot</h3>
                    <p>Ready for sim-to-real transfer experiments</p>
                  </div>
                </div>
              )}
            </div>
          ) : (
            <p className={styles.noHardware}>
              No hardware yet? Check out the <a href="/docs/hardware/economy-kit">$700 Economy Kit</a>
            </p>
          )}
        </div>

        {/* Experience Level */}
        <div className={styles.section}>
          <h2>📊 Programming Experience</h2>
          <div className={styles.experienceBadge}>
            {profile.experience.charAt(0).toUpperCase() + profile.experience.slice(1)} Developer
          </div>
        </div>

        {/* Personalized Recommendations */}
        {recommendations.length > 0 && (
          <div className={styles.section}>
            <h2>💡 Personalized Recommendations</h2>
            <div className={styles.recommendationsList}>
              {recommendations.map((rec, index) => (
                <div key={index} className={styles.recommendationCard}>
                  <h3>{rec.title}</h3>
                  <p>{rec.description}</p>
                  <a href={rec.link} className={styles.recommendationLink}>
                    Learn more →
                  </a>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Account Info */}
        <div className={styles.section}>
          <h2>ℹ️ Account Information</h2>
          <div className={styles.accountInfo}>
            <p><strong>User ID:</strong> {user.id}</p>
            <p><strong>Email Verified:</strong> {user.email_verified ? 'Yes' : 'No'}</p>
            <p><strong>Member Since:</strong> {new Date(user.created_at).toLocaleDateString()}</p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
