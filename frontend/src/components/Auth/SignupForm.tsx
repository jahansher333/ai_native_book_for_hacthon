import React, { useState } from 'react';
import styles from '../../css/auth.module.css';
import Link from '@docusaurus/Link';

interface ProfileData {
  hasRTX: boolean | null;
  hasJetson: boolean | null;
  hasRobot: boolean | null;
  experience: string;
}

export default function SignupForm() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [profile, setProfile] = useState<ProfileData>({
    hasRTX: null,
    hasJetson: null,
    hasRobot: null,
    experience: ''
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    // Client-side validation
    if (!email || !password) {
      setError('Email and password are required');
      setLoading(false);
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      setLoading(false);
      return;
    }

    if (profile.hasRTX === null || profile.hasJetson === null || profile.hasRobot === null) {
      setError('Please answer all hardware questions');
      setLoading(false);
      return;
    }

    if (!profile.experience) {
      setError('Please select your programming experience');
      setLoading(false);
      return;
    }

    try {
      const response = await fetch('http://localhost:8000/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email,
          password,
          profile: {
            hasRTX: profile.hasRTX,
            hasJetson: profile.hasJetson,
            hasRobot: profile.hasRobot,
            experience: profile.experience
          }
        })
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.detail || 'Signup failed');
        setLoading(false);
        return;
      }

      // Store token and profile
      localStorage.setItem('auth_token', data.token);
      localStorage.setItem('user_profile', JSON.stringify(data.user.profile));
      localStorage.setItem('user_email', data.user.email);

      // Redirect to docs after successful signup
      window.location.href = '/docs/intro';

    } catch (err) {
      setError('Network error. Please try again.');
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.signupForm}>
      <h2 className={styles.title}>Create Your Account</h2>
      <p className={styles.subtitle}>
        Get personalized learning recommendations based on your hardware
      </p>

      {/* Email */}
      <div className={styles.formGroup}>
        <label htmlFor="email">Email Address</label>
        <input
          type="email"
          id="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="student@example.com"
          required
          className={styles.input}
        />
      </div>

      {/* Password */}
      <div className={styles.formGroup}>
        <label htmlFor="password">Password</label>
        <div className={styles.passwordInput}>
          <input
            type={showPassword ? 'text' : 'password'}
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="Min 8 characters"
            required
            className={styles.input}
          />
          <button
            type="button"
            className={styles.togglePassword}
            onClick={() => setShowPassword(!showPassword)}
            aria-label="Toggle password visibility"
          >
            {showPassword ? '👁️' : '👁️‍🗨️'}
          </button>
        </div>
      </div>

      {/* Profile Questions */}
      <div className={styles.profileSection}>
        <h3 className={styles.sectionTitle}>Tell us about your setup</h3>
        <p className={styles.sectionSubtitle}>
          This helps us recommend the right tutorials for you
        </p>

        {/* Question 1: RTX GPU */}
        <div className={styles.questionGroup}>
          <label className={styles.questionLabel}>
            1. Do you have an NVIDIA RTX GPU?
          </label>
          <div className={styles.radioGroup}>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasRTX"
                checked={profile.hasRTX === true}
                onChange={() => setProfile({ ...profile, hasRTX: true })}
              />
              <span>Yes</span>
            </label>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasRTX"
                checked={profile.hasRTX === false}
                onChange={() => setProfile({ ...profile, hasRTX: false })}
              />
              <span>No</span>
            </label>
          </div>
        </div>

        {/* Question 2: Jetson */}
        <div className={styles.questionGroup}>
          <label className={styles.questionLabel}>
            2. Do you own a Jetson Orin Nano ($249)?
          </label>
          <div className={styles.radioGroup}>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasJetson"
                checked={profile.hasJetson === true}
                onChange={() => setProfile({ ...profile, hasJetson: true })}
              />
              <span>Yes</span>
            </label>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasJetson"
                checked={profile.hasJetson === false}
                onChange={() => setProfile({ ...profile, hasJetson: false })}
              />
              <span>No</span>
            </label>
          </div>
        </div>

        {/* Question 3: Real Robot */}
        <div className={styles.questionGroup}>
          <label className={styles.questionLabel}>
            3. Do you have access to a real robot?
          </label>
          <div className={styles.radioGroup}>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasRobot"
                checked={profile.hasRobot === true}
                onChange={() => setProfile({ ...profile, hasRobot: true })}
              />
              <span>Yes</span>
            </label>
            <label className={styles.radioLabel}>
              <input
                type="radio"
                name="hasRobot"
                checked={profile.hasRobot === false}
                onChange={() => setProfile({ ...profile, hasRobot: false })}
              />
              <span>No</span>
            </label>
          </div>
        </div>

        {/* Question 4: Experience */}
        <div className={styles.questionGroup}>
          <label htmlFor="experience" className={styles.questionLabel}>
            4. Your programming experience?
          </label>
          <select
            id="experience"
            value={profile.experience}
            onChange={(e) => setProfile({ ...profile, experience: e.target.value })}
            className={styles.select}
            required
          >
            <option value="">Select your level</option>
            <option value="beginner">Beginner (Learning Python basics)</option>
            <option value="intermediate">Intermediate (Comfortable with Python & ML)</option>
            <option value="advanced">Advanced (Production ML experience)</option>
          </select>
        </div>
      </div>

      {/* Error Message */}
      {error && <div className={styles.errorMessage}>{error}</div>}

      {/* Submit Button */}
      <button type="submit" className={styles.submitButton} disabled={loading}>
        {loading ? 'Creating Account...' : 'Create Account'}
      </button>

      {/* Signin Link */}
      <p className={styles.footerLink}>
        Already have an account? <a href="/signin">Sign in</a>
      </p>
    </form>
  );
}
