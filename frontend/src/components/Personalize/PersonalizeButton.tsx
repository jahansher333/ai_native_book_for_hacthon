/**
 * PersonalizeButton - Button component for triggering chapter personalization
 *
 * Displays a button that triggers personalization when clicked.
 * Shows loading state, error messages, and personalization status.
 */
import React from 'react';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  onPersonalize: () => void;
  isLoading: boolean;
  error: Error | null;
  isPersonalized: boolean;
  isFromCache: boolean;
}

export default function PersonalizeButton({
  onPersonalize,
  isLoading,
  error,
  isPersonalized,
  isFromCache
}: PersonalizeButtonProps): React.JSX.Element {
  return (
    <div className={styles.container}>
      <button
        onClick={onPersonalize}
        disabled={isLoading}
        className={`${styles.button} ${isLoading ? styles.loading : ''} ${isPersonalized ? styles.personalized : ''}`}
        aria-label="Personalize chapter content"
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : isPersonalized ? (
          <>
            <span className={styles.checkmark}>✓</span>
            {isFromCache ? 'Personalized (Cached)' : 'Personalized'}
          </>
        ) : (
          <>
            <span className={styles.icon}>✨</span>
            Personalize Chapter
          </>
        )}
      </button>

      {error && (
        <div className={styles.error} role="alert">
          <span className={styles.errorIcon}>⚠️</span>
          {error.message}
          {!error.message.includes('log in') && (
            <button
              onClick={onPersonalize}
              className={styles.retryButton}
              aria-label="Retry personalization"
            >
              Retry
            </button>
          )}
        </div>
      )}
    </div>
  );
}
