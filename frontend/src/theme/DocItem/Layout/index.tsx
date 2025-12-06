import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';
import {useLocation} from '@docusaurus/router';
import { useUserProfile } from '../../../contexts/UserProfileContext';
import PersonalizeButton from '../../../components/Personalize/PersonalizeButton';
import PersonalizationBadge from '../../../components/Personalize/PersonalizationBadge';
import LoadingOverlay from '../../../components/Personalize/LoadingOverlay';
import { usePersonalization } from '../../../hooks/usePersonalization';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): React.JSX.Element {
  const location = useLocation();
  const { isAuthenticated, profile } = useUserProfile();

  // Extract chapter ID from pathname
  const chapterId = location.pathname
    .replace('/docs/', '')
    .replace(/\/$/, '')
    .replace(/^\//, '');

  // Simple content extraction for personalization
  const originalContent = props.children?.toString() || '';

  const {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  } = usePersonalization(chapterId, originalContent, profile, isAuthenticated);

  // If not authenticated, just render original layout
  if (!isAuthenticated) {
    return <Layout {...props} />;
  }

  // If personalized content exists, show badge above
  return (
    <div className="docItem-wrapper">
      {personalizedContent ? (
        <PersonalizationBadge
          profile={{
            experience: profile?.experience || 'intermediate',
            hasRTX: profile?.hasRTX || false,
            hasJetson: profile?.hasJetson || false,
            hasRobot: profile?.hasRobot || false
          }}
          isFromCache={isFromCache}
          onReset={resetToDefault}
        />
      ) : (
        <div style={{ marginBottom: '1.5rem' }}>
          <PersonalizeButton
            onPersonalize={personalizeContent}
            isLoading={isPersonalizing}
            error={error}
          />
        </div>
      )}
      {isPersonalizing && <LoadingOverlay />}
      <Layout {...props} />
    </div>
  );
}
