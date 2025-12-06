import React from 'react';
import { useUserProfile } from '../../contexts/UserProfileContext';
import PersonalizeButton from './PersonalizeButton';
import PersonalizationBadge from './PersonalizationBadge';
import LoadingOverlay from './LoadingOverlay';
import { usePersonalization } from '../../hooks/usePersonalization';

interface ChapterWrapperProps {
  chapterId: string;
  children: React.ReactNode;
}

export default function ChapterWrapper({
  chapterId,
  children
}: ChapterWrapperProps): React.JSX.Element {
  const { isAuthenticated, profile } = useUserProfile();

  // Extract original content as string
  const originalContent = typeof children === 'string'
    ? children
    : React.Children.toArray(children).map(child => {
        if (typeof child === 'string') return child;
        if (React.isValidElement(child) && child.props.children) {
          return extractText(child.props.children);
        }
        return '';
      }).join('\n');

  const {
    isPersonalizing,
    personalizedContent,
    error,
    isFromCache,
    personalizeContent,
    resetToDefault
  } = usePersonalization(chapterId, originalContent, profile, isAuthenticated);

  // Helper to extract text from React elements
  function extractText(element: any): string {
    if (typeof element === 'string') return element;
    if (Array.isArray(element)) {
      return element.map(extractText).join('');
    }
    if (React.isValidElement(element) && (element.props as any).children) {
      return extractText((element.props as any).children);
    }
    return '';
  }

  // If not authenticated, render original content only
  if (!isAuthenticated) {
    return <>{children}</>;
  }

  // If personalized content exists, render it
  if (personalizedContent) {
    return (
      <div className="chapter-wrapper">
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
        <div
          className="personalized-content markdown"
          dangerouslySetInnerHTML={{ __html: personalizedContent }}
        />
      </div>
    );
  }

  // Default: render original content with personalize button
  return (
    <div className="chapter-wrapper">
      <PersonalizeButton
        onPersonalize={personalizeContent}
        isLoading={isPersonalizing}
        error={error}
      />
      {isPersonalizing && <LoadingOverlay />}
      <div className="original-content">
        {children}
      </div>
    </div>
  );
}
