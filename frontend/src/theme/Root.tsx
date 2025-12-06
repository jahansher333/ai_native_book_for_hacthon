import React from 'react';
import RagChat from '@site/src/components/RagChat';
import { UserProfileProvider } from '@site/src/contexts/UserProfileContext';

// Root component wraps the entire Docusaurus application
export default function Root({children}) {
  return (
    <UserProfileProvider>
      {children}
      <RagChat />
    </UserProfileProvider>
  );
}
