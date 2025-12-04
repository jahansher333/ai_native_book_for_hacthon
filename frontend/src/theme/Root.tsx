import React from 'react';
import RagChat from '@site/src/components/RagChat';

// Root component wraps the entire Docusaurus application
export default function Root({children}) {
  return (
    <>
      {children}
      <RagChat />
    </>
  );
}
