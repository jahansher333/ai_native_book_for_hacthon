import React, { JSX } from 'react';

interface PersonalizeButtonProps {
  onPersonalize: () => Promise<void>;
  isLoading: boolean;
  error: Error | null;
}

export default function PersonalizeButton({
  onPersonalize,
  isLoading,
  error
}: PersonalizeButtonProps): JSX.Element {
  return (
    <div className="mb-6 flex flex-col items-start gap-3">
      <button
        onClick={onPersonalize}
        disabled={isLoading}
        className="inline-flex items-center gap-2 px-6 py-3 rounded-xl font-semibold text-white bg-gradient-to-r from-blue-500 via-purple-500 to-pink-500 hover:shadow-lg hover:-translate-y-0.5 transition-all duration-300 disabled:opacity-50 disabled:cursor-not-allowed disabled:hover:translate-y-0"
        style={{ backgroundSize: '200% 100%', animation: isLoading ? 'none' : 'gradientShift 8s ease infinite' }}
        aria-label="Personalize this chapter to your experience and hardware"
        title="Adapt this chapter to your experience and hardware"
      >
        <span className="text-lg">✨</span>
        <span className="text-sm md:text-base">
          {isLoading ? 'Personalizing...' : 'Personalize for Me'}
        </span>
        {isLoading && (
          <svg className="animate-spin h-5 w-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
          </svg>
        )}
      </button>

      {error && (
        <div className="p-3 bg-red-50 border border-red-200 rounded-lg text-red-800 text-sm flex items-center gap-2 w-full" role="alert">
          <span>❌ {error.message}</span>
          <button
            onClick={onPersonalize}
            className="ml-auto px-3 py-1 bg-red-600 text-white rounded hover:bg-red-700 text-xs transition-colors"
          >
            Retry
          </button>
        </div>
      )}
    </div>
  );
}
