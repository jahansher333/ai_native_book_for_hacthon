import React from 'react';

export default function LoadingOverlay(): JSX.Element {
  return (
    <div
      className="fixed inset-0 bg-black/50 backdrop-blur-sm z-50 flex items-center justify-center animate-fade-in"
      role="status"
      aria-live="polite"
    >
      <div className="bg-white dark:bg-gray-800 rounded-2xl p-8 shadow-2xl max-w-md mx-4 flex flex-col items-center gap-4">
        <div className="w-12 h-12 border-4 border-purple-200 border-t-purple-600 rounded-full animate-spin"></div>
        <p className="text-lg font-semibold text-gray-800 dark:text-gray-200">
          Personalizing content for you...
        </p>
        <p className="text-sm text-gray-600 dark:text-gray-400">
          This may take 5-10 seconds
        </p>
      </div>

      <style jsx>{`
        @keyframes fade-in {
          from { opacity: 0; }
          to { opacity: 1; }
        }
        .animate-fade-in {
          animation: fade-in 0.2s ease-in;
        }
      `}</style>
    </div>
  );
}
