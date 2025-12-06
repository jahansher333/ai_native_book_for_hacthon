import React, { JSX } from 'react';

interface PersonalizationBadgeProps {
  profile: {
    experience: 'beginner' | 'intermediate' | 'advanced';
    hasRTX: boolean;
    hasJetson: boolean;
    hasRobot: boolean;
  };
  isFromCache: boolean;
  onReset: () => void;
}

export default function PersonalizationBadge({
  profile,
  isFromCache,
  onReset
}: PersonalizationBadgeProps): JSX.Element {
  // Build hardware list
  const hardwareList: string[] = [];
  if (profile.hasRTX) hardwareList.push('RTX GPU');
  if (profile.hasJetson) hardwareList.push('Jetson');
  if (profile.hasRobot) hardwareList.push('Real Robot');

  const hardwareText = hardwareList.length > 0
    ? ` with ${hardwareList.join(' & ')}`
    : ' (simulation-focused)';

  const experienceText = profile.experience.charAt(0).toUpperCase() +
                         profile.experience.slice(1);

  return (
    <div className="mb-6 p-4 bg-gradient-to-r from-blue-50 to-purple-50 dark:from-blue-900/20 dark:to-purple-900/20 border-2 border-blue-200 dark:border-blue-700/50 rounded-xl flex flex-col md:flex-row items-start md:items-center justify-between gap-3">
      <div className="flex items-center gap-2 flex-wrap">
        <span className="text-xl">📌</span>
        <span className="text-sm text-gray-700 dark:text-gray-300">
          Personalized for: <strong>{experienceText}</strong> user{hardwareText}
        </span>
        {isFromCache && (
          <span className="text-xs text-gray-500 italic ml-1">(cached)</span>
        )}
      </div>
      <button
        onClick={onReset}
        className="px-3 py-1.5 bg-white dark:bg-gray-800 border border-gray-300 dark:border-gray-600 rounded-lg text-sm hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors duration-200"
        aria-label="Reset to default content"
      >
        Reset to Default
      </button>
    </div>
  );
}
