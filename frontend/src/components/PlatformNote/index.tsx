import React from 'react';

export interface PlatformNoteProps {
  platforms: string[];
  notes?: Record<string, string>;
}

const PlatformNote: React.FC<PlatformNoteProps> = ({ platforms = ['Linux', 'macOS', 'WSL2'], notes }) => {
  const defaultNotes: Record<string, string> = {
    Linux: 'Native support, recommended for ROS 2',
    macOS: 'Supported via Homebrew (brew install ros)',
    WSL2: 'Requires Windows 11 + Ubuntu 22.04 WSL2 image',
    Windows: 'Limited support, WSL2 recommended',
  };

  const displayNotes = notes || defaultNotes;

  return (
    <div className="platform-note">
      <div className="platform-note__title">💻 Platform Compatibility</div>
      <table style={{ width: '100%', marginTop: '0.5rem' }}>
        <thead>
          <tr>
            <th style={{ textAlign: 'left', padding: '0.5rem' }}>Platform</th>
            <th style={{ textAlign: 'left', padding: '0.5rem' }}>Notes</th>
          </tr>
        </thead>
        <tbody>
          {platforms.map((platform) => (
            <tr key={platform}>
              <td style={{ padding: '0.5rem', fontWeight: 600 }}>{platform}</td>
              <td style={{ padding: '0.5rem' }}>{displayNotes[platform] || 'Check documentation'}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default PlatformNote;
