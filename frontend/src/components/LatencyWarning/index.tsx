import React from 'react';

export interface LatencyWarningProps {
  latencyRange?: string;
  context?: string;
}

const LatencyWarning: React.FC<LatencyWarningProps> = ({
  latencyRange = '50-200ms+',
  context = 'cloud robot control',
}) => {
  return (
    <div className="latency-warning">
      <div className="latency-warning__title">⚠️ LATENCY TRAP WARNING ⚠️</div>
      <div className="latency-warning__content">
        <p>
          This architecture sends commands over the network. Network latency (<strong>{latencyRange}</strong>) makes this <strong>UNSAFE</strong> for real robots.
        </p>
        <p>Use this pattern ONLY for:</p>
        <ul>
          <li>Simulation environments (Isaac Sim, Gazebo)</li>
          <li>High-level planning (not real-time control)</li>
          <li>Data collection (not actuation)</li>
        </ul>
        <p>
          <strong>For real robots:</strong> Deploy models to edge devices (Jetson) for &lt;10ms inference.
        </p>
      </div>
    </div>
  );
};

export default LatencyWarning;
