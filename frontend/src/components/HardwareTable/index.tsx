import React from 'react';

export interface HardwareTableProps {
  variant: 'economy' | 'workstation' | 'robot-lab' | 'cloud-native';
  lastUpdated?: string;
}

const economyKitData = [
  {
    component: 'Jetson Orin Nano Developer Kit',
    price: 249,
    specs: '10 TOPS INT8, 8GB RAM, 7-15W',
    source: 'NVIDIA Store',
    url: 'https://store.nvidia.com/',
  },
  {
    component: 'Intel RealSense D435i',
    price: 349,
    specs: 'Depth: 0.3-10m, 1280×720@90fps, IMU',
    source: 'Intel Store',
    url: 'https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html',
  },
  {
    component: 'ReSpeaker Mic Array v2.0',
    price: 69,
    specs: '4-mic circular array, USB, far-field',
    source: 'Seeed Studio',
    url: 'https://www.seeedstudio.com/',
  },
  {
    component: 'Power Supply + Cables + SD Card',
    price: 100,
    specs: '65W USB-C, cables, 128GB microSD',
    source: 'Amazon',
    url: 'https://www.amazon.com/',
  },
];

const workstationData = [
  {
    component: 'NVIDIA RTX 4070 or better',
    price: 599,
    specs: '12GB+ VRAM, CUDA cores',
    source: 'Varies',
    url: '',
  },
  {
    component: '32GB+ DDR4/DDR5 RAM',
    price: 120,
    specs: '3200MHz+',
    source: 'Amazon',
    url: 'https://www.amazon.com/',
  },
  {
    component: '1TB+ NVMe SSD',
    price: 100,
    specs: 'PCIe 4.0',
    source: 'Amazon',
    url: 'https://www.amazon.com/',
  },
  {
    component: 'Ubuntu 22.04 LTS',
    price: 0,
    specs: 'Free, open-source',
    source: 'Ubuntu',
    url: 'https://ubuntu.com/',
  },
];

const robotLabData = [
  {
    component: 'Unitree Go2 Quadruped',
    price: 2800,
    specs: 'Mid-range option, 12 motors',
    source: 'Unitree Robotics',
    url: 'https://www.unitree.com/',
  },
  {
    component: 'Unitree G1 Humanoid',
    price: 16000,
    specs: '43 DoF, advanced research platform',
    source: 'Unitree Robotics',
    url: 'https://www.unitree.com/',
  },
];

const cloudNativeData = [
  {
    component: 'AWS g5.xlarge (A10G GPU)',
    price: 1.006,
    specs: '24GB VRAM, hourly rate',
    source: 'AWS',
    url: 'https://aws.amazon.com/ec2/instance-types/g5/',
  },
  {
    component: 'Estimated quarterly cost',
    price: 205,
    specs: '5 hrs/week × 10 weeks ≈ 50 hours',
    source: 'Calculated',
    url: '',
  },
];

const dataMap = {
  economy: economyKitData,
  workstation: workstationData,
  'robot-lab': robotLabData,
  'cloud-native': cloudNativeData,
};

const HardwareTable: React.FC<HardwareTableProps> = ({
  variant,
  lastUpdated = '2025-12-04',
}) => {
  const data = dataMap[variant];
  const total = data.reduce((sum, item) => sum + item.price, 0);

  return (
    <div>
      <p style={{ fontSize: '0.9em', color: 'var(--ifm-color-emphasis-600)', marginBottom: '1rem' }}>
        <strong>Last Updated:</strong> {lastUpdated} (Prices verified as of December 2025. Check vendor sites for current pricing.)
      </p>
      <table className="hardware-table">
        <caption style={{ captionSide: 'top', textAlign: 'left', fontWeight: '600', fontSize: '1.1em', marginBottom: '0.5rem' }}>
          {variant === 'economy' && 'Economy Jetson Kit ($700)'}
          {variant === 'workstation' && 'Digital Twin Workstation'}
          {variant === 'robot-lab' && 'Robot Lab Hardware (Optional)'}
          {variant === 'cloud-native' && 'Cloud-Native Alternative'}
        </caption>
        <thead>
          <tr>
            <th scope="col">Component</th>
            <th scope="col">Price (USD)</th>
            <th scope="col">Specifications</th>
            <th scope="col">Source</th>
          </tr>
        </thead>
        <tbody>
          {data.map((item, index) => (
            <tr key={index}>
              <th scope="row">{item.component}</th>
              <td>${item.price.toFixed(variant === 'cloud-native' && index === 0 ? 3 : 0)}</td>
              <td>{item.specs}</td>
              <td>
                {item.url ? (
                  <a href={item.url} target="_blank" rel="noopener noreferrer">
                    {item.source}
                  </a>
                ) : (
                  item.source
                )}
              </td>
            </tr>
          ))}
          <tr className="total-row">
            <th scope="row">TOTAL</th>
            <td>${total.toFixed(0)}</td>
            <td colSpan={2}>
              {variant === 'economy' && 'Complete edge AI setup for real robots'}
              {variant === 'workstation' && 'One-time investment for simulation'}
              {variant === 'robot-lab' && 'Advanced research platforms'}
              {variant === 'cloud-native' && 'Pay-as-you-go for 10-week quarter'}
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  );
};

export default HardwareTable;
