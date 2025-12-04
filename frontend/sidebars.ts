import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'course-overview',
    'hardware',
    {
      type: 'category',
      label: '🤖 Module 1: Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'ros2/index',
        'ros2/nodes',
        'ros2/topics',
        'ros2/services',
        'ros2/actions',
        'ros2/rclpy',
        'ros2/urdf',
      ],
    },
    {
      type: 'category',
      label: '🏗️ Module 2: Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'gazebo-unity/index',
        'gazebo-unity/physics-sim',
        'gazebo-unity/gazebo-setup',
        'gazebo-unity/unity-sim',
        'gazebo-unity/sensors-lidar',
        'gazebo-unity/sensors-depth',
        'gazebo-unity/sensors-imu',
        'gazebo-unity/urdf-sdf',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 3: AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        'isaac/index',
        'isaac/isaac-setup',
        'isaac/synthetic-data',
        'isaac/vslam',
        'isaac/nav2',
        'isaac/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: '🗣️ Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'vla/index',
        'vla/whisper',
        'vla/llm-planning',
        'vla/ros-integration',
        'vla/capstone',
      ],
    },
  ],
};

export default sidebars;
