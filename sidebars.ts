import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: "doc",
      label: "Introduction",
      id: "intro"
    },
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      link: { type: 'doc', id: 'modules/module-1/index' },
      items: [
        'modules/module-1/module-1-summary',
        'modules/module-1/chapter-1-intro-ros2',
        'modules/module-1/chapter-2-ros2-communication',
        'modules/module-1/chapter-3-python-rclpy',
        'modules/module-1/chapter-4-urdf-robot-modeling',
        'modules/module-1/chapter-5-mini-project',
        'modules/module-1/chapter-6-advanced-ros2',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — The Digital Twin (Gazebo & Unity)',
      link: { type: 'doc', id: 'modules/module-2/index' },
      items: [
        'modules/module-2/chapter-1-introduction',
        'modules/module-2/chapter-2-gazebo-physics',
        'modules/module-2/chapter-3-unity-rendering',
        'modules/module-2/chapter-4-sensor-simulation',
        'modules/module-2/chapter-5-integration-projects',
        'modules/module-2/chapter-6-advanced-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac™)',
      link: { type: 'doc', id: 'modules/module-3/index' },
      items: [
        'modules/module-3/chapter-1-intro',
        'modules/module-3/chapter-2',
        'modules/module-3/chapter-3',
        'modules/module-3/chapter-4',
        'modules/module-3/chapter-5',
        'modules/module-3/chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA)',
      link: { type: 'doc', id: 'modules/module-4/index' },
      items: [
        'modules/module-4/chapter-1-intro',
        'modules/module-4/chapter-2-whisper',
        'modules/module-4/chapter-3-llm-planning',
        'modules/module-4/chapter-4-vision',
        'modules/module-4/chapter-5-capstone',
      ],
    }
  ],
  hardwareSidebar: [
    
    {
      type: "doc",
      label: "Introduction",
      id: "hardware/intro"
    },
    {
      type: "doc",
      label: "Digital Twin Workstation Requirements",
      id: "hardware/workstation"
    },
    {
      type: "doc",
      label: "Physical AI Edge Kit",
      id: "hardware/edge-kit"
    },
    {
      type: "doc",
      label: "Robot Platform Options",
      id: "hardware/robot-platform"
    }
  ]
  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
