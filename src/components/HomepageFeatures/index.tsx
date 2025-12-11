import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Define TypeScript interfaces for our data structures
interface ModuleCardProps {
  id: string;
  title: string;
  icon: string;
  description: string;
  link: string;
  priority: number;
}

interface CTAButtonProps {
  text: string;
  link: string;
  style?: any;
  trackingId?: string;
}

interface HardwareItemProps {
  id: string;
  name: string;
  icon: string;
  description: string;
  detailsLink: string;
}

// Module Card Component
const ModuleCard: React.FC<ModuleCardProps> = ({ id, title, icon, description, link, priority }) => {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleCardHeader}>
        <span className={styles.moduleIcon}>{icon}</span>
        <h3 className={styles.moduleTitle}>{title}</h3>
      </div>
      <p className={styles.moduleDescription}>{description}</p>
      <a href={link} className={styles.moduleLink}>
        Learn More
      </a>
    </div>
  );
};

// CTA Button Component
const CTAButton: React.FC<CTAButtonProps> = ({ text, link, trackingId }) => {
  return (
    <a
      href={link}
      className={styles.ctaButton}
      data-tracking-id={trackingId}
    >
      {text}
    </a>
  );
};

// Hardware Item Component
const HardwareItem: React.FC<HardwareItemProps> = ({ id, name, icon, description, detailsLink }) => {
  return (
    <div className={styles.hardwareItem}>
      <div className={styles.hardwareIcon}>{icon}</div>
      <h4 className={styles.hardwareName}>{name}</h4>
      <p className={styles.hardwareDescription}>{description}</p>
      <a href={detailsLink} className={styles.hardwareLink}>Details</a>
    </div>
  );
};

// Main HomepageFeatures component
const HomepageFeatures: React.FC = () => {
  // Define module data structure for the four main course modules
  const modules = [
    {
      id: 'module-1',
      title: 'The Robotic Nervous System (ROS 2)',
      icon: '🤖',
      description: 'Master middleware for robot control with ROS 2 nodes, topics, and services',
      link: '/docs/modules/module-1',
      priority: 1
    },
    {
      id: 'module-2',
      title: 'The Digital Twin (Gazebo & Unity)',
      icon: '🎮',
      description: 'Build physics simulations and high-fidelity virtual environments',
      link: '/docs/modules/module-2',
      priority: 2
    },
    {
      id: 'module-3',
      title: 'The AI-Robot Brain (NVIDIA Isaac)',
      icon: '🧠',
      description: 'Advanced perception, training, and sim-to-real transfer techniques',
      link: '/docs/modules/module-3',
      priority: 3
    },
    {
      id: 'module-4',
      title: 'Vision-Language-Action (VLA)',
      icon: '🗣️',
      description: 'Integrate voice commands and LLMs for cognitive robot planning',
      link: '/docs/modules/module-4',
      priority: 4
    }
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <div className={styles.moduleGrid}>
          {modules.map((module) => (
            <ModuleCard
              key={module.id}
              id={module.id}
              title={module.title}
              icon={module.icon}
              description={module.description}
              link={module.link}
              priority={module.priority}
            />
          ))}
        </div>
      </div>
    </section>
  );
};

export default HomepageFeatures;
export { ModuleCard, CTAButton, HardwareItem };