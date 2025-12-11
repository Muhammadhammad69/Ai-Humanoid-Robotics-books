import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Docusaurus Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  // Define learning outcomes data structure
  const learningOutcomes = [
    "Design and implement robotic control systems using ROS 2",
    "Create realistic physics simulations with Gazebo and Unity",
    "Develop AI perception and decision-making algorithms for robots",
    "Apply sim-to-real transfer techniques for real-world deployment",
    "Build cognitive systems that integrate vision, language, and action",
    "Implement voice-command interfaces and natural language processing for robots",
    "Design embodied AI systems that interact with physical environments",
    "Develop skills in machine learning for robotics applications"
  ];

  // Define hardware items data structure
  const hardwareRequirements = [
    {
      id: 'workstation',
      name: 'Development Workstation',
      icon: '💻',
      description: 'High-performance computer with multi-core processor and dedicated GPU for AI/ML computations',
      detailsLink: '/docs/hardware/workstation'
    },
    {
      id: 'edge-kit',
      name: 'Edge Computing Kit',
      icon: '📦',
      description: 'NVIDIA Jetson or similar edge computing platform for running AI models on robots',
      detailsLink: '/docs/hardware/edge-kit'
    },
    {
      id: 'robot',
      name: 'Humanoid Robot',
      icon: '🤖',
      description: 'Programmable humanoid robot platform for testing and deployment of AI algorithms',
      detailsLink: '/docs/hardware/robot-platform'
    }
  ];

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      {/* Hero section with gradient background */}
      <header className={styles.heroSection}>
        <div className="container">
          {/* styles.heroContent */}
          <div className={styles.heroContent}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </Heading>
            <p className={styles.heroTagline}>
              Mastering embodied intelligence for the next generation of autonomous systems
            </p>
            {/* <div className={styles.heroButtons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Learning
              </Link>
            </div> */}
            <div className={styles.heroButtons}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    Start Learning Now
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    Explore Modules
                  </Link>
                </div>
          </div>
        </div>
      </header>

      {/* Course overview section */}
      <section className={styles.courseOverview}>
        <div className="container">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <Heading as="h2" className={styles.sectionTitle}>
                About This Course
              </Heading>
              <p className={styles.courseDescription}>
                This comprehensive course covers the cutting-edge intersection of artificial intelligence and robotics,
                focusing on physical intelligence - how robots understand and interact with the real world.
                You'll learn to build embodied AI systems that can perceive, reason, and act in complex environments.
              </p>
            </div>
          </div>
        </div>
      </section>

      <main>
        <HomepageFeatures />

        {/* Learning Outcomes Section */}
        <section className={`${styles.learningOutcomes} `}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className={styles.sectionTitle}>
                  What You'll Learn
                </Heading>
                <div className={styles.outcomesGrid}>
                  {learningOutcomes.map((outcome, index) => (
                    <div key={index} className={styles.outcomeItem}>
                      <span className={styles.checkmark}>✓</span>
                      <span className={styles.outcomeText}>{outcome}</span>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Hardware Requirements Section */}
        <section className={`${styles.hardwareRequirements} `}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className={styles.sectionTitle}>
                  Hardware Requirements
                </Heading>
                <div className={styles.hardwareGrid}>
                  {hardwareRequirements.map((item) => (
                    <div key={item.id} className={styles.hardwareItem}>
                      <div className={styles.hardwareIcon}>{item.icon}</div>
                      <h3 className={styles.hardwareName}>{item.name}</h3>
                      <p className={styles.hardwareDescription}>{item.description}</p>
                      <Link to={item.detailsLink} className={styles.hardwareLink}>
                        Details
                      </Link>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* "Why Physical AI Matters" Section */}
        <section className={`${styles.whyMattersSection}`}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className={styles.sectionTitle}>
                  Why Physical AI Matters
                </Heading>
                <div className={styles.whyMattersContent}>
                  <p className={styles.whyMattersDescription}>
                    Physical AI represents a paradigm shift from traditional AI that operates on abstract data to AI that
                    understands and interacts with the physical world. This embodied intelligence is crucial for creating
                    robots that can truly assist humans in real-world environments.
                  </p>
                  <div className={styles.whyMattersPoints}>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>Real-World Application</h3>
                      <p>Physical AI systems learn from and adapt to the complexities of the real world, leading to more robust and practical AI solutions.</p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>Human-Robot Interaction</h3>
                      <p>Embodied AI enables natural and intuitive interaction between humans and robots in shared physical spaces.</p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>Generalization</h3>
                      <p>Physical AI systems develop better generalization capabilities by learning from multi-sensory physical experiences.</p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Call-to-Action Section */}
        {/* ${styles['fade-in-element']} */}
        <section className={`${styles.ctaSection} `}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2 text--center">
                <Heading as="h2" className={styles.ctaTitle}>
                  Ready to Start Your Journey in Physical AI & Robotics?
                </Heading>
                <p className={styles.ctaDescription}>
                  Join thousands of students learning the future of embodied intelligence.
                </p>
                <div className={styles.ctaButtons}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    Start Learning Now
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    Explore Modules
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
