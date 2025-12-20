import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Translate, {translate} from '@docusaurus/Translate';
import chatbotApi from '@site/src/services/chatbot-api';


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
            <Translate id="homepage.header.tutorial-link" description="Link text for docusaurus tutorial on homepage header">
              Docusaurus Tutorial - 5min ⏱️
            </Translate>
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const backendUrl = siteConfig.customFields.backendUrl
  chatbotApi.setBackendEndpoint(backendUrl);
  console.log("Backend URL from site config:", backendUrl);

  // Define learning outcomes data structure
  const learningOutcomes = [
    translate({
      id: 'homepage.learning-outcomes.1',
      message: 'Design and implement robotic control systems using ROS 2',
      description: 'Learning outcome 1 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.2',
      message: 'Create realistic physics simulations with Gazebo and Unity',
      description: 'Learning outcome 2 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.3',
      message: 'Develop AI perception and decision-making algorithms for robots',
      description: 'Learning outcome 3 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.4',
      message: 'Apply sim-to-real transfer techniques for real-world deployment',
      description: 'Learning outcome 4 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.5',
      message: 'Build cognitive systems that integrate vision, language, and action',
      description: 'Learning outcome 5 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.6',
      message: 'Implement voice-command interfaces and natural language processing for robots',
      description: 'Learning outcome 6 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.7',
      message: 'Design embodied AI systems that interact with physical environments',
      description: 'Learning outcome 7 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.8',
      message: 'Develop skills in machine learning for robotics applications',
      description: 'Learning outcome 8 on homepage'
    })
  ];

  // Define hardware items data structure
  const hardwareRequirements = [
    {
      id: 'workstation',
      name: translate({
        id: 'homepage.hardware.workstation.name',
        message: 'Development Workstation',
        description: 'Name for development workstation in hardware requirements'
      }),
      icon: '💻',
      description: translate({
        id: 'homepage.hardware.workstation.description',
        message: 'High-performance computer with multi-core processor and dedicated GPU for AI/ML computations',
        description: 'Description for development workstation in hardware requirements'
      }),
      detailsLink: '/docs/hardware/workstation'
    },
    {
      id: 'edge-kit',
      name: translate({
        id: 'homepage.hardware.edge-kit.name',
        message: 'Edge Computing Kit',
        description: 'Name for edge computing kit in hardware requirements'
      }),
      icon: '📦',
      description: translate({
        id: 'homepage.hardware.edge-kit.description',
        message: 'NVIDIA Jetson or similar edge computing platform for running AI models on robots',
        description: 'Description for edge computing kit in hardware requirements'
      }),
      detailsLink: '/docs/hardware/edge-kit'
    },
    {
      id: 'robot',
      name: translate({
        id: 'homepage.hardware.robot.name',
        message: 'Humanoid Robot',
        description: 'Name for humanoid robot in hardware requirements'
      }),
      icon: '🤖',
      description: translate({
        id: 'homepage.hardware.robot.description',
        message: 'Programmable humanoid robot platform for testing and deployment of AI algorithms',
        description: 'Description for humanoid robot in hardware requirements'
      }),
      detailsLink: '/docs/hardware/robot-platform'
    }
  ];

  return (
    <Layout
      title={translate({
        id: 'homepage.layout.title',
        message: `Hello from ${siteConfig.title}`,
        description: 'Title for homepage layout'
      })}
      description={translate({
        id: 'homepage.layout.description',
        message: 'Description will go into a meta tag in <head />',
        description: 'Description for homepage layout'
      })}>
      {/* Hero section with gradient background */}
      <header className={styles.heroSection}>
        <div className="container">
          {/* styles.heroContent */}
          <div className={styles.heroContent}>
            <Heading as="h1" className={styles.heroTitle}>
              <Translate id="homepage.hero.title" description="Title for homepage hero section">
                Physical AI & Humanoid Robotics
              </Translate>
            </Heading>
            <p className={styles.heroTagline}>
              <Translate id="homepage.hero.tagline" description="Tagline for homepage hero section">
                Mastering embodied intelligence for the next generation of autonomous systems
              </Translate>
            </p>
            {/* <div className={styles.heroButtons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                <Translate id="homepage.hero.start-learning" description="Start learning button text in hero section">
                  Start Learning
                </Translate>
              </Link>
            </div> */}
            <div className={styles.heroButtons}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    <Translate id="homepage.hero.start-learning-now" description="Start learning now button text in hero section">
                      Start Learning Now
                    </Translate>
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    <Translate id="homepage.hero.explore-modules" description="Explore modules button text in hero section">
                      Explore Modules
                    </Translate>
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
                <Translate id="homepage.course-overview.title" description="Title for course overview section">
                  About This Course
                </Translate>
              </Heading>
              <p className={styles.courseDescription}>
                <Translate id="homepage.course-overview.description" description="Description for course overview section">
                  This comprehensive course covers the cutting-edge intersection of artificial intelligence and robotics,
                  focusing on physical intelligence - how robots understand and interact with the real world.
                  You'll learn to build embodied AI systems that can perceive, reason, and act in complex environments.
                </Translate>
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
                  <Translate id="homepage.learning-outcomes.title" description="Title for learning outcomes section">
                    What You'll Learn
                  </Translate>
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
                  <Translate id="homepage.hardware-requirements.title" description="Title for hardware requirements section">
                    Hardware Requirements
                  </Translate>
                </Heading>
                <div className={styles.hardwareGrid}>
                  {hardwareRequirements.map((item) => (
                    <div key={item.id} className={styles.hardwareItem}>
                      <div className={styles.hardwareIcon}>{item.icon}</div>
                      <h3 className={styles.hardwareName}>{item.name}</h3>
                      <p className={styles.hardwareDescription}>{item.description}</p>
                      <Link to={item.detailsLink} className={styles.hardwareLink}>
                        <Translate id="homepage.hardware-requirements.details-link" description="Details link text in hardware requirements section">
                          Details
                        </Translate>
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
                  <Translate id="homepage.why-matters.title" description="Title for why physical AI matters section">
                    Why Physical AI Matters
                  </Translate>
                </Heading>
                <div className={styles.whyMattersContent}>
                  <p className={styles.whyMattersDescription}>
                    <Translate id="homepage.why-matters.description" description="Description for why physical AI matters section">
                      Physical AI represents a paradigm shift from traditional AI that operates on abstract data to AI that
                      understands and interacts with the physical world. This embodied intelligence is crucial for creating
                      robots that can truly assist humans in real-world environments.
                    </Translate>
                  </p>
                  <div className={styles.whyMattersPoints}>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-1.title" description="Title for first point in why physical AI matters section">
                          Real-World Application
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-1.description" description="Description for first point in why physical AI matters section">
                          Physical AI systems learn from and adapt to the complexities of the real world, leading to more robust and practical AI solutions.
                        </Translate>
                      </p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-2.title" description="Title for second point in why physical AI matters section">
                          Human-Robot Interaction
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-2.description" description="Description for second point in why physical AI matters section">
                          Embodied AI enables natural and intuitive interaction between humans and robots in shared physical spaces.
                        </Translate>
                      </p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-3.title" description="Title for third point in why physical AI matters section">
                          Generalization
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-3.description" description="Description for third point in why physical AI matters section">
                          Physical AI systems develop better generalization capabilities by learning from multi-sensory physical experiences.
                        </Translate>
                      </p>
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
                  <Translate id="homepage.cta.title" description="Title for call-to-action section">
                    Ready to Start Your Journey in Physical AI & Robotics?
                  </Translate>
                </Heading>
                <p className={styles.ctaDescription}>
                  <Translate id="homepage.cta.description" description="Description for call-to-action section">
                    Join thousands of students learning the future of embodied intelligence.
                  </Translate>
                </p>
                <div className={styles.ctaButtons}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    <Translate id="homepage.cta.start-learning" description="Start learning button text in CTA section">
                      Start Learning Now
                    </Translate>
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    <Translate id="homepage.cta.explore-modules" description="Explore modules button text in CTA section">
                      Explore Modules
                    </Translate>
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
