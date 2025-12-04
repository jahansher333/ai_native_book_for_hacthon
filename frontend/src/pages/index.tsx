import React, { JSX, useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleMouseMove = (e: MouseEvent) => {
      setMousePosition({
        x: (e.clientX / window.innerWidth) * 100,
        y: (e.clientY / window.innerHeight) * 100,
      });
    };

    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  return (
    <header className={styles.heroBanner}>
      <div
        className={styles.heroBackground}
        style={{
          backgroundPosition: `${mousePosition.x}% ${mousePosition.y}%`,
        }}
      />
      <div className={styles.heroParticles} />
      <div className={styles.heroContent}>
        <div className={styles.heroTitle}>
          <span className={styles.titleGradient}>Physical AI</span>
          <br />
          <span className={styles.titleSecondary}>& Humanoid Robotics</span>
        </div>
        <p className={styles.heroSubtitle}>
          From Digital Twins to Autonomous Humanoids
        </p>
        <p className={styles.heroDescription}>
          Master the complete Physical AI stack with hands-on learning: ROS 2, Digital Twin Simulation,
          NVIDIA Isaac, and Vision-Language-Action models
        </p>
        <div className={styles.heroButtons}>
          <Link
            className={`button button--primary button--lg ${styles.ctaButton}`}
            to="/docs/intro">
            🚀 Start Learning
          </Link>
          <Link
            className={`button button--secondary button--lg ${styles.secondaryButton}`}
            to="/docs/hardware">
            💰 $700 Hardware Kit
          </Link>
        </div>
        <div className={styles.heroStats}>
          <div className={styles.stat}>
            <div className={styles.statNumber}>13</div>
            <div className={styles.statLabel}>Weeks</div>
          </div>
          <div className={styles.stat}>
            <div className={styles.statNumber}>4</div>
            <div className={styles.statLabel}>Modules</div>
          </div>
          <div className={styles.stat}>
            <div className={styles.statNumber}>29</div>
            <div className={styles.statLabel}>Chapters</div>
          </div>
          <div className={styles.stat}>
            <div className={styles.statNumber}>$700</div>
            <div className={styles.statLabel}>Total Cost</div>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ emoji, title, description, link }: {
  emoji: string;
  title: string;
  description: string;
  link: string;
}) {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <Link
      to={link}
      className={styles.featureCard}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      style={{
        transform: isHovered ? 'translateY(-8px) scale(1.02)' : 'translateY(0) scale(1)',
      }}
    >
      <div className={styles.featureEmoji}>{emoji}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
      <div className={styles.featureArrow}>→</div>
    </Link>
  );
}

function InteractiveModules() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>
          <span className={styles.titleGradient}>Learn by Building</span>
        </h2>
        <p className={styles.sectionSubtitle}>
          Four comprehensive modules to master Physical AI and Humanoid Robotics
        </p>
        <div className={styles.featuresGrid}>
          <FeatureCard
            emoji="🤖"
            title="ROS 2 Fundamentals"
            description="Master the robotic nervous system: nodes, topics, services, actions, and URDF. Build distributed robot systems."
            link="/docs/ros2"
          />
          <FeatureCard
            emoji="🏗️"
            title="Digital Twin Simulation"
            description="Create high-fidelity simulations with Gazebo & Unity. Simulate sensors, physics, and environments."
            link="/docs/gazebo-unity"
          />
          <FeatureCard
            emoji="🧠"
            title="NVIDIA Isaac AI"
            description="Generate synthetic data, implement VSLAM & Nav2, and master sim-to-real transfer workflows."
            link="/docs/isaac"
          />
          <FeatureCard
            emoji="🗣️"
            title="Vision-Language-Action"
            description="Build voice-controlled humanoids with Whisper, LLM planning, and edge AI deployment on Jetson."
            link="/docs/vla"
          />
        </div>
      </div>
    </section>
  );
}

function WhyPhysicalAI() {
  const useCases = [
    { icon: '🏭', title: 'Manufacturing', desc: 'Autonomous assembly tasks' },
    { icon: '🏥', title: 'Healthcare', desc: 'Elder care assistants' },
    { icon: '🏠', title: 'Smart Homes', desc: 'Natural language control' },
    { icon: '🚀', title: 'Space', desc: 'Extreme environment robots' },
  ];

  return (
    <section className={styles.whySection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why Physical AI Matters</h2>
        <p className={styles.sectionSubtitle}>
          The next frontier in AI: moving beyond screens to embodied intelligence
        </p>
        <div className={styles.useCasesGrid}>
          {useCases.map((useCase, idx) => (
            <div key={idx} className={styles.useCase}>
              <div className={styles.useCaseIcon}>{useCase.icon}</div>
              <h4 className={styles.useCaseTitle}>{useCase.title}</h4>
              <p className={styles.useCaseDesc}>{useCase.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  const paths = [
    {
      title: 'Economy Jetson Kit',
      price: '$700',
      best: true,
      features: ['Jetson Orin Nano', 'RealSense D435i', 'ReSpeaker Mic', 'Complete Edge AI'],
      link: '/docs/hardware',
    },
    {
      title: 'Cloud-Native',
      price: '$205/quarter',
      best: false,
      features: ['AWS g5.xlarge', 'No upfront cost', 'Pay-as-you-go', 'Isaac Sim ready'],
      link: '/docs/hardware',
    },
    {
      title: 'Workstation',
      price: '$1,500-2,500',
      best: false,
      features: ['RTX 4070+ GPU', 'Train locally', 'No cloud costs', 'Full simulation'],
      link: '/docs/hardware',
    },
  ];

  return (
    <section className={styles.pathSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Choose Your Learning Path</h2>
        <p className={styles.sectionSubtitle}>
          Flexible options for every budget and learning style
        </p>
        <div className={styles.pathsGrid}>
          {paths.map((path, idx) => (
            <div key={idx} className={`${styles.pathCard} ${path.best ? styles.pathBest : ''}`}>
              {path.best && <div className={styles.bestBadge}>⭐ Recommended</div>}
              <h3 className={styles.pathTitle}>{path.title}</h3>
              <div className={styles.pathPrice}>{path.price}</div>
              <ul className={styles.pathFeatures}>
                {path.features.map((feature, i) => (
                  <li key={i}>✓ {feature}</li>
                ))}
              </ul>
              <Link to={path.link} className={styles.pathButton}>
                Learn More →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaSubtitle}>
          Join thousands learning to create autonomous humanoid robots
        </p>
        <div className={styles.ctaButtons}>
          <Link
            className={`button button--primary button--lg ${styles.ctaButton}`}
            to="/docs/intro">
            Start Free Course →
          </Link>
          <Link
            className={`button button--secondary button--lg ${styles.secondaryButton}`}
            to="https://github.com/jahansher333/Ai_Native_Books_Pyhsical_Ai">
            ⭐ Star on GitHub
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics - From Digital Twins to Autonomous Humanoids">
      <HomepageHeader />
      <main>
        <InteractiveModules />
        <WhyPhysicalAI />
        <LearningPath />
        <CTASection />
      </main>
    </Layout>
  );
}
