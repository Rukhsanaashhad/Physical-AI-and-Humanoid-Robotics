import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
// import HomepageFeatures from '@site/src/components/HomepageFeatures'; // Removing this import
import Heading from '@theme/Heading';

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
            to="/docs/intro/index">
            Explore the Course ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

// New component for Module blocks
const ModuleBlocks: React.FC = () => {
  const modules = [
    {
      title: '01 - ROS 2: Robotic Nervous System',
      description: 'Understand the core framework for building advanced robot applications, including nodes, topics, services, and actions.',
      link: '/docs/module-1/ros2-overview',
    },
    {
      title: '02 - Digital Twin (Gazebo & Unity)',
      description: 'Explore high-fidelity simulation and realistic environments using Gazebo for physics and Unity for rendering.',
      link: '/docs/module-2/gazebo-physics',
    },
    {
      title: '03 - NVIDIA Isaac: Robot Brain',
      description: 'Dive into photorealistic simulation, synthetic data generation, and hardware-accelerated perception with NVIDIA Isaac Sim and Isaac ROS.',
      link: '/docs/module-3/isaac-sim',
    },
    {
      title: '04 - Vision-Language-Action',
      description: 'Learn how to integrate voice commands, cognitive planning with LLMs, and multimodal AI for intuitive robot control.',
      link: '/docs/module-4/whisper',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className={clsx('col col--6', styles.moduleBlock)}>
              <div className="card">
                <div className="card__header">
                  <Heading as="h3">{module.title}</Heading>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                </div>
                <div className="card__footer">
                  <Link
                    className="button button--primary button--block"
                    to={module.link}>
                    Learn More
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};


export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Documentation for the Physical AI and Humanoid Robotics Course">
      <HomepageHeader />
      <main>
        <ModuleBlocks />
      </main>
    </Layout>
  );
}

