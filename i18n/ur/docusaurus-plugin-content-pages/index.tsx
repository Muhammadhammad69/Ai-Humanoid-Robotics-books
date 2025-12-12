import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Translate, {translate} from '@docusaurus/Translate';

import styles from '@site/src/pages/index.module.css';

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
              ڈوکوسورس ٹیوٹوریل - 5 منٹ ⏱️
            </Translate>
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
    translate({
      id: 'homepage.learning-outcomes.1',
      message: 'ROS 2 کا استعمال کرتے ہوئے روبوٹک کنٹرول سسٹم ڈیزائن اور نافذ کریں',
      description: 'Learning outcome 1 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.2',
      message: 'گیزبو اور یونٹی کے ساتھ حقیقی فزکس سیمولیشنز تیار کریں',
      description: 'Learning outcome 2 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.3',
      message: 'روبوٹس کے لیے AI ادراک اور فیصلہ سازی الگورتھم تیار کریں',
      description: 'Learning outcome 3 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.4',
      message: 'حقیقی دنیا کی تنصیب کے لیے سیمولیٹ سے ریل ٹرانسفر تکنیکیں لاگو کریں',
      description: 'Learning outcome 4 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.5',
      message: 'وژن، زبان، اور ایکشن کو ضم کرنے والے کوگنیٹو سسٹم تیار کریں',
      description: 'Learning outcome 5 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.6',
      message: 'روبوٹس کے لیے آواز کمانڈ انٹرفیسز اور قدرتی زبان کی پروسیسنگ نافذ کریں',
      description: 'Learning outcome 6 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.7',
      message: 'جسمانی ماحول کے ساتھ بات چیت کرنے والے ایمبیڈڈ AI سسٹم ڈیزائن کریں',
      description: 'Learning outcome 7 on homepage'
    }),
    translate({
      id: 'homepage.learning-outcomes.8',
      message: 'روبوٹکس ایپلی کیشنز کے لیے مشین لرننگ میں مہارت حاصل کریں',
      description: 'Learning outcome 8 on homepage'
    })
  ];

  // Define hardware items data structure
  const hardwareRequirements = [
    {
      id: 'workstation',
      name: translate({
        id: 'homepage.hardware.workstation.name',
        message: 'ترقی کا ورک اسٹیشن',
        description: 'Name for development workstation in hardware requirements'
      }),
      icon: '💻',
      description: translate({
        id: 'homepage.hardware.workstation.description',
        message: 'AI/ML کمپیوٹیشنز کے لیے ملٹی-کور پروسیسر اور مخصوص GPU کے ساتھ ہائی پرفارمنس کمپیوٹر',
        description: 'Description for development workstation in hardware requirements'
      }),
      detailsLink: '/docs/hardware/workstation'
    },
    {
      id: 'edge-kit',
      name: translate({
        id: 'homepage.hardware.edge-kit.name',
        message: 'ایج کمپیوٹنگ کٹ',
        description: 'Name for edge computing kit in hardware requirements'
      }),
      icon: '📦',
      description: translate({
        id: 'homepage.hardware.edge-kit.description',
        message: 'روبوٹس پر AI ماڈلز چلانے کے لیے NVIDIA جیٹسن یا اسی قسم کا ایج کمپیوٹنگ پلیٹ فارم',
        description: 'Description for edge computing kit in hardware requirements'
      }),
      detailsLink: '/docs/hardware/edge-kit'
    },
    {
      id: 'robot',
      name: translate({
        id: 'homepage.hardware.robot.name',
        message: 'ہیومنوائڈ روبوٹ',
        description: 'Name for humanoid robot in hardware requirements'
      }),
      icon: '🤖',
      description: translate({
        id: 'homepage.hardware.robot.description',
        message: 'AI الگورتھم کی ٹیسٹنگ اور تنصیب کے لیے قابل پروگرام ہیومنوائڈ روبوٹ پلیٹ فارم',
        description: 'Description for humanoid robot in hardware requirements'
      }),
      detailsLink: '/docs/hardware/robot-platform'
    }
  ];

  return (
    <Layout
      title={translate({
        id: 'homepage.layout.title',
        message: `${siteConfig.title} سے خوش آمدید`,
        description: 'Title for homepage layout'
      })}
      description={translate({
        id: 'homepage.layout.description',
        message: 'وضاحت <head /> میں میٹا ٹیگ میں جائے گی',
        description: 'Description for homepage layout'
      })}>
      {/* Hero section with gradient background */}
      <header className={styles.heroSection}>
        <div className="container">
          {/* styles.heroContent */}
          <div className={styles.heroContent}>
            <Heading as="h1" className={styles.heroTitle}>
              <Translate id="homepage.hero.title" description="Title for homepage hero section">
                فزکل AI اور ہیومنوائڈ روبوٹکس
              </Translate>
            </Heading>
            <p className={styles.heroTagline}>
              <Translate id="homepage.hero.tagline" description="Tagline for homepage hero section">
                خودمختار سسٹم کی اگلی نسل کے لیے جسمانی انٹیلی جنس کا ماسٹر بنیں
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
                      ابھی سیکھنا شروع کریں
                    </Translate>
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    <Translate id="homepage.hero.explore-modules" description="Explore modules button text in hero section">
                      ماڈیولز دریافت کریں
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
                  اس کورس کے بارے میں
                </Translate>
              </Heading>
              <p className={styles.courseDescription}>
                <Translate id="homepage.course-overview.description" description="Description for course overview section">
                  یہ جامع کورس مصنوعی ذہانت اور روبوٹکس کے جدید تقاطع کو احاطہ کرتا ہے، جسمانی ذہانت پر توجہ مرکوز کرتا ہے - کہ روبوٹس حقیقی دنیا کو کیسے سمجھتے ہیں اور اس کے ساتھ بات چیت کرتے ہیں۔ آپ جسمانی AI سسٹم تیار کرنا سیکھیں گے جو جٹیل ماحول میں سمجھ سکیں، سوچ سکیں، اور کارروائی کر سکیں۔
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
                    آپ کیا سیکھیں گے
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
                    ہارڈ ویئر کی ضروریات
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
                          تفصیلات
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
                    فزکل AI کیوں اہم ہے
                  </Translate>
                </Heading>
                <div className={styles.whyMattersContent}>
                  <p className={styles.whyMattersDescription}>
                    <Translate id="homepage.why-matters.description" description="Description for why physical AI matters section">
                      فزکل AI روایتی AI سے ایک پیراڈائم شفٹ کی نمائندگی کرتا ہے جو مبہم ڈیٹا پر کام کرتا ہے تاکہ AI کو سمجھا جا سکے اور جسمانی دنیا کے ساتھ بات چیت کی جا سکے۔ یہ جسمانی انٹیلی جنس حقیقی دنیا کے ماحول میں انسانوں کی مدد کرنے والے روبوٹس تیار کرنے کے لیے اہم ہے۔
                    </Translate>
                  </p>
                  <div className={styles.whyMattersPoints}>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-1.title" description="Title for first point in why physical AI matters section">
                          حقیقی دنیا کی ایپلی کیشن
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-1.description" description="Description for first point in why physical AI matters section">
                          فزکل AI سسٹم حقیقی دنیا کی جٹیل صورتحال سے سیکھتے ہیں اور اس کے مطابق ایڈجسٹ ہوتے ہیں، جس کے نتیجے میں زیادہ مضبوط اور عملی AI حل نکلتے ہیں۔
                        </Translate>
                      </p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-2.title" description="Title for second point in why physical AI matters section">
                          انسان-روبوٹ بات چیت
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-2.description" description="Description for second point in why physical AI matters section">
                          جس جسمانی انٹیلی جنس انسانوں اور روبوٹس کے درمیان مشترکہ جسمانی جگہوں میں قدرتی اور سمجھدار بات چیت کو فعال بناتی ہے۔
                        </Translate>
                      </p>
                    </div>
                    <div className={styles.point}>
                      <h3 className={styles.pointTitle}>
                        <Translate id="homepage.why-matters.point-3.title" description="Title for third point in why physical AI matters section">
                          جنرلائزیشن
                        </Translate>
                      </h3>
                      <p>
                        <Translate id="homepage.why-matters.point-3.description" description="Description for third point in why physical AI matters section">
                          فزکل AI سسٹم ملٹی-سینسری جسمانی تجربات سے سیکھ کر بہتر جنرلائزیشن کی صلاحیتیں تیار کرتے ہیں۔
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
                    فزکل AI اور روبوٹکس میں اپنی سفر شروع کرنے کے لیے تیار ہیں؟
                  </Translate>
                </Heading>
                <p className={styles.ctaDescription}>
                  <Translate id="homepage.cta.description" description="Description for call-to-action section">
                    جسمانی انٹیلی جنس کے مستقبل کو سیکھنے والے ہزاروں طلباء میں شامل ہوں۔
                  </Translate>
                </p>
                <div className={styles.ctaButtons}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    <Translate id="homepage.cta.start-learning" description="Start learning button text in CTA section">
                      ابھی سیکھنا شروع کریں
                    </Translate>
                  </Link>
                  <Link
                    className="button button--secondary button--lg"
                    to="/docs/modules/module-1">
                    <Translate id="homepage.cta.explore-modules" description="Explore modules button text in CTA section">
                      ماڈیولز دریافت کریں
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