import React, { useState, useEffect } from 'react';
import styles from './HeroSection.module.css';

/**
 * HeroSection Component
 * A responsive hero section with image on left, content on right (desktop)
 * and stacked layout (mobile)
 *
 * @param {string} heading - Main headline text (required)
 * @param {string} description - Optional supporting text
 * @param {string} ctaText - Text for the call-to-action button (default: "Start Reading")
 * @param {string} ctaUrl - URL for the CTA button link (required)
 * @param {string} imageAlt - Alt text for the hero image (required)
 * @param {string} className - Optional additional CSS classes
 * @param {function} onCtaClick - Optional callback for CTA button click
 */
const HeroSection = ({
  heading = "AI-Powered Humanoid Robotics",
  description = "Explore the future of artificial intelligence and robotics with our comprehensive guide",
  ctaText = "Start Reading",
  ctaUrl = "/getting-started",
  imageAlt = "AI humanoid robot illustration",
  className = "",
  onCtaClick = () => {}
}) => {
  const [isImageLoaded, setIsImageLoaded] = useState(false);
  const [isHovered, setIsHovered] = useState(false);

  // Validate required props
  useEffect(() => {
    if (!ctaUrl || !imageAlt) {
      console.error('HeroSection: ctaUrl and imageAlt are required props');
    }
  }, [ctaUrl, imageAlt]);

  return (
    <section
      className={`${styles.heroSection} ${className}`}
      aria-label="Hero section with introduction to AI robotics"
      tabIndex={-1}
    >
      <div className={styles.container}>
        <div className={styles.grid}>
          {/* Image Column */}
          <div className={styles.imageColumn}>
            <div className={styles.imageContainer}>
              <img
                src="/src/assets/images/hero-illustration.png" // Placeholder - will be replaced with actual image
                alt={imageAlt}
                className={`${styles.heroImage} ${isImageLoaded ? styles.loaded : ''}`}
                loading="eager"
                onLoad={() => setIsImageLoaded(true)}
                onError={(e) => {
                  e.target.src = 'data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="%23e5e7eb"/><text x="50%" y="50%" font-family="Arial" font-size="16" fill="%236b7280" text-anchor="middle" dominant-baseline="middle">Hero Image</text></svg>';
                  setIsImageLoaded(true);
                }}
              />
              {!isImageLoaded && (
                <div className={styles.imagePlaceholder}>
                  Loading...
                </div>
              )}
            </div>
          </div>

          {/* Content Column */}
          <div className={styles.contentColumn}>
            <div className={styles.contentWrapper}>
              <h1 className={styles.heading}>
                {heading}
              </h1>
              {description && (
                <p className={styles.description}>
                  {description}
                </p>
              )}
              <a
                href={ctaUrl}
                className={`${styles.ctaButton} ${isHovered ? styles.hovered : ''}`}
                onClick={(e) => {
                  onCtaClick();
                  // Allow default navigation unless prevented
                }}
                onMouseEnter={() => setIsHovered(true)}
                onMouseLeave={() => setIsHovered(false)}
                aria-label={`Start reading about ${heading}`}
                role="button"
                tabIndex="0"
              >
                {ctaText}
              </a>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;