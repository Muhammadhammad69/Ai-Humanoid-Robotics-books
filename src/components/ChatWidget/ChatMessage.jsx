import React from 'react';
import styles from './ChatWidget.module.css';

const ChatMessage = ({ message, onRetry }) => {
  const { role, content, timestamp, status = 'sent', error, id } = message;

  if (status === 'error') {
    return (
      <div className={`${styles.message} ${styles.error}`}>
        <div className={styles.messageContent}>
          <span className={styles.errorMessage}>⚠️ {error || 'An error occurred'}</span>
          <button className={styles.retryButton} onClick={() => onRetry && onRetry(id)}>
            Retry
          </button>
        </div>
        {timestamp && <div className={styles.timestamp}>{timestamp}</div>}
      </div>
    );
  }

  if (status === 'pending') {
    return (
      <div className={`${styles.message} ${styles.user} ${styles.loading}`}>
        <div className={styles.messageContent}>
          {content}
          <span className={styles.loadingDots}> ● ● ●</span>
        </div>
        {timestamp && <div className={styles.timestamp}>{timestamp}</div>}
      </div>
    );
  }

  return (
    <div className={`${styles.message} ${styles[role]}`}>
      <div className={styles.messageContent}>
        {content}
      </div>
      {timestamp && <div className={styles.timestamp}>{timestamp}</div>}
    </div>
  );
};

export default ChatMessage;