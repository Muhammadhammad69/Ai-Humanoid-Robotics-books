import React from 'react';
import styles from './ChatWidget.module.css';

const ChatHeader = ({ onClose }) => {
  return (
    <div className={styles.chatHeader}>
      <div className={styles.chatTitle}>AI Assistant</div>
      <button className={styles.closeButton} onClick={onClose}>
        ×
      </button>
    </div>
  );
};

export default ChatHeader;